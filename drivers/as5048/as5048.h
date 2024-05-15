#pragma once

#include "../synhal32/synhal.h"

class AS5048
{
public:
  AS5048()
  {
    //_angle = -1;
    //_status = 0;
    //_errorcounter = 0;
    //_configured = false;
    _reversed = false;
  }

  // initialize state and spi communication
  // set the update interval of the status register per call to update.
  // e.G. set it to 10 and status gets read every 10th update call
  // this is to expedite the angle reading, since we dont have to transmit the
  // angle register address for every read.
  void init(uint16_t spi_num)
  {
    _angle = -1;
    _status = 0;
    _errorcounter = 0;
    _spi.init(spi_num, 8000000, false, true, true, true);
    update();
    syn::Thread::usleep(50);
    update();
  }

  // run regularily to update angle and state
  // state gets updated every status_update_interval ticks
  // if the state update fails for any reason, it will be attempted at the next tick again.
  // keep the minimum time between cs low and first clock edge using setup_delay, measure and adapt
  bool update(uint16_t setup_delay = 10)
  {
    // clear error, angle, diagnostics, 
    uint16_t command[3] = {  0x4000|0x0001, 0xC000|0x3FFF, 0x4000|0x3FFD }; //  0x4000|0x0001};
    uint32_t tstamp = OS_TIME_Get_us();
    for(int i = 0; i < 3; ++i)
    {
      _spi.busy_bidi(command + i, 1, setup_delay);
      // keep the minimum time between 2 commands (cs high)
      if(i < 2 && setup_delay > 0)
      {
        uint16_t x = setup_delay * 7;
        while(x--)
        {
          syn::System::nop();
        }
      }
    }
    // read the data check for errors
    _status = command[0];
    if(_status & 0x4000)
    {
      ++_errorcounter;
      return false;
    }
    uint16_t ang = command[2];
    if(ang & 0x4000)
    {
      ++_errorcounter;
      return false;
    }
    // check parity
    int16_t angle = ang & 0x3FFF;
    uint16_t count = 0;
    while (ang) {
        count += ang & 1;
        ang >>= 1;
    }
    if(count & 1)
    {
      ++_errorcounter;
      return false;
    }
    if(_reversed)
      _angle = 0x3FFF - angle;
    else
      _angle = angle;
    _timestamp = tstamp;
    return true;
  }

  // set the incrementing direction of the sensor programatically
  void set_reverse(bool rev)
  {
    if(_reversed == rev)
      return;
    _reversed = rev;
    if(_angle != -1)
    {
      _angle = 0x3FFF - _angle;
    }
  }

  bool get_reversed() const
  {
    return _reversed;
  }

  // returns the read out angle in the range 0 to 4095
  // returns -1 on error (no magnet present, i2c error)
  int16_t angle() const
  {
    return _angle;
  }

  uint32_t timestamp() const
  {
    return _timestamp;
  }

  // returns the raw bitwise status word of the sensor
  uint8_t status () const
  {
    return _status;
  }

  bool magnet_ok() const
  {
    return (_status & 0x0F00) == 0x0100;
  }

  bool magnet_present() const
  {
    return (_status & 0xFF) < 0xFF;
  }

  bool magnet_weak() const
  {
    return _status & 0x0800;
  }

  bool magnet_strong() const
  {
    return _status & 0x0400;
  }

private:
  syn::SpiMaster _spi;
  uint32_t _timestamp; // the actual stamp is u64, but u32 is more than enough
  int16_t _angle;
  uint16_t _status;
  uint32_t _errorcounter;
  bool _reversed;
};

// run a AS5048 internally, but keep track of the position when its possible to
// cross the zero point multiple times
class AS5048Multiturn
{
public:
  AS5048Multiturn()
  {}

  // initialize state and i2c communication
  // set the update interval of the status register per call to update.
  // e.G. set it to 10 and status gets read every 10th update call
  // this is to expedite the angle reading, since we dont have to transmit the
  // angle register address for every read.
  // the direction parameter controls wether the sensor counts up or down in a specific direction
  // this can also be selected via pin on the chip directly, might be the better solution in the long run
  void init(uint16_t spi_num)
  {
    _sensor.init(spi_num);
    _prev_timestamp = 0;
    _pos = 0;
    _speed = 0;
    int16_t angle = _sensor.angle();
    // possibly the value is -1 on sensor failure
    // if the sensor failed, set previous to 0
    // the position will jump when it was possible to read the sensor
    // else the current good reading marks the zero-point
    if(angle == -1)
      _prev_angle = 0;
    else
      _prev_angle = angle;
  }

  // returns the raw bitwise status word of the sensor
  // 0x20 == magnet present
  // 0x01 == magnet weak
  // 0x08 == magnet strong
  // 0x80 == i2c could not read status register / bus failure
  uint8_t status() const
  {
    return _sensor.status();
  }

  bool magnet_ok() const
  {
    return _sensor.magnet_ok();
  }

  bool magnet_present() const
  {
    return _sensor.magnet_present();
  }

  void set_reverse(bool rev)
  {
    _sensor.set_reverse(rev);
  }

  bool get_reversed() const
  {
    return _sensor.get_reversed();
  }

  // reset the global position
  void reset_position(int32_t pos)
  {
    _pos = pos;
    _prev_angle = _sensor.angle();
  }

  // run regularily to update angle and state
  // state gets updated every status_update_interval ticks
  // if the state update fails for any reason, it will be attempted at the next tick again.
  bool update()
  {
    if(_sensor.update() && _sensor.magnet_present())
    {
      // TODO confirm sensor behavior on weak / strong / no magnet or test for magnet_good instead of present
      // get the current angle
      int16_t angle = _sensor.angle();
      if(angle == -1)
        return false;
      // calculate the change between the values
      // angle > _prev --> positive value or backwards with 0 point crossing
      // angle < prev --> negative value or forward with 0 point crossing
      // angle == prev --> zero value
      int32_t change = angle - _prev_angle;
      _prev_angle = angle;
      // if the rate of change per update is bigger than half of the sensor resolution, we cant tell what happened
      // reading one sensor takes about 0.3ms when reading the status, too
      // but limiting to an update rate of 100Hz limits the maximum RPS to 50, or 3000 RPM, which is okay for steppers
      // than again, running a PID loop that mainly reads just the raw angle, could go over 1000Hz easily
      if(change > 8192)
      {
        change = 16383 - change;
      }
      else if (change < -8192)
      {
        change = 16383 + change;
      }
      _pos += change;

      uint32_t elapsed_us = _sensor.timestamp() - _prev_timestamp;
      _prev_timestamp = _sensor.timestamp();
      int32_t sptmp = _speed * 15;
      sptmp += (change * 1000000) / int32_t(elapsed_us);
      _speed = sptmp / 16;

      return true;
    }
    return false;
  }

  int32_t position() const
  {
    return _pos;
  }

  int32_t speed() const
  {
    return _speed;
  }

  uint32_t timestamp() const
  {
    return _sensor.timestamp();
  }

private:
  AS5048 _sensor;
  int32_t _pos;
  int32_t _speed;
  uint32_t _prev_timestamp;
  int16_t _prev_angle;
};