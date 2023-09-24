#pragma once

#include "../synhal32/synhal.h"

class AS5600
{
public:
  AS5600()
  {
    _angle = -1;
    _status = 0;
  }

  // initialize state and i2c communication
  // set the update interval of the status register per call to update.
  // e.G. set it to 10 and status gets read every 10th update call
  // this is to expedite the angle reading, since we dont have to transmit the
  // angle register address for every read.
  void init(uint16_t i2c_num, bool i2c_remap, uint8_t status_update_interval)
  {
    _angle = 0;
    _status_interval = status_update_interval;
    _current_interval = 1;
    _status = 0;
    _i2c.init(i2c_num, 0x36 << 1, i2c_remap);
    if(_update_status())
    {
      _configure();
      // reset the read address to expected raw angle after configure call
      uint8_t data = 0x0C;
      if(_i2c.write(&data, 1) == true)
      {
        _current_interval = status_update_interval;
      }
    }
  }

  // run regularily to update angle and state
  // state gets updated every status_update_interval ticks
  // if the state update fails for any reason, it will be attempted at the next tick again.
  void update()
  {
    if(--_current_interval == 0)
    {
      _current_interval = _update_status();
    }
    else
    {
      _update_angle();
    }
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
  // 0x20 == magnet present
  // 0x01 == magnet weak
  // 0x08 == magnet strong
  // 0x80 == i2c could not read status register / bus failure
  uint8_t status () const
  {
    return _status;
  }

  bool magnet_ok() const
  {
    return _status == 0x20;
  }

  bool magnet_present() const
  {
    return _status & 0x20;
  }

  bool magnet_weak() const
  {
    return _status & 0x10;
  }

  bool magnet_strong() const
  {
    return _status & 0x08;
  }

  bool i2c_failure() const
  {
    return _status & 0x80;
  }

  uint16_t read_conf()
  {
    uint8_t data[2] = {0x07, 0};
    if(_i2c.write(data, 1) && _i2c.read(data, 2))
    {
      return uint16_t(data[0]) << 8 | uint16_t(data[1]);
    }
    return 0xFFFF;
  }
private:
  uint8_t _update_status()
  {
    uint8_t next_interval = 1;
    // read status and angle
    uint8_t data[3] = {0x0B, 0, 0};
    if(_i2c.write(data, 1) && _i2c.read(data, 3))
    {
      OS_TIME t_measure = OS_TIME_Get_us();
      // Anding really neccessairy? Yes it is.
      _status = data[0] & 0x38;
      if(magnet_present())
      {
        _timestamp = t_measure;
        _angle = uint16_t(data[1]) << 8 | uint16_t(data[2]);
        // set read address to raw angle for next angle update to skip address transmission
        // only if status is not to be read out everytime
        if(_status_interval > 1)
        {
          data[0] = 0x0C;
          if(_i2c.write(data, 1) == true)
          {
            next_interval = _status_interval;
          }
        }
      }
      else
      {
        _angle = -1;
      }
    }
    else
    {
      _status = 0x80;
      _angle = -1;
    }
    return next_interval;
  }

  void _configure()
  {
    // reg 0x07
    // FTH 010b -> 7 / 1 threshold
    // SF 01b -> 1.1ms settling time
    // reg 0x08
    // HYST 10b -> 2 LSB
    uint8_t data[3] = {0x07, 0x05, 0x04};
    if(!_i2c.write(data, 3))
    {
      // failed to configure, try again in 10ms
      syn::Thread::sleep(10);
      _i2c.write(data, 3);
    }
  }

  void _update_angle()
  {
    uint8_t data[2];
    // address should be set to raw angle already by update method
    if(_i2c.read(data, 2))
    {
      _timestamp = OS_TIME_Get_us();
      _angle = uint16_t(data[0]) << 8 | uint16_t(data[1]);
    }
    else
    {
      _angle = -1;
    }
  }

  syn::I2cMaster _i2c;
  uint32_t _timestamp; // the actual stamp is u64, but u32 is more than enough
  uint16_t _angle;
  uint8_t _status_interval;
  uint8_t _current_interval;
  uint8_t _status;
};

// run a AS5600 internally, but keep track of the position when its possible to
// cross the zero point multiple times
class AS5600Multiturn
{
public:
  AS5600Multiturn()
  {}

  // initialize state and i2c communication
  // set the update interval of the status register per call to update.
  // e.G. set it to 10 and status gets read every 10th update call
  // this is to expedite the angle reading, since we dont have to transmit the
  // angle register address for every read.
  // the direction parameter controls wether the sensor counts up or down in a specific direction
  // this can also be selected via pin on the chip directly, might be the better solution in the long run
  void init(uint16_t i2c_num, bool i2c_remap, uint8_t status_update_interval)
  {
    _sensor.init(i2c_num, i2c_remap, status_update_interval);
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
  uint8_t status() const
  {
    return _sensor.status();
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
    _sensor.update();
    if(_sensor.magnet_present())
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
      if(change > 2047)
      {
        change = 4095 - change;
      }
      else if (change < -2047)
      {
        change = 4095 + change;
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

  uint16_t read_conf()
  {
    return _sensor.read_conf();
  }
private:
  AS5600 _sensor;
  int32_t _pos;
  int32_t _speed;
  uint32_t _prev_timestamp;
  int16_t _prev_angle;
};