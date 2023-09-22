#pragma once

#include "../synhal32/synhal.h"

class AS5600
{
public:
  AS5600()
  {
    _angle = 0xFFFF;
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
    _current_interval = 0;
    _status = 0;
    _i2c.init(i2c_num, 0x36 << 1, i2c_remap);
    if(_update_status())
    {
      _current_interval = status_update_interval;
    }
    else
    {
      _current_interval = 1;
    }
  }

  // run regularily to update angle and state
  // state gets updated every status_update_interval ticks
  // if the state update fails for any reason, it will be attempted at the next tick again.
  void update()
  {
    if(--_current_interval == 0)
    {
      if(!_update_status())
      {
        _current_interval = 1;
      }
    }
    else
    {
      _update_angle();
    }
  }

  // returns the read out angle in the range 0 to 4095
  // returns 0xFFFF on error (no magnet present, i2c error)
  uint16_t angle() const
  {
    return _angle;
  }

  // returns the raw bitwise status word of the sensor
  // 0x20 == magnet present
  // 0x01 == magnet weak
  // 0x08 == magnet strong
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
private:
  bool _update_status()
  {
    // read status and angle
    uint8_t data[3] = {0x0B, 0, 0};
    if(_i2c.write(data, 1) && _i2c.read(data, 3))
    {
      // TODO Anding really neccessairy?
      _status = data[0] & 0x38;
      if(magnet_present())
      {
        _angle = uint16_t(data[1]) << 8 | uint16_t(data[2]);
        // set read address to raw angle for next angle update to skip address transmission
        data[0] = 0x0C;
        if(_i2c.write(data, 1) == true)
        {
          _current_interval = _status_interval;
        }
      }
      else
      {
        _angle = 0xFFFF;
      }
    }
    else
    {
      _status = 0;
      _angle = 0xFFFF;
    }
    return _current_interval > 0;
  }

  void _update_angle()
  {
    uint8_t data[2];
    // address should be set to raw angle already by update method
    if(_i2c.read(data, 2))
    {
      _angle = uint16_t(data[0]) << 8 | uint16_t(data[1]);
    }
    else
    {
      _angle = 0xFFFF;
    }
  }

  syn::I2cMaster _i2c;
  uint16_t _angle;
  uint8_t _status_interval;
  uint8_t _current_interval;
  uint8_t _status;
  //uint8_t _gain;
  //uint16_t _magnitude;
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
  void init(uint16_t i2c_num, bool i2c_remap, uint8_t status_update_interval)
  {
    _sensor.init(i2c_num, i2c_remap, status_update_interval);
    _pos = 0;
    int16_t angle = _sensor.angle();
    // possibly the value is 0xFFFF on sensor failure
    // if the sensor failed, set previous to 0
    // the position will jump when it was possible to read the sensor
    // else the current good reading marks the zero-point
    if(angle = 0xFFFF)
      _prev_angle = 0;
    else
      _prev_angle = angle;
  }

  // returns the raw bitwise status word of the sensor
  // 0x20 == magnet present
  // 0x01 == magnet weak
  // 0x08 == magnet strong
  uint8_t status()
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
  void update()
  {
    _sensor.update();
    if(_sensor.magnet_present())
    {
      // TODO confirm sensor behavior on weak / strong / no magnet or test for magnet_good instead of present
      // get the current angle
      int16_t angle = _sensor.angle();
      // calculate the change between the values
      // angle > _prev --> positive value
      // angle < prev --> negative value
      // angle == prev --> zero value
      int32_t change = angle - _prev_angle;
      _pos += change;
      _prev_angle = angle;
    }
  }

  int32_t position()
  {
    return _pos;
  }
private:
  AS5600 _sensor;
  int32_t _pos;
  int16_t _prev_angle;
};