#pragma once

#include "../synhal32/synhal.h"

class AS5600
{
public:
  AS5600()
  {

  }

  void init(uint16_t i2c_num, uint8_t status_update_interval)
  {
    _angle = 0;
    _status_interval = status_update_interval;
    _current_interval = 0;
    _status = 0;
    _i2c.init(i2c_num, 0x36 << 1);
    if(_update_status())
    {
      _current_interval = status_update_interval;
    }
    else
    {
      _current_interval = 1;
    }
  }

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

  uint16_t angle() const
  {
    return _angle;
  }

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
      _status = data[0] & 0x38;
      if(magnet_present())
      {
        _angle = uint16_t(data[0]) << 8 | uint16_t(data[1]);
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
    // address should be set to raw angle alreadu by update method
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
  uint8_t _gain;
  uint16_t _magnitude;
};