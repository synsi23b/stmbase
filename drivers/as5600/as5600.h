#pragma once

#include "../synhal32/synhal.h"

class AS5600
{
public:
  AS5600()
  {

  }

  void init(uint16_t i2c_num)
  {
    uint8_t addr = 0x0B;
    _angle = 0;
    _status = 0;
    _i2c.init(i2c_num, 0x36 << 1);
    OS_ASSERT(_i2c.write(&addr, 1) == true, ERR_DEVICE_NOT_ENABLED);
  }

  void update()
  {
    // read angle and status
    uint8_t data[3];
    // set address to status
    data[0] = 0x0B;
    _i2c.write(data, 1);
    // read status, raw angle, scaled angle
    data[0] = 0x0E;
    _i2c.read(data, 3);
    _status = data[0];
    _angle = uint16_t(data[1]) << 8 | uint16_t(data[2]);
    
  }

  uint16_t angle() const
  {
    return _angle;
  }

  uint8_t status () const
  {
    return _status;
  }
private:
  syn::I2cMaster _i2c;
  uint16_t _angle;
  uint8_t _status;
};