#pragma once
#include "../synhal32/synhal.h"

class Zetta8k : public syn::EepromBase {

public:
  Zetta8k()
  {
  }

  void init(uint16_t i2c_port, bool remap)
  {
    _i2c.init(i2c_port, 0xA0, remap);
  }

  struct Page
  {
    uint32_t _addressblock; // will be overwritten by the page func
    union 
    {
      uint32_t ui32[4];
      uint16_t ui16[8];
      uint8_t ui8[16];
    }data;
  };
  // write the value to the address.
  // address can be anything from 0 to 1023
  // after writing, the chip needs 3ms to rewrite the page
  //bool write(uint16_t address, uint8_t value)
  //{
  //  uint8_t data[2] = {uint8_t(address & 0xFF), value};
  //  return _write(address, data, 2);
  //}

  //// write the value to the address.
  //// address can be anything from 0 to 1023
  //// if the write crosses a 16byte page border, it will fail
  //// after writing, the chip needs 3ms to rewrite the page
  //bool write(uint16_t address, uint16_t value)
  //{
  //  uint8_t data[3] = {uint8_t(address & 0xFF), uint8_t(value & 0xFF),  uint8_t(value >> 8)};
  //  return _write(address, data, 3);
  //}

  // write the value to the address.
  // address can be anything from 0 to 1023
  // if the write crosses a 16byte page border, it will fail
  // after writing, the chip needs 3ms to rewrite the page
  virtual bool write(uint16_t address, uint32_t value)
  {
    uint8_t data[5] = {uint8_t(address & 0xFF), uint8_t(value & 0xFF), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24)};
    return _write(address, data, 5);
  }

  bool write_page(uint16_t address, Page& page)
  {
    //uint8_t data[5] = {uint8_t(address & 0xFF), uint8_t(value & 0xFF), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24)};
    page._addressblock = (uint32_t(address & 0xFF)) << 24;
    uint8_t* data = ((uint8_t*)&page) + 3;
    return _write(address, data, 17);
  }

  //// read the value from the address
  //// address can be anything from 0 to 1023
  //bool read(uint16_t address, uint8_t& value)
  //{
  //  return _read(address, &value, 1);
  //}

  //// read the value from the address
  //// address can be anything from 0 to 1023
  //bool read(uint16_t address, uint16_t& value)
  //{
  //  return _read(address, (uint8_t*)&value, 2);
  //}

  // read the value from the address
  // address can be anything from 0 to 1023
  virtual bool read(uint16_t address, uint32_t& value)
  {
    return _read(address, (uint8_t*)&value, 4);
  }

  bool read_page(uint16_t address, Page& page)
  {
    //uint8_t data[5] = {uint8_t(address & 0xFF), uint8_t(value & 0xFF), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24)};
    //page._addressblock = (uint32_t(address & 0xFF)) << 24;
    uint8_t* data = ((uint8_t*)&page) + 4;
    return _read(address, data, 16);
  }
private:
  bool _write(uint16_t address, uint8_t* data, uint16_t size)
  {
    if(((address % 16) + size) > 17)
    {
      // writing over page border
      OS_ASSERT(true == false, ERR_NOT_IMPLMENTED);
      return false;
    }
    uint8_t addrmsb = 0xA0 | ((address & 0x300) >> 7);
    _i2c.overwrite_address(addrmsb);
    return _i2c.write(data, size);
  }

  bool _read(uint16_t address, uint8_t* values, uint16_t size)
  {
    uint8_t addrlsb = address & 0xFF;
    if(_write(address, &addrlsb, 1))
    {
      return _i2c.read(values, size);
    }
    return false;
  }

  syn::I2cMaster _i2c;
};