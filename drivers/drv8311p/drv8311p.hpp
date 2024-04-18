#pragma once

#include "../synhal32/synhal.h"

class Drv8311p {
public:
  Drv8311p()
  {
  }

  void init(uint16_t spi_port, uint8_t address, char csel = 'x', uint8_t pin = 0)
  {
    (void)pin;
    _address = address;
    _spi.init(spi_port, 4000000, false, false, true, csel == 'x');
    if(csel != 'x')
      OS_ASSERT(true == false, ERR_NOT_IMPLMENTED);
  }

  // if the value is negative (bit 15 set), read failed
  int16_t get_status()
  {
    return _read_reg(0);
  }

  int16_t get_temp_status()
  {
    return _read_reg(4);
  }

  int16_t get_supply_status()
  {
    return _read_reg(5);
  }

  int16_t get_driver_status()
  {
    return _read_reg(6);
  }

  int16_t get_system_status()
  {
    return _read_reg(7);
  }

  int16_t get_sync_period()
  {
    return _read_reg(0xC);
  }

  int16_t get_fault_mode()
  {
    return _read_reg(0x10);
  }

  int16_t get_sys_ctrl()
  {
    return _read_reg(0x12);
  }

  int16_t get_drv_ctrl()
  {
    return _read_reg(0x13);
  }

  int16_t get_fault_tctrl()
  {
    return _read_reg(0x16);
  }

  bool clear_faults()
  {
    return _write_reg(0x17, 0x0001);
  }

  bool write_pwm_period(uint16_t value)
  {
    return _write_reg(0x18, value & 0x0FFF);
  }

  bool write_pwms(uint16_t a, uint16_t b, uint16_t c)
  {
    return _write_reg(0x18, value & 0x0FFF);
  }

  uint8_t status()
  {
    return _status;
  }
private:
  bool _parity_check(uint16_t data)
  {
    uint16_t count = 0;
    while (data) {
        count += data & 1;
        data >>= 1;
    }
    return (count & 1) == 0;
  }

  // if the value is negative (bit 15 set), read failed
  int16_t _read_reg(uint16_t address)
  {
    address <<= 3;
    // set rw and secondary address
    address |= 0x8000 | (_address << 11);
    // check parity, if it fails, set header parity bit
    if(!_parity_check(address))
      address |= 0x0001;
    // compose the command and run bidiractional spi
    uint16_t command[2] = { address , 0xCCCC };
    if(_spi.busy_bidi(command, 2))
    {
      // check the parity of the returned value
      if(_parity_check(command[1]))
      {
        _status = uint8_t(command[0] & 0xFF);
        return command[1] & 0x7FFF;
      }
    }
    return -1;
  }

  bool _write_reg(uint16_t address, uint16_t value)
  {
    address <<= 3;
    // set secondary address
    address |= (_address << 11);
    // check parity, if it fails, set header parity bit
    if(!_parity_check(address))
      address |= 0x0001;
    value = value & 0x7FFF;
    if(!_parity_check(value))
      value |= 0x8000;
    // compose the command and run bidiractional spi
    uint16_t command[2] = { address , value };
    if(_spi.busy_bidi(command, 2))
    {
      // check the parity of the returned value
      if(_parity_check(command[1]))
      {
        _status = uint8_t(command[0] & 0xFF);
        return true;
      }
    }
    return false;
  }

  bool _write_regs(uint16_t* values, uint16_t size)
  {

    // shift reg address in place
    values[0] <<= 3;
    // set secondary address
    values[0] |= (_address << 11);
    // check parity, if it fails, set header parity bit
    if(!_parity_check(values[0]))
      values[0] |= 0x0001;
    // set the data values
    uint16_t* ptr = values + 1;
    uint16_t* ptre = values + size;
    while(ptr != ptre)
    {
      uint16_t val = *ptr & 0x7FFF;
      if(!_parity_check(val))
        val |= 0x8000;
      *ptr++ = val;
    }
    if(_spi.busy_bidi(values, size))
    {
      if(size > 1)
      {
        // check the parity of the returned value
        if(_parity_check(command[1]))
        {
          _status = uint8_t(values[0] & 0xFF);
          return true;
        }
      }
      else
      {
        _status = uint8_t(values[0] & 0xFF);
        return true;
      }
    }
    return false;
  }

  bool _write_3_regs(uint16_t address, uint16_t a, uint16_t b, uint16_t c)
  {
    address <<= 3;
    // set rw and secondary address
    address |= 0x8000 | (_address << 11);
    // check parity, if it fails, set header parity bit
    if(!_parity_check(address))
      address |= 0x0001;
    
    value = value & 0x7FFF;
    if(!_parity_check(value))
      value |= 0x8000;
    // compose the command and run bidiractional spi
    uint16_t command[4] = { address , a, b, c };
    if(_spi.busy_bidi(command, 2))
    {
      // check the parity of the returned value
      if(_parity_check(command[1]))
      {
        _status = uint8_t(command[0] & 0xFF);
        return true;
      }
    }
    return false;
  } 

  syn::SpiMaster _spi;
  uint8_t _address;
  uint8_t _status;
};