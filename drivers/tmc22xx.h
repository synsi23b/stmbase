#pragma once

#include "../../synhal32/synhal.h"

class Tmc22xx
{
public:
  Tmc22xx()
  {

  }

  void init(syn::Gpio& nenable, syn::Gpio& dir, syn::Gpio& step, uint16_t timer_num, uint16_t step_channel, syn::Usart* pusart, uint8_t address)
  {
    _address = address;
    _pusart = pusart;
    nenable.set();
    nenable.mode(syn::Gpio::out_push_pull);
    _nenab = nenable;
    dir.mode(syn::Gpio::out_push_pull);
    dir.set();
    _dir = dir;
    _tim.init(timer_num);
    _tim.configStepper();
    _tim.enablePwm(step, step_channel);
  }

  void set_speed(uint16_t hz)
  {
    _tim.setStepperHz(hz);
  }

  bool r_general(uint32_t& val)
  {
    return _read_reg(val, 0x00);
  }

  bool r_status(uint32_t& val)
  {
    return _read_reg(val, 0x01);
  }

  void clear_status(uint32_t bits)
  {
    _write_reg(bits, 0x01);
  }

  bool r_ifcnt(uint32_t& val)
  {
    auto res =  _read_reg(val, 0x02);
    val &= 0xff;
    return res;
  }

  bool r_otp_bits(uint32_t& val)
  {
    return _read_reg(val, 0x05);
  }

  bool r_inputs(uint32_t& val)
  {
    return _read_reg(val, 0x06);
  }
private:
  void _crc_calc(uint8_t* data, uint16_t count)
  {
    uint16_t i,j;
    uint8_t currentByte;
    uint8_t crc = 0;
    for (i=0; i<(count-1); i++) 
    { // Execute for all bytes of a message
      currentByte = data[i]; // Retrieve a byte to be sent from Array
      for (j=0; j<8; j++)
      {
        if ((crc >> 7) ^ (currentByte & 0x01)) // update CRC based result of XOR operation
        { 
          crc = (crc << 1) ^ 0x07;
        }
        else
        {
          crc = (crc << 1);
        }
        currentByte = currentByte >> 1;
      } // for CRC bit
    } // for message byte
    *(data + (count-1)) = crc; // CRC located in last byte of message
  }

  void _write_reg(uint32_t value, uint8_t regaddress)
  {
    uint8_t data[8] = { 0x55, _address, regaddress, 0, 0, 0, 0, 0 };
    data[2] |= 0x80;
    data[3] = (value >> 24) & 0xFF;
    data[4] = (value >> 16) & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    data[6] = value & 0xFF;
    _crc_calc(data, 8);
    _pusart->write(data, 8);
  }

  uint32_t _read_reg(uint32_t& value, uint8_t regaddress)
  {
    uint8_t data[8] = { 0x55, _address, regaddress, 0, 0, 0, 0, 0 };
    _crc_calc(data, 4);
    _pusart->write(data, 4);
    _pusart->read(data, 8, 2);
    uint8_t crc = data[7];
    _crc_calc(data, 8);
    if(data[7] == crc)
    {
      uint32_t tmp = uint32_t(data[3]) << 24;
      tmp |= uint32_t(data[4]) << 16;
      tmp |= uint32_t(data[5]) << 8;
      tmp |= uint32_t(data[6]);
      value = tmp;
      return true;
    }
    return false;
  }

  syn::Timer _tim;
  syn::Gpio _dir;
  syn::Gpio _nenab;
  syn::Usart* _pusart;
  uint8_t _address;
};