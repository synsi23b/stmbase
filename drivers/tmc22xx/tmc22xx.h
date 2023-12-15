#pragma once

#include "../../synhal32/synhal.h"

class Tmc22xx
{
public:
  class GCONF {
  public:
    static const uint32_t I_scale_analog = (1 << 0);
    static const uint32_t internal_Rsense = (1 << 1);
    static const uint32_t en_SpreadCycle = (1 << 2);
    static const uint32_t shaft = (1 << 3);
    static const uint32_t index_otpw = (1 << 4);
    static const uint32_t index_step = (1 << 5);
    static const uint32_t pdn_disable = (1 << 6);
    static const uint32_t mstep_reg_select = (1 << 7);
    static const uint32_t multistep_filt = (1 << 8);
  };

  Tmc22xx()
  {

  }

  bool init(syn::Gpio& nenable, syn::Gpio& dir, 
            uint16_t timer_num, uint16_t step_channel,
            int8_t step_pin_port, uint8_t step_pin_num,
            uint16_t usart_num, uint8_t address,
            bool reverse_motor)
  {
    bool ret = false;
    _acceleration = 1;
    if(usart_num != 0)
    {
      _usart.init(usart_num, syn::Usart::b230400, true);
      _address = address;
      write_reg(4 << 8, 0x03); // set answer delay on read reg to 5 * 8 bit times
      // set chopconf step on both edges, 
      //write_reg(0x10010053 | (1 << 29), 0x6C);
      ret = set_motor_reverse(reverse_motor);
    }
    nenable.set();
    nenable.mode(syn::Gpio::out_push_pull);
    _nenab = nenable;
    dir.mode(syn::Gpio::out_push_pull);
    dir.set();
    _dir = dir;
    _timramp.init(timer_num, 256, 5120); // 5120 minimum speed in Hertz. 1 / 20th rotation per second
    _timramp.enable_pwm(step_pin_port, step_pin_num, step_channel);
    return ret;
  }

  void set_acceleration(uint16_t acceleration)
  {
    if(acceleration == 0)
      acceleration = 1;
    _acceleration = acceleration;
  }

  void set_speed(int32_t hz)
  {
    if(hz < 0)
    {
      _dir.clear();
      hz = -hz;
    }
    else if (hz > 0)
    {
      _dir.set();
    }
    _timramp.linear(hz);
  }

  bool stopped() const
  {
    return _timramp.current_hz() == 0;
  }

  bool target_reached() const
  {
    return _timramp.target_reached();
  }

  void enable(bool state = true)
  {
    _nenab.set(!state);
  }

  bool set_motor_reverse(bool true_reverse)
  {
    uint32_t val = true_reverse ? GCONF::shaft : 0;
    uint32_t gconf = GCONF::I_scale_analog | GCONF::pdn_disable | GCONF::mstep_reg_select | GCONF::multistep_filt | val;
    return s_gconf_check(gconf);
  }

  void s_gconf(uint32_t val)
  {
    write_reg(val, 0x00);
  }

  bool r_gconf(uint32_t& val)
  {
    return read_reg(val, 0x00);
  }

  bool s_gconf_check(uint32_t val)
  {
    s_gconf(val);
    uint32_t check;
    if(r_gconf(check) && check == val)
      return true;
    // failed to write and read back general config, try again
    syn::Thread::sleep(1);
    s_gconf(val);
    return (r_gconf(check) && check == val);
  }

  bool r_status(uint32_t& val)
  {
    return read_reg(val, 0x01);
  }

  void clear_status(uint32_t bits)
  {
    write_reg(bits, 0x01);
  }

  bool r_ifcnt(uint32_t& val)
  {
    auto res =  read_reg(val, 0x02);
    val &= 0xff;
    return res;
  }

  bool r_otp_bits(uint32_t& val)
  {
    return read_reg(val, 0x05);
  }

  bool r_inputs(uint32_t& val)
  {
    return read_reg(val, 0x06);
  }

  bool r_sg_result(uint32_t& val)
  {
    return read_reg(val, 0x41);
  }

  void write_reg(uint32_t value, uint8_t regaddress)
  {
    uint8_t data[8] = { 0x55, _address, regaddress, 0, 0, 0, 0, 0 };
    data[2] |= 0x80;
    data[3] = (value >> 24) & 0xFF;
    data[4] = (value >> 16) & 0xFF;
    data[5] = (value >> 8) & 0xFF;
    data[6] = value & 0xFF;
    _crc_calc(data, 8);
    _usart.write(data, 8);
  }

  bool read_reg(uint32_t& value, uint8_t regaddress)
  {
    uint8_t data[12] = { 0x55, _address, regaddress, 0 };
    _crc_calc(data, 4);
    _usart.reset();
    _usart.write(data, 4);
    if(_usart.read(data, 12, 2) != 12)
      return false;
    uint8_t crc = data[11];
    _crc_calc(data+4, 8);
    if(data[11] == crc)
    {
      uint32_t tmp = uint32_t(data[7]) << 24;
      tmp |= uint32_t(data[8]) << 16;
      tmp |= uint32_t(data[9]) << 8;
      tmp |= uint32_t(data[10]);
      value = tmp;
      return true;
    }
    return false;
  }

  void dma_isr(uint16_t irq_stat)
  {
    _timramp.isr(irq_stat);
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

  syn::TimerRamper _timramp;
  syn::Gpio _dir;
  syn::Gpio _nenab;
  syn::Usart _usart;
  uint16_t _acceleration;
  uint8_t _address;
};