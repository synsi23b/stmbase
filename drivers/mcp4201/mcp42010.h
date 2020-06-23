#pragma once

#include "../synhal32/synhal.h"
#include <cmath>

template<uint32_t DaisyChainLength>
class Mcp42010
{
  class Device
  {
    public:

      Device()
      {}

    void init(uint16_t potinum)
    {
      value = 0x80;
      OS_ASSERT(potinum < 2, ERR_FORBIDDEN);
      command = 0x10 | (1 << potinum);
    }

    bool set(uint8_t new_value)
    {
      bool ret = false;
      if(value != new_value)
      {
        value = new_value;
        ret = true;
      }
      return ret;
    }

    // reversed order of items because this is a little endian chip
    // and we are going to use 16bit transfer mode for the spi
    uint8_t value;
    uint8_t command;
  };
public:
  void init(uint16_t spi_port)
  {
    for(auto &d : _even)
      d.init(0);
    for(auto &d : _odd)
      d.init(1);
    _spi.init(spi_port, 5800000, false, false, true, true);
  }

  // set the value using a float array ranging from 0.0 to 1.0
  void setf(float *pbuffer, uint16_t length)
  {
    static uint8_t _buffer[DaisyChainLength * 2];
    
    for(uint16_t i = 0; i < length; ++i)
    {
      uint32_t value = std::round(255.0f * (*pbuffer++));
      _buffer[i] = value & 0xFF;
    }
    set(_buffer, length);
  }

  void set(uint8_t *pbuffer, uint16_t length)
  {
    OS_ASSERT(length % 2 == 0, ERR_FORBIDDEN);
    OS_ASSERT(length <= (DaisyChainLength * 2), ERR_BUFFER_OVERFLOW);
    int16_t end = DaisyChainLength - (length / 2);
    _set(_even, pbuffer, end);
    _set(_odd, pbuffer + 1, end);
  }
private:
  void _set(Device* devices, uint8_t* pbuffer, int16_t end)
  {
    uint16_t dirty = 0xFFFF;
    for(int16_t dev_idx = DaisyChainLength - 1; dev_idx >= end; --dev_idx)
    {
      // we fill the devices in reverse order, because actually the array is reversed.
      // when transmitting all the devices in a daisychain, we have to start with the
      // last device we want to access.
      if(devices[dev_idx].set(*pbuffer))
      {
        dirty = dev_idx;
      }
      pbuffer += 2;
    }
    if(dirty != 0xFFFF)
    {
      // dirty contains the lowest chain index that needs to be written
      // for example, if there is a chain of 5 potentiometers and dirty is 1
      // we only write the first 4 devices and skip the last one
      // therefore, dirty is the direct index to the command bytes that we will send
      uint16_t* psend = (uint16_t*)&devices[dirty];
      // and the amount of frames to send via spi is chainlength - dirty
      // for example in a chain of 5, only the first is dirty -> 5 - 4 = 1 frame
      uint16_t size = DaisyChainLength - dirty;
      _spi.busy_tx(psend, size);
    }
  }

  Device _even[DaisyChainLength];
  Device _odd[DaisyChainLength];
  syn::SpiMaster _spi;
};