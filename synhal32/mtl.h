#pragma once

#ifdef VSCODE_ONLY
#include <stdint-gcc.h>
#else
#include <stdint.h>
#endif

#include <cstring> // memcopy
#include <new>

namespace mtl
{
  template <typename T>
  inline void set_bit_band(T *base, uint16_t bitnum, uint16_t value)
  {
    uint32_t b = (uint32_t)base;
    if (bitnum > 23)
    {
      b += 3;
      bitnum -= 24;
    }
    else if (bitnum > 15)
    {
      b += 2;
      bitnum -= 16;
    }
    else if (bitnum > 8)
    {
      b += 1;
      bitnum -= 8;
    }
    if (b < 0x40000000)
    {
      // address is in sram
      b -= 0x20000000;
      *(uint32_t *)(0x22000000 + 32 * b + bitnum * 4) = value;
    }
    else
    {
      // address is peripheral
      b -= 0x40000000;
      *(uint32_t *)(0x42000000 + 32 * b + bitnum * 4) = value;
    }
  }

  template <typename T>
  inline bool get_bit_band(T *base, uint16_t bitnum)
  {
    uint32_t b = (uint32_t)base;
    if (bitnum > 23)
    {
      b += 3;
      bitnum -= 24;
    }
    else if (bitnum > 15)
    {
      b += 2;
      bitnum -= 16;
    }
    else if (bitnum > 8)
    {
      b += 1;
      bitnum -= 8;
    }
    if (b < 0x40000000)
    {
      // address is in sram
      b -= 0x20000000;
      return *(uint32_t *)(0x22000000 + 32 * b + bitnum * 4);
    }
    else
    {
      // address is peripheral
      b -= 0x40000000;
      return *(uint32_t *)(0x42000000 + 32 * b + bitnum * 4);
    }
  }

  inline void IntToUnicode(uint32_t value, uint8_t *pbuf, uint16_t len)
  {
    uint16_t idx = 0;

    for (idx = 0; idx < len; idx++)
    {
      uint32_t digit = value >> 28;
      if (digit < 10)
      {
        pbuf[2 * idx] = digit + '0';
      }
      else
      {
        pbuf[2 * idx] = digit + 'A' - 10;
      }

      value = value << 4;

      pbuf[2 * idx + 1] = 0;
    }
  }

  inline char *IntToStr(int16_t val, char *pbuf)
  {
    if (val == 0)
    {
      *pbuf++ = '0';
    }
    else
    {
      int32_t bigval = val;
      if (val < 0)
      {
        *pbuf++ = '-';
        bigval = -bigval;
      }
      uint16_t r_val = bigval;
      uint16_t denom = 10000; // 16 bit integer can only go up to 32k
      uint32_t r;
      while (true)
      {
        r = r_val / denom;
        if (r != 0)
          break;
        denom /= 10;
      }
      while (true)
      { // r contains the valid number
        // denom contais the denom that created that
        *pbuf++ = '0' + r;
        r_val -= r * denom;
        if (denom == 1)
          break;
        denom /= 10;
        r = r_val / denom;
      }
    }
    return pbuf;
  }

  template <typename Value_t, uint32_t Size>
  class Ringbuffer
  {
  public:
    Ringbuffer()
    {
      _readptr = _writeptr = _buffer;
      _count = 0;
    }

    uint32_t in_avail() const
    {
      return _count;
    }

    uint32_t remainingSpace() const
    {
      return Size - _count;
    }

    bool full() const
    {
      return _count == Size;
    }

    // look at some fancy values in the future
    bool peek(Value_t &val, uint32_t offset = 0) const
    {
      if (offset < Size && offset < _count)
      {
        // check if we can just add offset to readpointer, else we have to wrap around to the start
        Value_t *ptr = _readptr + offset;
        if (ptr < &_buffer[Size])
          val = *ptr;
        else
          val = _buffer[ptr - (&_buffer[Size])];
        return true;
      }
      return false;
    }

    // remove up to count values from the buffer
    void flush(uint32_t count = Size)
    {
      uint32_t ct = _count;
      if (count >= _count)
      {
        _readptr = _writeptr = _buffer;
        ct = 0;
      }
      else
      { // don't wipe everything
        Value_t *ptr = _readptr + count;
        if (ptr < &_buffer[Size]) // we can delete without overshoot
          _readptr = ptr;
        else if (ptr == &_buffer[Size]) // exactly wrap around
          _readptr = _buffer;
        else // we got overshoot
          _readptr = _buffer + ((&_buffer[Size]) - ptr);
        ct -= count;
      }
      _count = ct;
    }

    // returns false if the buffer was empty when attempting to read
    bool pop(Value_t &v)
    {
      bool ret = false;
      if (_count != 0)
      {
        _count -= 1;
        v = *_readptr++;
        if (_readptr == &_buffer[Size])
        {
          _readptr = _buffer;
        }
        ret = true;
      }
      return ret;
    }

    // returns the amount of Values popped
    uint16_t batchPop(Value_t *vs, uint16_t size)
    {
      if (_count < size)
      {
        size = _count;
        _count = 0;
      }
      else
      {
        _count -= size;
      }
      if (size)
      { // only do something if the buffer is not empty
        Value_t *pEnd = &_buffer[Size];
        uint16_t wc = pEnd - _readptr;
        if (wc >= size)
        {
          std::memcpy(vs, _readptr, size);
          _readptr += size;
          if (_readptr == pEnd)
            _readptr = _buffer;
        }
        else
        { // less than size till the back, minimum 1 spot
          uint16_t tmp = size - wc;
          std::memcpy(vs, _readptr, wc * sizeof(Value_t));
          std::memcpy(vs + wc, _buffer, tmp * sizeof(Value_t));
          _readptr = _buffer + tmp;
        }
      }
      return size;
    }

    // with overwrite set returns false if another value was overwritten
    // with overwrite not set returns false if the value was not written
    bool push(const Value_t &v, bool overwrite = true)
    {
      bool ret = false;
      if (overwrite)
      {
        if (!full())
        {
          ret = true;
        }
        _push(v);
      }
      else if (!full())
      {
        _push(v);
        ret = true;
      }
      return ret;
    }

    // with overwrite set returns false if another value was overwritten
    // with overwrite not set returns false if the value was not written
    bool batchPush(const Value_t *vs, uint16_t size, bool overwrite = true)
    {
      bool ret = false;
      if (overwrite)
      {
        _batchPush(vs, size);
        ret = _count <= Size;
        if (!ret)
        {
          _count = Size;
          _readptr = _writeptr;
        }
      }
      else if (remainingSpace() >= size)
      {
        _batchPush(vs, size);
        ret = true;
      }
      return ret;
    }

  private:
    void _push(const Value_t &v)
    {
      _count += 1;
      *_writeptr++ = v;
      if (_writeptr == &_buffer[Size])
      {
        _writeptr = _buffer;
      }
    }

    void _batchPush(const Value_t *vs, uint16_t size)
    {
      Value_t *pEnd = &_buffer[Size];
      uint16_t wc = pEnd - _writeptr;
      if (wc >= size)
      {
        std::memcpy(_writeptr, vs, size * sizeof(Value_t));
        _writeptr += size;
        if (_writeptr == pEnd)
          _writeptr = _buffer;
      }
      else
      { // less than size till the back, minimum 1 spot
        uint16_t tmp = size - wc;
        std::memcpy(_writeptr, vs, wc * sizeof(Value_t));
        std::memcpy(_buffer, vs + wc, tmp * sizeof(Value_t));
        _writeptr = _buffer + tmp;
      }
      _count += size;
    }

    Value_t *_readptr;
    Value_t *_writeptr;
    volatile uint32_t _count;
    Value_t _buffer[Size];
  };

  template <typename Value_t>
  class DynRingbuffer
  {
  public:
    DynRingbuffer(uint16_t size)
    {
      _size = size;
      _readptr = _writeptr = _buffer = new Value_t[_size];
      _end = _buffer + size;
      _count = 0;
    }

    ~DynRingbuffer()
    {
      delete[] _buffer;
    }

    uint16_t in_avail() const
    {
      return _count;
    }

    bool full() const
    {
      return _count == _size;
    }

    // returns false if the buffer was empty when attempting to read
    bool pop(Value_t &v)
    {
      bool ret = false;
      if (_count != 0)
      {
        --_count;
        v = *_readptr++;
        if (_readptr == _end)
        {
          _readptr = _buffer;
        }
        ret = true;
      }
      return ret;
    }

    // with overwrite set returns false if another value was overwritten
    // with overwrite not set returns false if the value was not written
    bool push(Value_t &v, bool overwrite = true)
    {
      bool ret = false;
      if (overwrite)
      {
        if (!full())
        {
          ret = true;
        }
        ++_count;
        *_writeptr++ = v;
        if (_writeptr == _end)
        {
          _writeptr = _buffer;
        }
      }
      else if (!full())
      {
        ++_count;
        *_writeptr++ = v;
        if (_writeptr == _end)
        {
          _writeptr = _buffer;
        }
        ret = true;
      }
      return ret;
    }

  private:
    Value_t *_readptr;
    Value_t *_writeptr;
    uint16_t _count;
    uint16_t _size;
    Value_t *_buffer;
    Value_t *_end;
  };
} // namespace mtl
