#pragma once

#include <cassert>
#include <string>

#ifdef SYNHAL_MRAA
#include <mraa/i2c.hpp>
namespace syn {
  class I2cMaster {
  public:
    I2cMaster(uint16_t port, uint8_t address)
    : _i2cdev(port)
    {
      _address = address >> 1;
    }

    void write(uint8_t* data, uint16_t size) {
      _i2cdev.address(_address);
      _i2cdev.write(data, size);
    }
  private:
    mraa::I2c _i2cdev;
    uint8_t _address;
  };
}
#else
#include <synhal.h>
#endif

namespace SSD1306 {
struct Fontinfo_t {
  uint16_t character_height;
  char start_ascii;
  char end_ascii;
  const uint16_t (*char_info)[3];
  const uint8_t *char_bitmap;
};

extern const Fontinfo_t lucidaSansUnicode_13ptFontInfo;

static const uint16_t displaywidth = 128;
static const uint16_t displayheight = 64;
static const uint16_t pagecount = 8;

class Display {
public:
  static const uint8_t I2CADDRESS_A = 0x78;
  static const uint8_t I2CADDRESS_B = 0x7A;

  Display(uint16_t port, uint8_t address);

  void init();
  void writeColumns(uint16_t page, uint16_t columnstart, uint8_t* data, uint16_t columncount);

private:
  void _writeCom(uint8_t com);
  void _writeData(uint8_t *data, uint16_t columncount);

  syn::I2cMaster _i2c;
};

class GfxMemory {
  static const uint16_t CHUNKSIZE = 16; // for uint16 bitmask
  static const uint16_t LENCHANGED = displaywidth / CHUNKSIZE;

  class Page {
  public:
    void init(uint16_t pagenum);
    void sync(Display &dsp, bool forced);
    void clear();
    void writePixel(uint16_t x, uint16_t y, bool is_set);

  private:
    int16_t _findChangedStart();
    int16_t _findChangedEnd();

    uint8_t _columns[displaywidth + 1]; // holds copy of memory + write command in first byte
    uint8_t _pagenum; // own array index
    uint16_t _changed[LENCHANGED]; // holds bitmask of changed columns to write only what is necesarry
    uint16_t _ypixelstart; // substract from requested pixel to write, offset to columns
  };
public:
  GfxMemory();

  void init(Display *pdsp);
  void sync(bool forced = false);

  void clear();
  void setCursor(uint16_t x, uint16_t y) {
    _curx = x;
    _cury = y;
  }

  void writePicture(uint16_t width, uint16_t height, const char *data);
  void writeBlock(uint16_t width, uint16_t height, bool filled, bool adv_x = false);
  // advances x pointer one line beyond the end of the char, keeps y pointer
  void writeChar(uint16_t width, uint16_t height, const uint8_t *data);
  // writes string and advances pointer behind it
  void writeString(const std::string &text, const Fontinfo_t &info, bool fillLine = false);

  template<typename Fn>
  void writeAnything(uint16_t width, uint16_t height, Fn funct, bool adv_x, bool adv_y) {
    if (_curx <= displaywidth) {
      uint16_t xend = _curx + width;
      uint16_t yend = _cury + height;
      if (xend > displaywidth)
        xend = displaywidth;
      if (yend > displayheight)
        yend = displayheight;
      for (uint16_t ly = _cury; ly < yend; ++ly) {
        Page *ppage = _findPage(ly);
        for (uint16_t lx = _curx; lx < xend; ++lx)
          ppage->writePixel(lx, ly, funct());
      }
      if (adv_x)
        _curx = xend;
      if (adv_y)
        _cury = yend;
    }
  }
private:
  Page * _findPage(uint16_t y);

  Display* _pdsp;
  Page _pages[pagecount];
  uint16_t _curx, _cury;
};
}
