#include "ssd1306.h"

SSD1306::Display::Display(uint16_t port, uint8_t address) :
    _i2c(port, address) {
}

void SSD1306::Display::init() {
  _i2c.init();
  _writeCom(0xAE); // display off
  _writeCom(0xD5); // set display clock div
  _writeCom(0x80);
  _writeCom(0xA8); // set multiplex
  _writeCom(0x3F); // pixel  in height
  _writeCom(0xD3); // offset
  _writeCom(0x00); // none
  _writeCom(0x40); // startline 0
  _writeCom(0x8D); // chargepump
  _writeCom(0x14);
  _writeCom(0x81); // contrast
  _writeCom(0xCF);
  _writeCom(0xD9); // set precharge
  _writeCom(0xF1);
  _writeCom(0xDB); // set com detect
  _writeCom(0x40);
  _writeCom(0x00);
  _writeCom(0xA4); // resume display
  _writeCom(0xA6); // normal, none inverted mode
  _writeCom(0xAF); // turn on oled
}

void SSD1306::Display::writeColumns(uint16_t page, uint16_t columnstart, uint8_t* data, uint16_t columncount) {
  assert(page < pagecount);
  assert(columnstart < displaywidth);
  assert(columncount <= displaywidth - columnstart);
  _writeCom(uint8_t(0xB0 | page)); // set correct page
  _writeCom(columnstart & 0x0F); // set correct column
  _writeCom(uint8_t(0x10 | columnstart >> 4));
  _writeData(data, columncount);
}

void SSD1306::Display::_writeCom(uint8_t com) {
  uint8_t buf[2] = { 0x0, com };
  _i2c.write(buf, 2);
}

// writes the data but puts the writecommand on the column just before the actual data
// this is reversed after its done. but necesarry thats why no const on data
void SSD1306::Display::_writeData(uint8_t *data, uint16_t columncount) {
  // can only send about 32 bytes at a time, so chop up the data to chunks of 30 bytes + address and write command
  // do it recursively because we are super cool
  uint16_t bytestowrite = columncount + 1;
  if (bytestowrite > 31) { // write address + command + 30 data bytes
    bytestowrite = 31;
  }
  --data;
  auto tmp = *data; // preserv
  *data = 0x40; // display gfx ram memory write command
  _i2c.write(data, bytestowrite);
  *data = tmp; // and reset
  columncount = columncount - bytestowrite + 1; // + 1 for the command
  if (columncount > 0) {
    _writeData(data + 30, columncount);
  }
}

void SSD1306::GfxMemory::Page::init(uint16_t pagenum) {
  _pagenum = pagenum;
  _ypixelstart = pagenum * 8;
  clear();
}

void SSD1306::GfxMemory::Page::sync(SSD1306::Display &dsp, bool forced) {
  if (forced) {
    dsp.writeColumns(_pagenum, 0, &_columns[1], displaywidth);
    for (auto & ch : _changed)
      ch = 0; // reset changed to 0
    return;
  }
  int16_t startindex = _findChangedStart();
  if (startindex != -1) {
    uint16_t endindex = _findChangedEnd();

    uint16_t count = endindex - startindex;
    dsp.writeColumns(_pagenum, startindex, &_columns[startindex + 1], count);

    for (auto & ch : _changed)
      ch = 0; // reset changed to 0
  }
}

void SSD1306::GfxMemory::Page::clear() {
  for (auto & col : _columns)
    col = 0xAA; // reset memory to clear
  for (auto & ch : _changed)
    ch = 0xFFFF; // reset changed to full
}

void SSD1306::GfxMemory::Page::writePixel(uint16_t x, uint16_t y, bool is_set) {
  y -= _ypixelstart;
  assert(y >= 0 && y < 8);
  assert(x >= 0 && x < displaywidth);
  auto* pcol = &_columns[x + 1]; // because of the reserved byte at the start of the ram
  auto curcol = *pcol;
  if (is_set)
    curcol |= uint8_t(1 << y);
  else
    curcol &= ~uint8_t(1 << y);
  if (curcol != *pcol) {
    *pcol = curcol;
    uint16_t chgIndex = x / CHUNKSIZE;
    _changed[chgIndex] |= uint16_t((1 << x % CHUNKSIZE));
  }
}

// return the index of the column we need to start writing to display
// return -1 if this page was not changed at all
int16_t SSD1306::GfxMemory::Page::_findChangedStart() {
  int16_t index = 0;
  for (auto ch : _changed) {
    if (ch != 0) {
      uint16_t mask = 1 << (CHUNKSIZE - 1);
      for (uint16_t i = 0; i < CHUNKSIZE; ++i, mask >>= 1)
        if (ch & mask)
          return index + i;
    }
    index += CHUNKSIZE;
  }
  return -1;
}

// return the one past the end index of the last changed column
int16_t SSD1306::GfxMemory::Page::_findChangedEnd() {
  int16_t index = LENCHANGED * CHUNKSIZE;
  for (uint16_t i = LENCHANGED - 1; i >= 0; --i) {
    auto ch = _changed[i];
    if (ch != 0) {
      uint16_t mask = 0x0001;
      for (uint16_t j = 0; j < CHUNKSIZE; ++j, mask <<= 1)
        if (ch & mask)
          return index - j;
    }
    index -= CHUNKSIZE;
  }
  return -1; // should not happen, we only ever call if there is a change
}

SSD1306::GfxMemory::GfxMemory() {
  _pdsp = 0;
  _curx = _cury = 0;
  for (uint16_t i = 0; i < pagecount; ++i)
    _pages[i].init(i);
}

void SSD1306::GfxMemory::init(SSD1306::Display *pdsp) {
  assert(pdsp);
  _pdsp = pdsp;
  _pdsp->init();
  sync();
}

void SSD1306::GfxMemory::sync(bool forced) {
  assert(_pdsp);
  for (auto & pg : _pages)
    pg.sync(*_pdsp, forced);
}

void SSD1306::GfxMemory::clear() {
  for (auto & pg : _pages)
    pg.clear();
}

void SSD1306::GfxMemory::writePicture(uint16_t width, uint16_t height, const char *data) {
  writeAnything(width, height, [&data]() {return *data++ != '.';}, false, false);
}

void SSD1306::GfxMemory::writeBlock(uint16_t width, uint16_t height, bool filled, bool adv_x) {
  writeAnything(width, height, [filled]() {return filled;}, adv_x, false);
}

// advances x pointer one line beyond the end of the car, keeps y pointer
void SSD1306::GfxMemory::writeChar(uint16_t width, uint16_t height, const uint8_t *data) {
  uint16_t mask = 0x80;
  uint16_t counter = 0;
  writeAnything(width, height, [width, &data, &mask, &counter]() {
    if (counter == width) {
      counter = 0;
      mask = 0x80;
      ++data;
    }
    else if (mask == 0) {
      mask = 0x80;
      ++data;
    }
    bool ret = *data & mask;
    mask >>= 1;
    ++counter;
    return ret;
  }, true, false);
  writeBlock(1, height, false, true);
}

// writes string and advances pointer behind it
void SSD1306::GfxMemory::writeString(const std::string &text, const Fontinfo_t &info, bool fillLine) {
  for (char cr : text) {
    if (cr < info.start_ascii || cr > info.end_ascii)
      cr = '?';
    uint16_t c = cr - info.start_ascii;
    uint16_t w = info.char_info[c][0];
    uint16_t h = info.char_info[c][1];
    uint16_t offs = info.char_info[c][2];
    writeChar(w, h, info.char_bitmap + offs);
  }
  if (fillLine) {
    writeBlock(displaywidth, info.char_info[0][1], false);
  }
}

SSD1306::GfxMemory::Page * SSD1306::GfxMemory::_findPage(uint16_t y) {
  assert(y < displayheight);
  // 8 pixel height per chunk
  return &_pages[y / 8];
}

