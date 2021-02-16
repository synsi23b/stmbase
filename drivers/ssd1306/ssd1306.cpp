#include "ssd1306.h"

SSD1306::Display::Display(uint8_t port, uint8_t address) 
#ifndef STM8S103
:
    _i2c(port, address)
#endif
{
  _address = address;
}

static const uint8_t nothing_pxl[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

void SSD1306::Display::init(syn::I2c::Speed speed) {
  static const uint8_t inicoms[] = {
    0xAE, // display off
    // 0xD5, // set display clock div
    // 0x80, // reset value is 0x80 already
    // 0xA8, // set multiplex
    // 0x3F, // pixel  in height, reset is 64 pixel = 0x3F
    0xD3, // display shift 1 line, not sure if all displays need it.
    0x01,
    //0xDA, // com pin cfg, reset is already correct
    //0x12, // use 0x02 for 32 pixel height display
    // 0x40, // start page 0 reset to zero, possible range 0x40 .. 0x7F
    // 0xA0, // mirror RAM around Y Axis -> 0xA1, reset no mirror: SEG0 = COL0
    0x20, // set addressing mode
    0x01,  // vertical addressing
    0x81, // contrast
    0xFF, // 8bit contrast intensity value, default 7F
    0xD9, // set precharge period
    0xF1,
    0xDB, // set com deselect level VCC
    0x30, // 0.83*Vcc
    // 0x00, // ???
    //0xA4, // resume display
    //0xA6, // normal, none inverted mode
    0x8D, // activate charge pump
    0x14,
    0xAF  // turn on oled
  };
  _i2c.init(speed);
  _writeCom(inicoms, sizeof(inicoms));
}

void SSD1306::Display::writePixels(const uint8_t *data, uint8_t count)
{
  _i2c.busy_loop(syn::Routine::yield);
  _i2c.set_inject(0x40); // display gfx ram memory write command
  _i2c.write_async(_address, data, count);
}

void SSD1306::Display::_writeCom(const uint8_t *coms, uint8_t count)
{
  _i2c.set_inject(0x00);
  _i2c.write_async(_address, coms, count);
}

void SSD1306::Display::set_page(uint8_t page_start, uint8_t page_end)
{
  _i2c.busy_loop(syn::Routine::yield);
  _combuf[0] = 0x22;
  _combuf[1] = page_start;
  _combuf[2] = page_end;
  _writeCom(_combuf, 3);
}

void SSD1306::Display::set_column(uint8_t column_start, uint8_t column_end)
{
  _i2c.busy_loop(syn::Routine::yield);
  _combuf[0] = 0x21;
  _combuf[1] = column_start;
  _combuf[2] = column_end;
  _writeCom(_combuf, 3);
}

void SSD1306::Display::flip(bool mode)
{
  _i2c.busy_loop(syn::Routine::yield);
  if(mode)
  {
    // top is connector
    _combuf[0] = 0xC8; // flip
    _combuf[1] = 0xA1; // and mirror
  }
  else
  {
    // bottom is connector
    _combuf[0] = 0xC0; // un-flip
    _combuf[1] = 0xA0; // un-mirror
  }
  _writeCom(_combuf, 2);
}

void SSD1306::Display::shift(uint8_t lines)
{
  _i2c.busy_loop(syn::Routine::yield);
  _combuf[0] = 0xD3;
  _combuf[1] = lines;
  _writeCom(_combuf, 2);
}

void SSD1306::Display::write_line(uint8_t line, const char* text, const Fontinfo_t *pfont)
{
  if(line > 7)
  {
    line = 7;
  }
  uint8_t height = pfont->character_height;
  uint8_t y_end = line + height - 1;
  if(y_end > 7)
  {
    y_end = 7;
  }
  set_page(line, y_end);
  set_column(0, 127);
  uint8_t x_start = 0;
  for(char c = *text++; c != 0 && x_start < 127; c = *text++)
  {
    if(c < ' ' || c > '~')
    {
      c = '?';
    }
    c -= ' ';
    const uint16_t *offs = pfont->char_offset;
    uint16_t offset = offs[c];
    uint8_t len = offs[c + 1] - offset;
    uint8_t x_end = x_start + (len / height);
    if(x_end > 127)
    {
      x_end = 127;
    }
    //set_column(x_start, x_end);
    x_start = x_end;
    const uint8_t *pixel = pfont->char_bitmap + offset;
    writePixels(pixel, len);
  }
  if(x_start < 127)
  {
    // fill the rest with empty line
    set_column(x_start, 127);
    int16_t count = 127 - x_start;
    count *= height;
    while(count > 0)
    {
      writePixels(nothing_pxl, sizeof(nothing_pxl));
      count -= sizeof(nothing_pxl);
    }
  }
}
