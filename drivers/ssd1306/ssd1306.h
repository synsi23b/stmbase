#pragma once

#include <synhal.h>
#include <synos.h>

namespace SSD1306 {

struct Fontinfo_t {
  uint8_t character_height; // in pages
  // table containing the offset into the bitmap for the character
  const uint16_t *char_offset;
  // pixel values directly writable in vertical mode
  // setup the characters start and end (offset_table[char + 1]) 
  const uint8_t *char_bitmap;
};

extern const Fontinfo_t font_nokia_8;
extern const Fontinfo_t font_pokemon_8;
extern const Fontinfo_t font_vermin_vibes_16;
extern const Fontinfo_t font_runescape_16;

static const uint16_t displaywidth = 128;
static const uint16_t displayheight = 64;
static const uint16_t pagecount = 8;

class Display {
public:
  static const uint8_t I2CADDRESS_A = 0x78;
  static const uint8_t I2CADDRESS_B = 0x7A;

  Display(uint8_t port, uint8_t address);

  void init(syn::I2c::Speed speed = syn::I2c::high);
  // write pixels to whatever area set by page and column commands.
  // pointer is auto incrementing and wrapping
  void writePixels(const uint8_t *data, uint8_t count);
  // set drawing window page (up / down)
  void set_page(uint8_t page_start, uint8_t page_end);
  // set drawing window column
  void set_column(uint8_t column_start, uint8_t column_end);
  // write a line of text, starting at page. Fonts are generally 8 or 16 bits hight
  // that translates to 1 or 2 pages in height. The remaining space after the strinng
  // will be cleared to blank.
  void write_line(uint8_t page, const char* text, const Fontinfo_t *pfont);
  // flipped = true -> top is at connectors
  // default = false -> bottom is at connectors
  void flip(bool mode);
  // shift the ram by x lines, maximum is 63, default after init is 1
  void shift(uint8_t lines);
private:
  // write command. make sure sharec command buffer is not busy before writing
  void _writeCom(const uint8_t *coms, uint8_t count);

  uint8_t _address;
  uint8_t _combuf[3];
  syn::I2c _i2c;
};

}
