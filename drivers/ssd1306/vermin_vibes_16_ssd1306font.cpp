#include "ssd1306.h"
// microfont vermin_vibes with height 16
const uint8_t font_vermin_vibes_16_bmp[] = {
  // ' ' @ offset 0
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '!' @ offset 8
  0xe0, 0x1b, 0xe0, 0x1b, 0x0, 0x0,
  // '"' @ offset 14
  0xe0, 0x0, 0xe0, 0x0, 0x0, 0x0, 0xe0, 0x0, 0xe0, 0x0, 0x0, 0x0,
  // '#' @ offset 26
  0xc0, 0xc, 0xe0, 0x1f, 0xe0, 0x1f, 0xc0, 0xc, 0xc0, 0xc, 0xe0, 0x1f, 0xe0, 0x1f, 0xc0, 0xc, 0x0, 0x0,
  // '$' @ offset 44
  0xf0, 0x9, 0xf0, 0xd, 0xb0, 0xd, 0xf8, 0x1f, 0xf8, 0x1f, 0xb0, 0xd, 0xb0, 0xf, 0x90, 0xf, 0x0, 0x0,
  // '%' @ offset 62
  0x60, 0x18, 0x60, 0x1c, 0x0, 0xe, 0x0, 0x7, 0x80, 0x3, 0xc0, 0x1, 0xe0, 0x18, 0x60, 0x18, 0x20, 0x0, 0x0, 0x0,
  // '&' @ offset 82
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x58, 0x13, 0x48, 0x13, 0xc8, 0x1e, 0xc8, 0x1e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f,
  // ''' @ offset 102
  0xe0, 0x0, 0xe0, 0x0, 0x0, 0x0,
  // '(' @ offset 108
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x18, 0x60, 0x18, 0x0, 0x0,
  // ')' @ offset 118
  0x60, 0x18, 0x60, 0x18, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // '*' @ offset 128
  0xb8, 0xd, 0xf0, 0xf, 0xe0, 0x7, 0xf0, 0xf, 0xf0, 0xf, 0xe0, 0x7, 0xf0, 0xf, 0xb0, 0x1d, 0x10, 0x0, 0x0, 0x0,
  // '+' @ offset 148
  0x0, 0x3, 0x0, 0x3, 0xc0, 0xf, 0xc0, 0xf, 0x0, 0x3, 0x0, 0x3, 0x0, 0x0,
  // ',' @ offset 162
  0x0, 0x1c, 0x0, 0xc, 0x0, 0x0,
  // '-' @ offset 168
  0x0, 0x3, 0x0, 0x3, 0x0, 0x3, 0x0, 0x3, 0x0, 0x3, 0x0, 0x3, 0x0, 0x0,
  // '.' @ offset 182
  0x0, 0x18, 0x0, 0x18, 0x0, 0x0,
  // '/' @ offset 188
  0x0, 0x18, 0x0, 0x1c, 0x0, 0xe, 0x0, 0x7, 0x80, 0x3, 0xc0, 0x1, 0xe0, 0x0, 0x60, 0x0, 0x20, 0x0, 0x0, 0x0,
  // '0' @ offset 208
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x18, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x18, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // '1' @ offset 226
  0xc0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // '2' @ offset 232
  0x60, 0x1f, 0x60, 0x1f, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0xe0, 0x1b, 0xe0, 0x1b, 0x0, 0x0,
  // '3' @ offset 250
  0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // '4' @ offset 268
  0xe0, 0x3, 0xc0, 0x3, 0x0, 0x3, 0x0, 0x3, 0x0, 0x3, 0x0, 0x3, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // '5' @ offset 286
  0xe0, 0x1b, 0xe0, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1f, 0x60, 0x1f, 0x0, 0x0,
  // '6' @ offset 304
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1f, 0x60, 0x1f, 0x0, 0x0,
  // '7' @ offset 322
  0x60, 0x1c, 0x60, 0xe, 0x60, 0x7, 0xe0, 0x3, 0xe0, 0x1, 0xe0, 0x0, 0x60, 0x0, 0x20, 0x0, 0x0, 0x0,
  // '8' @ offset 340
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // '9' @ offset 358
  0xe0, 0x1b, 0xe0, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // ':' @ offset 376
  0xc0, 0xc, 0xc0, 0xc, 0x0, 0x0,
  // ';' @ offset 382
  0xc0, 0x1c, 0xc0, 0xc, 0x0, 0x0,
  // '<' @ offset 388
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x58, 0x13, 0x48, 0x13, 0xc8, 0x1e, 0xc8, 0x1e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f,
  // '=' @ offset 408
  0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0x0, 0x0,
  // '>' @ offset 422
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x58, 0x13, 0x48, 0x13, 0xc8, 0x1e, 0xc8, 0x1e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f,
  // '?' @ offset 442
  0x60, 0x1a, 0x60, 0x1b, 0x60, 0x3, 0x60, 0x3, 0x60, 0x3, 0x60, 0x3, 0xe0, 0x3, 0xe0, 0x3, 0x0, 0x0,
  // '@' @ offset 460
  0x80, 0x7, 0x40, 0x8, 0x20, 0x13, 0xa0, 0x14, 0xa0, 0x14, 0xa0, 0x17, 0x40, 0x14, 0x80, 0x7, 0x0, 0x0,
  // 'A' @ offset 478
  0x0, 0x10, 0x0, 0x18, 0x0, 0x1c, 0x0, 0xe, 0x0, 0x7, 0x80, 0x3, 0xc0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // 'B' @ offset 496
  0xf8, 0x7f, 0xf8, 0x3f, 0xd8, 0x1c, 0xf8, 0xe, 0xf8, 0x7, 0xf8, 0x3, 0xd8, 0x1, 0xc8, 0x0, 0x40, 0x0,
  // 'C' @ offset 514
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x0, 0x0,
  // 'D' @ offset 532
  0xf8, 0x7f, 0xf8, 0x3f, 0x18, 0x1c, 0x18, 0xe, 0x18, 0x7, 0x98, 0x3, 0xf8, 0x1, 0xf8, 0x0, 0x0, 0x0,
  // 'E' @ offset 550
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x0, 0x0,
  // 'F' @ offset 568
  0xf0, 0x3f, 0xf0, 0x1f, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0x0, 0x0,
  // 'G' @ offset 586
  0xf0, 0xf, 0xf0, 0xf, 0x30, 0xc, 0x30, 0xc, 0x30, 0xc, 0x30, 0xc, 0x30, 0x3f, 0xb0, 0x1f, 0x0, 0x0,
  // 'H' @ offset 604
  0xf0, 0x3f, 0xf0, 0x1f, 0x80, 0x1, 0x80, 0x1, 0x80, 0x1, 0x80, 0x1, 0xf8, 0xf, 0xfc, 0xf, 0x0, 0x0,
  // 'I' @ offset 622
  0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // 'J' @ offset 628
  0x0, 0x7f, 0x0, 0x3f, 0x0, 0x1c, 0x0, 0xe, 0x0, 0x7, 0x80, 0x3, 0xf8, 0x1, 0xf8, 0x0, 0x0, 0x0,
  // 'K' @ offset 646
  0xf0, 0xf, 0xf0, 0xf, 0x80, 0x1, 0xc0, 0x3, 0xe0, 0x7, 0x70, 0xe, 0x30, 0x1c, 0x10, 0x38, 0x0, 0x0,
  // 'L' @ offset 664
  0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x0,
  // 'M' @ offset 682
  0xe0, 0x1f, 0xc0, 0x1f, 0x80, 0x3, 0x0, 0x7, 0x0, 0x7, 0x80, 0x3, 0xc0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // 'N' @ offset 700
  0xf8, 0xf, 0xf0, 0xf, 0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x7, 0xf0, 0xf, 0xf0, 0x1f, 0x0, 0x0,
  // 'O' @ offset 718
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // 'P' @ offset 736
  0xb0, 0x3f, 0xb0, 0x1f, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xf0, 0x1, 0xf0, 0x1, 0x0, 0x0,
  // 'Q' @ offset 754
  0xf0, 0xf, 0xf0, 0xf, 0x30, 0xc, 0x30, 0x1f, 0x30, 0x3e, 0x30, 0xc, 0xf0, 0xf, 0xf0, 0xf, 0x0, 0x0,
  // 'R' @ offset 772
  0xb0, 0xf, 0xb0, 0xf, 0xb0, 0x1, 0xb0, 0x3, 0xb0, 0x7, 0xb0, 0xf, 0xf0, 0x1d, 0xf0, 0x39, 0x0, 0x0,
  // 'S' @ offset 790
  0xe0, 0x1b, 0xe0, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1f, 0x60, 0x1f, 0x0, 0x0,
  // 'T' @ offset 808
  0x60, 0x0, 0x60, 0x0, 0x60, 0x0, 0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x0, 0x60, 0x0, 0x60, 0x0, 0x0, 0x0,
  // 'U' @ offset 826
  0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // 'V' @ offset 844
  0x20, 0x0, 0x60, 0x0, 0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x7, 0xe0, 0xf, 0xe0, 0x1f, 0x0, 0x0,
  // 'W' @ offset 862
  0xe0, 0x1f, 0xe0, 0xf, 0x0, 0x7, 0x80, 0x3, 0x80, 0x3, 0x0, 0x7, 0xe0, 0xf, 0xe0, 0x1f, 0x0, 0x0,
  // 'X' @ offset 880
  0x38, 0xc, 0x70, 0xe, 0xe0, 0x7, 0xc0, 0x3, 0xc0, 0x3, 0xe0, 0x7, 0x70, 0xe, 0x30, 0x1c, 0x10, 0x0, 0x0, 0x0,
  // 'Y' @ offset 900
  0x70, 0x0, 0xe0, 0x0, 0xc0, 0x1, 0x80, 0x1f, 0x80, 0x1f, 0xc0, 0x1, 0xe0, 0x0, 0x60, 0x0, 0x20, 0x0,
  // 'Z' @ offset 918
  0x60, 0x18, 0x60, 0x1c, 0x60, 0x1e, 0x60, 0x1f, 0xe0, 0x1b, 0xe0, 0x19, 0xe0, 0x18, 0x60, 0x18, 0x20, 0x0, 0x0, 0x0,
  // '[' @ offset 938
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x18, 0x60, 0x18, 0x0, 0x0,
  // '\' @ offset 948
  0x60, 0x0, 0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x7, 0x0, 0xe, 0x0, 0x1c, 0x0, 0x18, 0x0, 0x10, 0x0, 0x0,
  // ']' @ offset 968
  0x60, 0x18, 0x60, 0x18, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // '^' @ offset 978
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x58, 0x13, 0x48, 0x13, 0xc8, 0x1e, 0xc8, 0x1e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f,
  // '_' @ offset 998
  0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x0,
  // '`' @ offset 1016
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x58, 0x13, 0x48, 0x13, 0xc8, 0x1e, 0xc8, 0x1e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f,
  // 'a' @ offset 1036
  0xe0, 0x1f, 0xc0, 0x1f, 0x80, 0x3, 0x0, 0x7, 0x0, 0xe, 0x0, 0x1c, 0x0, 0x18, 0x0, 0x10, 0x0, 0x0,
  // 'b' @ offset 1054
  0xf8, 0x7f, 0xf8, 0x3f, 0xd8, 0x1c, 0xf8, 0xe, 0xf8, 0x7, 0xf8, 0x3, 0xd8, 0x1, 0xc8, 0x0, 0x40, 0x0,
  // 'c' @ offset 1072
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x0, 0x0,
  // 'd' @ offset 1090
  0xf8, 0x7f, 0xf8, 0x3f, 0x18, 0x1c, 0x18, 0xe, 0x18, 0x7, 0x98, 0x3, 0xf8, 0x1, 0xf8, 0x0, 0x0, 0x0,
  // 'e' @ offset 1108
  0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x0, 0x0,
  // 'f' @ offset 1126
  0xf0, 0x3f, 0xf0, 0x1f, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0x0, 0x0,
  // 'g' @ offset 1144
  0xf0, 0xf, 0xf0, 0xf, 0x30, 0xc, 0x30, 0xc, 0x30, 0xc, 0x30, 0xc, 0x30, 0x3f, 0xb0, 0x1f, 0x0, 0x0,
  // 'h' @ offset 1162
  0xf0, 0x3f, 0xf0, 0x1f, 0x80, 0x1, 0x80, 0x1, 0x80, 0x1, 0x80, 0x1, 0xf8, 0xf, 0xfc, 0xf, 0x0, 0x0,
  // 'i' @ offset 1180
  0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // 'j' @ offset 1186
  0x0, 0x7f, 0x0, 0x3f, 0x0, 0x1c, 0x0, 0xe, 0x0, 0x7, 0x80, 0x3, 0xf8, 0x1, 0xf8, 0x0, 0x0, 0x0,
  // 'k' @ offset 1204
  0xf0, 0xf, 0xf0, 0xf, 0x80, 0x1, 0xc0, 0x3, 0xe0, 0x7, 0x70, 0xe, 0x30, 0x1c, 0x10, 0x38, 0x0, 0x0,
  // 'l' @ offset 1222
  0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x0,
  // 'm' @ offset 1240
  0xe0, 0x1f, 0xc0, 0x1f, 0x80, 0x3, 0x0, 0x7, 0x0, 0x7, 0x80, 0x3, 0xc0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // 'n' @ offset 1258
  0xf8, 0xf, 0xf0, 0xf, 0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x7, 0xf0, 0xf, 0xf0, 0x1f, 0x0, 0x0,
  // 'o' @ offset 1276
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0x60, 0x18, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // 'p' @ offset 1294
  0xb0, 0x3f, 0xb0, 0x1f, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xb0, 0x1, 0xf0, 0x1, 0xf0, 0x1, 0x0, 0x0,
  // 'q' @ offset 1312
  0xf0, 0xf, 0xf0, 0xf, 0x30, 0xc, 0x30, 0x1f, 0x30, 0x3e, 0x30, 0xc, 0xf0, 0xf, 0xf0, 0xf, 0x0, 0x0,
  // 'r' @ offset 1330
  0xb0, 0xf, 0xb0, 0xf, 0xb0, 0x1, 0xb0, 0x3, 0xb0, 0x7, 0xb0, 0xf, 0xf0, 0x1d, 0xf0, 0x39, 0x0, 0x0,
  // 's' @ offset 1348
  0xe0, 0x1b, 0xe0, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1b, 0x60, 0x1f, 0x60, 0x1f, 0x0, 0x0,
  // 't' @ offset 1366
  0x60, 0x0, 0x60, 0x0, 0x60, 0x0, 0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x0, 0x60, 0x0, 0x60, 0x0, 0x0, 0x0,
  // 'u' @ offset 1384
  0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0x0, 0x18, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x0,
  // 'v' @ offset 1402
  0xe0, 0x1f, 0xe0, 0xf, 0x0, 0x7, 0x80, 0x3, 0xc0, 0x1, 0xe0, 0x0, 0x60, 0x0, 0x20, 0x0, 0x0, 0x0,
  // 'w' @ offset 1420
  0xe0, 0x1f, 0xe0, 0xf, 0x0, 0x7, 0x80, 0x3, 0x80, 0x3, 0x0, 0x7, 0xe0, 0xf, 0xe0, 0x1f, 0x0, 0x0,
  // 'x' @ offset 1438
  0x38, 0xc, 0x70, 0xe, 0xe0, 0x7, 0xc0, 0x3, 0xc0, 0x3, 0xe0, 0x7, 0x70, 0xe, 0x30, 0x1c, 0x10, 0x0, 0x0, 0x0,
  // 'y' @ offset 1458
  0x70, 0x0, 0xe0, 0x0, 0xc0, 0x1, 0x80, 0x1f, 0x80, 0x1f, 0xc0, 0x1, 0xe0, 0x0, 0x60, 0x0, 0x20, 0x0,
  // 'z' @ offset 1476
  0x60, 0x18, 0x60, 0x1c, 0x60, 0x1e, 0x60, 0x1f, 0xe0, 0x1b, 0xe0, 0x19, 0xe0, 0x18, 0x60, 0x18, 0x20, 0x0, 0x0, 0x0,
  // '{' @ offset 1496
  0xe0, 0x1f, 0xe0, 0x1f, 0x60, 0x18, 0x60, 0x18, 0x0, 0x0, 0x0, 0x0,
  // '|' @ offset 1508
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x58, 0x13, 0x48, 0x13, 0xc8, 0x1e, 0xc8, 0x1e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f,
  // '}' @ offset 1528
  0x60, 0x18, 0x60, 0x18, 0xe0, 0x1f, 0xe0, 0x1f, 0x0, 0x3,
  // '~' @ offset 1538
  0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f, 0x58, 0x13, 0x48, 0x13, 0xc8, 0x1e, 0xc8, 0x1e, 0xf8, 0x1f, 0xf8, 0x1f, 0xf8, 0x1f,
};

const uint16_t font_vermin_vibes_16_off[] = {
  0, // ' '
  8, // '!'
  14, // '"'
  26, // '#'
  44, // '$'
  62, // '%'
  82, // '&'
  102, // '''
  108, // '('
  118, // ')'
  128, // '*'
  148, // '+'
  162, // ','
  168, // '-'
  182, // '.'
  188, // '/'
  208, // '0'
  226, // '1'
  232, // '2'
  250, // '3'
  268, // '4'
  286, // '5'
  304, // '6'
  322, // '7'
  340, // '8'
  358, // '9'
  376, // ':'
  382, // ';'
  388, // '<'
  408, // '='
  422, // '>'
  442, // '?'
  460, // '@'
  478, // 'A'
  496, // 'B'
  514, // 'C'
  532, // 'D'
  550, // 'E'
  568, // 'F'
  586, // 'G'
  604, // 'H'
  622, // 'I'
  628, // 'J'
  646, // 'K'
  664, // 'L'
  682, // 'M'
  700, // 'N'
  718, // 'O'
  736, // 'P'
  754, // 'Q'
  772, // 'R'
  790, // 'S'
  808, // 'T'
  826, // 'U'
  844, // 'V'
  862, // 'W'
  880, // 'X'
  900, // 'Y'
  918, // 'Z'
  938, // '['
  948, // '\'
  968, // ']'
  978, // '^'
  998, // '_'
  1016, // '`'
  1036, // 'a'
  1054, // 'b'
  1072, // 'c'
  1090, // 'd'
  1108, // 'e'
  1126, // 'f'
  1144, // 'g'
  1162, // 'h'
  1180, // 'i'
  1186, // 'j'
  1204, // 'k'
  1222, // 'l'
  1240, // 'm'
  1258, // 'n'
  1276, // 'o'
  1294, // 'p'
  1312, // 'q'
  1330, // 'r'
  1348, // 's'
  1366, // 't'
  1384, // 'u'
  1402, // 'v'
  1420, // 'w'
  1438, // 'x'
  1458, // 'y'
  1476, // 'z'
  1496, // '{'
  1508, // '|'
  1528, // '}'
  1538, // '~'
  1558, // ' '
};

const SSD1306::Fontinfo_t font_vermin_vibes_16 = {
  // uint8_t character_height; // in pages
  2,
  // table containing the offset into the bitmap for the character
  font_vermin_vibes_16_off,
  // pixel values directly writable in vertical mode
  // setup the characters start and end (offset_table[char + 1])
  font_vermin_vibes_16_bmp
};

