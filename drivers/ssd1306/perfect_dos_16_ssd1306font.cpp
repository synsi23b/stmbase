#include "ssd1306.h"
// microfont perfect_dos with height 16
const uint8_t font_perfect_dos_16_bmp[] = {
  // ' ' @ offset 0
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '!' @ offset 18
  0x0, 0x0, 0x0, 0x0, 0xe0, 0x0, 0xf0, 0x37, 0xf0, 0x37, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '"' @ offset 36
  0x0, 0x0, 0x38, 0x0, 0x78, 0x0, 0x0, 0x0, 0x0, 0x0, 0x78, 0x0, 0x38, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '#' @ offset 54
  0x80, 0x8, 0xe0, 0x3f, 0xe0, 0x3f, 0x80, 0x8, 0xe0, 0x3f, 0xe0, 0x3f, 0x80, 0x8, 0x0, 0x0, 0x0, 0x0,
  // '$' @ offset 72
  0x70, 0xc, 0xf8, 0x18, 0x88, 0x10, 0x8e, 0x70, 0x8e, 0x70, 0x98, 0x1f, 0x30, 0xf, 0x0, 0x0, 0x0, 0x0,
  // '%' @ offset 90
  0xc0, 0x30, 0xc0, 0x18, 0x0, 0xc, 0x0, 0x6, 0x0, 0x3, 0x80, 0x31, 0xc0, 0x30, 0x0, 0x0, 0x0, 0x0,
  // '&' @ offset 108
  0x0, 0x1e, 0x60, 0x3f, 0xf0, 0x21, 0x90, 0x23, 0xf0, 0x1e, 0x60, 0x3f, 0x0, 0x21, 0x0, 0x0, 0x0, 0x0,
  // ''' @ offset 126
  0x0, 0x0, 0x40, 0x0, 0x78, 0x0, 0x38, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '(' @ offset 144
  0x0, 0x0, 0x0, 0x0, 0xc0, 0xf, 0xe0, 0x1f, 0x30, 0x30, 0x10, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // ')' @ offset 162
  0x0, 0x0, 0x0, 0x0, 0x10, 0x20, 0x30, 0x30, 0xe0, 0x1f, 0xc0, 0xf, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '*' @ offset 180
  0x0, 0x2, 0x80, 0xa, 0x80, 0xf, 0x0, 0x7, 0x0, 0x7, 0x80, 0xf, 0x80, 0xa, 0x0, 0x2, 0x0, 0x0,
  // '+' @ offset 198
  0x0, 0x0, 0x0, 0x2, 0x0, 0x2, 0x80, 0xf, 0x80, 0xf, 0x0, 0x2, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0,
  // ',' @ offset 216
  0x0, 0x0, 0x0, 0x0, 0x0, 0x20, 0x0, 0x3c, 0x0, 0x1c, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '-' @ offset 234
  0x0, 0x2, 0x0, 0x2, 0x0, 0x2, 0x0, 0x2, 0x0, 0x2, 0x0, 0x2, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0,
  // '.' @ offset 252
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x30, 0x0, 0x30, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '/' @ offset 270
  0x0, 0x30, 0x0, 0x18, 0x0, 0xc, 0x0, 0x6, 0x0, 0x3, 0x80, 0x1, 0xc0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '0' @ offset 288
  0xc0, 0xf, 0xe0, 0x1f, 0x30, 0x30, 0x10, 0x23, 0x30, 0x30, 0xe0, 0x1f, 0xc0, 0xf, 0x0, 0x0, 0x0, 0x0,
  // '1' @ offset 306
  0x0, 0x0, 0x40, 0x20, 0x60, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x20, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0,
  // '2' @ offset 324
  0x20, 0x38, 0x30, 0x3c, 0x10, 0x26, 0x10, 0x23, 0x90, 0x21, 0xf0, 0x30, 0x60, 0x30, 0x0, 0x0, 0x0, 0x0,
  // '3' @ offset 342
  0x20, 0x10, 0x30, 0x30, 0x10, 0x21, 0x10, 0x21, 0x10, 0x21, 0xf0, 0x3f, 0xe0, 0x1e, 0x0, 0x0, 0x0, 0x0,
  // '4' @ offset 360
  0x0, 0x3, 0x80, 0x3, 0xc0, 0x2, 0x60, 0x22, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x22, 0x0, 0x0, 0x0, 0x0,
  // '5' @ offset 378
  0xf0, 0x11, 0xf0, 0x31, 0x10, 0x21, 0x10, 0x21, 0x10, 0x21, 0x10, 0x3f, 0x10, 0x1e, 0x0, 0x0, 0x0, 0x0,
  // '6' @ offset 396
  0xc0, 0x1f, 0xe0, 0x3f, 0x30, 0x21, 0x10, 0x21, 0x10, 0x21, 0x0, 0x3f, 0x0, 0x1e, 0x0, 0x0, 0x0, 0x0,
  // '7' @ offset 414
  0x30, 0x0, 0x30, 0x0, 0x10, 0x3c, 0x10, 0x3e, 0x10, 0x3, 0xf0, 0x1, 0xf0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '8' @ offset 432
  0xe0, 0x1e, 0xf0, 0x3f, 0x10, 0x21, 0x10, 0x21, 0x10, 0x21, 0xf0, 0x3f, 0xe0, 0x1e, 0x0, 0x0, 0x0, 0x0,
  // '9' @ offset 450
  0xe0, 0x0, 0xf0, 0x21, 0x10, 0x21, 0x10, 0x21, 0x10, 0x31, 0xf0, 0x1f, 0xe0, 0xf, 0x0, 0x0, 0x0, 0x0,
  // ':' @ offset 468
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xc0, 0x18, 0xc0, 0x18, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // ';' @ offset 486
  0x0, 0x0, 0x0, 0x0, 0x0, 0x20, 0xc0, 0x38, 0xc0, 0x18, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '<' @ offset 504
  0x0, 0x0, 0x0, 0x2, 0x0, 0x7, 0x80, 0xd, 0xc0, 0x18, 0x60, 0x30, 0x20, 0x20, 0x0, 0x0, 0x0, 0x0,
  // '=' @ offset 522
  0x0, 0x0, 0x80, 0x4, 0x80, 0x4, 0x80, 0x4, 0x80, 0x4, 0x80, 0x4, 0x80, 0x4, 0x0, 0x0, 0x0, 0x0,
  // '>' @ offset 540
  0x0, 0x0, 0x20, 0x20, 0x60, 0x30, 0xc0, 0x18, 0x80, 0xd, 0x0, 0x7, 0x0, 0x2, 0x0, 0x0, 0x0, 0x0,
  // '?' @ offset 558
  0x60, 0x0, 0x70, 0x0, 0x10, 0x0, 0x10, 0x37, 0x90, 0x37, 0xf0, 0x0, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '@' @ offset 576
  0xc0, 0x1f, 0xe0, 0x3f, 0x20, 0x20, 0x20, 0x2f, 0x20, 0x2f, 0xe0, 0x2f, 0xc0, 0x7, 0x0, 0x0, 0x0, 0x0,
  // 'A' @ offset 594
  0x80, 0x3f, 0xc0, 0x3f, 0x60, 0x2, 0x30, 0x2, 0x60, 0x2, 0xc0, 0x3f, 0x80, 0x3f, 0x0, 0x0, 0x0, 0x0,
  // 'B' @ offset 612
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x10, 0x21, 0x10, 0x21, 0xf0, 0x3f, 0xe0, 0x1e, 0x0, 0x0, 0x0, 0x0,
  // 'C' @ offset 630
  0xc0, 0xf, 0xe0, 0x1f, 0x30, 0x30, 0x10, 0x20, 0x10, 0x20, 0x30, 0x30, 0x60, 0x18, 0x0, 0x0, 0x0, 0x0,
  // 'D' @ offset 648
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x10, 0x20, 0x30, 0x30, 0xe0, 0x1f, 0xc0, 0xf, 0x0, 0x0, 0x0, 0x0,
  // 'E' @ offset 666
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x10, 0x21, 0x90, 0x23, 0x30, 0x30, 0x70, 0x38, 0x0, 0x0, 0x0, 0x0,
  // 'F' @ offset 684
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x10, 0x21, 0x90, 0x3, 0x30, 0x0, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'G' @ offset 702
  0xc0, 0xf, 0xe0, 0x1f, 0x30, 0x30, 0x10, 0x22, 0x10, 0x22, 0x30, 0x1e, 0x60, 0x3e, 0x0, 0x0, 0x0, 0x0,
  // 'H' @ offset 720
  0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x1, 0x0, 0x1, 0x0, 0x1, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x0, 0x0, 0x0,
  // 'I' @ offset 738
  0x0, 0x0, 0x0, 0x0, 0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x10, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'J' @ offset 756
  0x0, 0x1c, 0x0, 0x3c, 0x0, 0x20, 0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x1f, 0x10, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'K' @ offset 774
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x3, 0x80, 0x7, 0xf0, 0x3c, 0x70, 0x38, 0x0, 0x0, 0x0, 0x0,
  // 'L' @ offset 792
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x10, 0x20, 0x0, 0x20, 0x0, 0x30, 0x0, 0x38, 0x0, 0x0, 0x0, 0x0,
  // 'M' @ offset 810
  0xf0, 0x3f, 0xf0, 0x3f, 0xe0, 0x0, 0xc0, 0x1, 0xe0, 0x0, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x0, 0x0, 0x0,
  // 'N' @ offset 828
  0xf0, 0x3f, 0xf0, 0x3f, 0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x0, 0x0, 0x0,
  // 'O' @ offset 846
  0xe0, 0x1f, 0xf0, 0x3f, 0x10, 0x20, 0x10, 0x20, 0x10, 0x20, 0xf0, 0x3f, 0xe0, 0x1f, 0x0, 0x0, 0x0, 0x0,
  // 'P' @ offset 864
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x10, 0x21, 0x10, 0x1, 0xf0, 0x1, 0xe0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'Q' @ offset 882
  0xf0, 0xf, 0xf8, 0x1f, 0x8, 0x10, 0x8, 0x1c, 0x8, 0x78, 0xf8, 0x7f, 0xf0, 0x4f, 0x0, 0x0, 0x0, 0x0,
  // 'R' @ offset 900
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x10, 0x1, 0x10, 0x3, 0xf0, 0x3f, 0xe0, 0x3c, 0x0, 0x0, 0x0, 0x0,
  // 'S' @ offset 918
  0x60, 0x18, 0xf0, 0x38, 0x90, 0x21, 0x10, 0x21, 0x10, 0x23, 0x70, 0x3e, 0x60, 0x1c, 0x0, 0x0, 0x0, 0x0,
  // 'T' @ offset 936
  0x0, 0x0, 0x70, 0x0, 0x30, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x30, 0x20, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'U' @ offset 954
  0xf0, 0x1f, 0xf0, 0x3f, 0x0, 0x20, 0x0, 0x20, 0x0, 0x20, 0xf0, 0x3f, 0xf0, 0x1f, 0x0, 0x0, 0x0, 0x0,
  // 'V' @ offset 972
  0xf0, 0x7, 0xf0, 0xf, 0x0, 0x18, 0x0, 0x30, 0x0, 0x18, 0xf0, 0xf, 0xf0, 0x7, 0x0, 0x0, 0x0, 0x0,
  // 'W' @ offset 990
  0xf0, 0x1f, 0xf0, 0x3f, 0x0, 0x38, 0x0, 0xf, 0x0, 0x38, 0xf0, 0x3f, 0xf0, 0x1f, 0x0, 0x0, 0x0, 0x0,
  // 'X' @ offset 1008
  0x30, 0x30, 0xf0, 0x3c, 0xc0, 0xf, 0x80, 0x7, 0xc0, 0xf, 0xf0, 0x3c, 0x30, 0x30, 0x0, 0x0, 0x0, 0x0,
  // 'Y' @ offset 1026
  0x0, 0x0, 0xf0, 0x0, 0xf0, 0x21, 0x0, 0x3f, 0x0, 0x3f, 0xf0, 0x21, 0xf0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'Z' @ offset 1044
  0x70, 0x38, 0x30, 0x3c, 0x10, 0x26, 0x10, 0x23, 0x90, 0x21, 0xf0, 0x30, 0x70, 0x38, 0x0, 0x0, 0x0, 0x0,
  // '[' @ offset 1062
  0x0, 0x0, 0x0, 0x0, 0xf0, 0x3f, 0xf0, 0x3f, 0x10, 0x20, 0x10, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '\' @ offset 1080
  0xe0, 0x0, 0xc0, 0x1, 0x80, 0x3, 0x0, 0x7, 0x0, 0xe, 0x0, 0x1c, 0x0, 0x38, 0x0, 0x0, 0x0, 0x0,
  // ']' @ offset 1098
  0x0, 0x0, 0x0, 0x0, 0x10, 0x20, 0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '^' @ offset 1116
  0x20, 0x0, 0x30, 0x0, 0x18, 0x0, 0xc, 0x0, 0x18, 0x0, 0x30, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '_' @ offset 1134
  0x0, 0x40, 0x0, 0x40, 0x0, 0x40, 0x0, 0x40, 0x0, 0x40, 0x0, 0x40, 0x0, 0x40, 0x0, 0x40, 0x0, 0x0,
  // '`' @ offset 1152
  0x0, 0x0, 0x0, 0x0, 0x8, 0x0, 0x18, 0x0, 0x30, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'a' @ offset 1170
  0x0, 0x1c, 0x80, 0x3e, 0x80, 0x22, 0x80, 0x22, 0x80, 0x1f, 0x0, 0x3f, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0,
  // 'b' @ offset 1188
  0x10, 0x0, 0xf0, 0x3f, 0xf0, 0x3f, 0x80, 0x20, 0x80, 0x21, 0x0, 0x3f, 0x0, 0x1e, 0x0, 0x0, 0x0, 0x0,
  // 'c' @ offset 1206
  0x0, 0x1f, 0x80, 0x3f, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x31, 0x0, 0x11, 0x0, 0x0, 0x0, 0x0,
  // 'd' @ offset 1224
  0x0, 0x1e, 0x0, 0x3f, 0x80, 0x21, 0x90, 0x20, 0xf0, 0x1f, 0xf0, 0x3f, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0,
  // 'e' @ offset 1242
  0x0, 0x1f, 0x80, 0x3f, 0x80, 0x22, 0x80, 0x22, 0x80, 0x22, 0x80, 0x33, 0x0, 0x13, 0x0, 0x0, 0x0, 0x0,
  // 'f' @ offset 1260
  0x0, 0x0, 0x0, 0x21, 0xe0, 0x3f, 0xf0, 0x3f, 0x10, 0x21, 0x30, 0x0, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'g' @ offset 1278
  0xc0, 0x27, 0xe0, 0x6f, 0x20, 0x48, 0x20, 0x48, 0xc0, 0x7f, 0xe0, 0x3f, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'h' @ offset 1296
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x1, 0x80, 0x0, 0x80, 0x3f, 0x0, 0x3f, 0x0, 0x0, 0x0, 0x0,
  // 'i' @ offset 1314
  0x0, 0x0, 0x0, 0x0, 0x80, 0x20, 0xb0, 0x3f, 0xb0, 0x3f, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'j' @ offset 1332
  0x0, 0x0, 0x0, 0x30, 0x0, 0x70, 0x0, 0x40, 0x20, 0x40, 0xec, 0x7f, 0xec, 0x3f, 0x0, 0x0, 0x0, 0x0,
  // 'k' @ offset 1350
  0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x6, 0x0, 0xf, 0x80, 0x39, 0x80, 0x30, 0x0, 0x0, 0x0, 0x0,
  // 'l' @ offset 1368
  0x0, 0x0, 0x0, 0x0, 0x10, 0x20, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'm' @ offset 1386
  0x80, 0x3f, 0x80, 0x3f, 0x80, 0x1, 0x0, 0x1f, 0x80, 0x1, 0x80, 0x3f, 0x0, 0x3f, 0x0, 0x0, 0x0, 0x0,
  // 'n' @ offset 1404
  0x80, 0x0, 0x80, 0x3f, 0x0, 0x3f, 0x80, 0x0, 0x80, 0x0, 0x80, 0x3f, 0x0, 0x3f, 0x0, 0x0, 0x0, 0x0,
  // 'o' @ offset 1422
  0x0, 0x1f, 0x80, 0x3f, 0x80, 0x20, 0x80, 0x20, 0x80, 0x20, 0x80, 0x3f, 0x0, 0x1f, 0x0, 0x0, 0x0, 0x0,
  // 'p' @ offset 1440
  0x20, 0x40, 0xe0, 0x7f, 0xc0, 0x7f, 0x20, 0x48, 0x20, 0x8, 0xe0, 0xf, 0xc0, 0x7, 0x0, 0x0, 0x0, 0x0,
  // 'q' @ offset 1458
  0xc0, 0x7, 0xe0, 0xf, 0x20, 0x8, 0x20, 0x48, 0xc0, 0x7f, 0xe0, 0x7f, 0x20, 0x40, 0x0, 0x0, 0x0, 0x0,
  // 'r' @ offset 1476
  0x80, 0x20, 0x80, 0x3f, 0x0, 0x3f, 0x80, 0x21, 0x80, 0x0, 0x80, 0x3, 0x0, 0x3, 0x0, 0x0, 0x0, 0x0,
  // 's' @ offset 1494
  0x0, 0x11, 0x80, 0x33, 0x80, 0x26, 0x80, 0x24, 0x80, 0x2c, 0x80, 0x39, 0x0, 0x11, 0x0, 0x0, 0x0, 0x0,
  // 't' @ offset 1512
  0x80, 0x0, 0x80, 0x0, 0xe0, 0x1f, 0xf0, 0x3f, 0x80, 0x20, 0x80, 0x30, 0x0, 0x10, 0x0, 0x0, 0x0, 0x0,
  // 'u' @ offset 1530
  0x80, 0x1f, 0x80, 0x3f, 0x0, 0x20, 0x0, 0x20, 0x80, 0x1f, 0x80, 0x3f, 0x0, 0x20, 0x0, 0x0, 0x0, 0x0,
  // 'v' @ offset 1548
  0x80, 0xf, 0x80, 0x1f, 0x0, 0x30, 0x0, 0x20, 0x0, 0x30, 0x80, 0x1f, 0x80, 0xf, 0x0, 0x0, 0x0, 0x0,
  // 'w' @ offset 1566
  0x80, 0x1f, 0x80, 0x3f, 0x0, 0x30, 0x0, 0x1e, 0x0, 0x30, 0x80, 0x3f, 0x80, 0x1f, 0x0, 0x0, 0x0, 0x0,
  // 'x' @ offset 1584
  0x80, 0x20, 0x80, 0x31, 0x0, 0x1f, 0x0, 0xe, 0x0, 0x1f, 0x80, 0x31, 0x80, 0x20, 0x0, 0x0, 0x0, 0x0,
  // 'y' @ offset 1602
  0xe0, 0x47, 0xe0, 0x4f, 0x0, 0x48, 0x0, 0x48, 0x0, 0x68, 0xe0, 0x3f, 0xe0, 0x1f, 0x0, 0x0, 0x0, 0x0,
  // 'z' @ offset 1620
  0x80, 0x31, 0x80, 0x39, 0x80, 0x2c, 0x80, 0x26, 0x80, 0x23, 0x80, 0x31, 0x80, 0x30, 0x0, 0x0, 0x0, 0x0,
  // '{' @ offset 1638
  0x0, 0x0, 0x0, 0x1, 0x0, 0x1, 0xe0, 0x1f, 0xf0, 0x3e, 0x10, 0x20, 0x10, 0x20, 0x0, 0x0, 0x0, 0x0,
  // '|' @ offset 1656
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xf0, 0x3f, 0xf0, 0x3f, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '}' @ offset 1674
  0x0, 0x0, 0x10, 0x20, 0x10, 0x20, 0xf0, 0x3e, 0xe0, 0x1f, 0x0, 0x1, 0x0, 0x1, 0x0, 0x0, 0x0, 0x0,
  // '~' @ offset 1692
  0x10, 0x0, 0x18, 0x0, 0x8, 0x0, 0x18, 0x0, 0x10, 0x0, 0x18, 0x0, 0x8, 0x0, 0x0, 0x0, 0x0, 0x0,
};

const uint16_t font_perfect_dos_16_off[] = {
  0, // ' '
  18, // '!'
  36, // '"'
  54, // '#'
  72, // '$'
  90, // '%'
  108, // '&'
  126, // '''
  144, // '('
  162, // ')'
  180, // '*'
  198, // '+'
  216, // ','
  234, // '-'
  252, // '.'
  270, // '/'
  288, // '0'
  306, // '1'
  324, // '2'
  342, // '3'
  360, // '4'
  378, // '5'
  396, // '6'
  414, // '7'
  432, // '8'
  450, // '9'
  468, // ':'
  486, // ';'
  504, // '<'
  522, // '='
  540, // '>'
  558, // '?'
  576, // '@'
  594, // 'A'
  612, // 'B'
  630, // 'C'
  648, // 'D'
  666, // 'E'
  684, // 'F'
  702, // 'G'
  720, // 'H'
  738, // 'I'
  756, // 'J'
  774, // 'K'
  792, // 'L'
  810, // 'M'
  828, // 'N'
  846, // 'O'
  864, // 'P'
  882, // 'Q'
  900, // 'R'
  918, // 'S'
  936, // 'T'
  954, // 'U'
  972, // 'V'
  990, // 'W'
  1008, // 'X'
  1026, // 'Y'
  1044, // 'Z'
  1062, // '['
  1080, // '\'
  1098, // ']'
  1116, // '^'
  1134, // '_'
  1152, // '`'
  1170, // 'a'
  1188, // 'b'
  1206, // 'c'
  1224, // 'd'
  1242, // 'e'
  1260, // 'f'
  1278, // 'g'
  1296, // 'h'
  1314, // 'i'
  1332, // 'j'
  1350, // 'k'
  1368, // 'l'
  1386, // 'm'
  1404, // 'n'
  1422, // 'o'
  1440, // 'p'
  1458, // 'q'
  1476, // 'r'
  1494, // 's'
  1512, // 't'
  1530, // 'u'
  1548, // 'v'
  1566, // 'w'
  1584, // 'x'
  1602, // 'y'
  1620, // 'z'
  1638, // '{'
  1656, // '|'
  1674, // '}'
  1692, // '~'
  1710, // ' '
};

const SSD1306::Fontinfo_t font_perfect_dos_16 = {
  // uint8_t character_height; // in pages
  2,
  // table containing the offset into the bitmap for the character
  font_perfect_dos_16_off,
  // pixel values directly writable in vertical mode
  // setup the characters start and end (offset_table[char + 1])
  font_perfect_dos_16_bmp
};

