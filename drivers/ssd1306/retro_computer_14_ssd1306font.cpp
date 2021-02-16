#include "ssd1306.h"
// microfont retro_computer with height 14
const uint8_t font_retro_computer_14_bmp[] = {
  // ' ' @ offset 0
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '!' @ offset 10
  0x0, 0x0, 0xc0, 0xcf, 0xc0, 0xcf, 0x0, 0x0,
  // '"' @ offset 18
  0x0, 0x0, 0xf0, 0x0, 0x30, 0x0, 0x0, 0x0, 0xf0, 0x0, 0x30, 0x0, 0x0, 0x0,
  // '#' @ offset 32
  0x0, 0x0, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x33, 0x0, 0x33, 0x0, 0x0,
  // '$' @ offset 56
  0x0, 0x0, 0x80, 0x61, 0x80, 0x61, 0x60, 0x66, 0x60, 0x66, 0x78, 0xe6, 0x78, 0xe6, 0x60, 0x66, 0x60, 0x66, 0x60, 0x18, 0x60, 0x18, 0x0, 0x0,
  // '%' @ offset 80
  0x0, 0x0, 0x80, 0xc1, 0x40, 0xc2, 0x40, 0x32, 0x80, 0x31, 0x0, 0xc, 0x0, 0xc, 0x0, 0x63, 0x0, 0x93, 0xc0, 0x90, 0xc0, 0x60, 0x0, 0x0,
  // '&' @ offset 104
  0x0, 0x0, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0x0, 0xf0, 0x0, 0xf0, 0x0, 0x30, 0x0, 0x30, 0x0, 0x0,
  // ''' @ offset 132
  0x0, 0x0, 0xf0, 0x0, 0x30, 0x0, 0x0, 0x0,
  // '(' @ offset 140
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // ')' @ offset 152
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x3f, 0x0, 0x3f, 0x0, 0x0,
  // '*' @ offset 164
  0x0, 0x0, 0x80, 0x2, 0x0, 0x1, 0xc0, 0x7, 0x0, 0x1, 0x80, 0x2, 0x0, 0x0,
  // '+' @ offset 178
  0x0, 0x0, 0x0, 0xc, 0x0, 0xc, 0x0, 0x3f, 0x0, 0x3f, 0x0, 0xc, 0x0, 0xc, 0x0, 0x0,
  // ',' @ offset 194
  0x0, 0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0,
  // '-' @ offset 202
  0x0, 0x0, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0x0,
  // '.' @ offset 218
  0x0, 0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0,
  // '/' @ offset 226
  0x0, 0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x30, 0x0, 0x30, 0x0, 0xc, 0x0, 0xc, 0x0, 0x3, 0x0, 0x3, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x0,
  // '0' @ offset 250
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x3f, 0x0, 0x3f, 0x0, 0x0,
  // '1' @ offset 274
  0x0, 0x0, 0x0, 0x3, 0x0, 0x3, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // '2' @ offset 286
  0x0, 0x0, 0xc0, 0xf0, 0xc0, 0xf0, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0x0, 0xc3, 0x0, 0xc3, 0x0, 0x0,
  // '3' @ offset 310
  0x0, 0x0, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0x0, 0x33, 0x0, 0x33, 0x0, 0x0,
  // '4' @ offset 334
  0x0, 0x0, 0xc0, 0xf, 0xc0, 0xf, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // '5' @ offset 358
  0x0, 0x0, 0xc0, 0xcf, 0xc0, 0xcf, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0x30, 0xc0, 0x30, 0x0, 0x0,
  // '6' @ offset 382
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0x30, 0xc0, 0x30, 0x0, 0x0,
  // '7' @ offset 406
  0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0xf0, 0xc0, 0xf0, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0x3, 0xc0, 0x3, 0x0, 0x0,
  // '8' @ offset 430
  0x0, 0x0, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0x0, 0x33, 0x0, 0x33, 0x0, 0x0,
  // '9' @ offset 454
  0x0, 0x0, 0x0, 0xc3, 0x0, 0xc3, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0x0, 0x3f, 0x0, 0x3f, 0x0, 0x0,
  // ':' @ offset 478
  0x0, 0x0, 0x0, 0xc3, 0x0, 0xc3, 0x0, 0x0,
  // ';' @ offset 486
  0x0, 0x0, 0x80, 0x61, 0x80, 0xe1, 0x0, 0x0,
  // '<' @ offset 494
  0x0, 0x0, 0x0, 0xc, 0x0, 0xc, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // '=' @ offset 510
  0x0, 0x0, 0x0, 0x33, 0x0, 0x33, 0x0, 0x33, 0x0, 0x33, 0x0, 0x33, 0x0, 0x33, 0x0, 0x33, 0x0, 0x33, 0x0, 0x0,
  // '>' @ offset 530
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x33, 0x0, 0x33, 0x0, 0xc, 0x0, 0xc, 0x0, 0x0,
  // '?' @ offset 546
  0x0, 0x0, 0x0, 0x3, 0x0, 0x3, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xc, 0xc0, 0xc, 0x0, 0x3, 0x0, 0x3, 0x0, 0x0,
  // '@' @ offset 570
  0x0, 0x0, 0x0, 0xff, 0x0, 0xff, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0xfc, 0xc0, 0xfc, 0xc0, 0xcc, 0xc0, 0xcc, 0x0, 0xff, 0x0, 0xff, 0x0, 0x0,
  // 'A' @ offset 594
  0x0, 0x0, 0x0, 0xff, 0x0, 0xff, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0x0, 0xff, 0x0, 0xff, 0x0, 0x0,
  // 'B' @ offset 618
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0x0, 0x33, 0x0, 0x33, 0x0, 0x0,
  // 'C' @ offset 642
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x33, 0x0, 0x33, 0x0, 0x0,
  // 'D' @ offset 666
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x3f, 0x0, 0x3f, 0x0, 0x0,
  // 'E' @ offset 690
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // 'F' @ offset 714
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x0,
  // 'G' @ offset 738
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0x3c, 0xc0, 0x3c, 0x0, 0x0,
  // 'H' @ offset 762
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // 'I' @ offset 786
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // 'J' @ offset 810
  0x0, 0x0, 0x0, 0x3c, 0x0, 0x3c, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x3f, 0xc0, 0x3f, 0x0, 0x0,
  // 'K' @ offset 834
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // 'L' @ offset 858
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0,
  // 'M' @ offset 882
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x3, 0x0, 0x3, 0x0, 0xc, 0x0, 0xc, 0x0, 0x3, 0x0, 0x3, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // 'N' @ offset 906
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x3, 0x0, 0x3, 0x0, 0xc, 0x0, 0xc, 0x0, 0x30, 0x0, 0x30, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // 'O' @ offset 930
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x3f, 0x0, 0x3f, 0x0, 0x0,
  // 'P' @ offset 954
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0x0, 0x3, 0x0, 0x3, 0x0, 0x0,
  // 'Q' @ offset 978
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x30, 0xc0, 0x30, 0x0, 0xcf, 0x0, 0xcf, 0x0, 0x0,
  // 'R' @ offset 1002
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0x0, 0xf3, 0x0, 0xf3, 0x0, 0x0,
  // 'S' @ offset 1026
  0x0, 0x0, 0x0, 0xc3, 0x0, 0xc3, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0x30, 0xc0, 0x30, 0x0, 0x0,
  // 'T' @ offset 1050
  0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x0,
  // 'U' @ offset 1074
  0x0, 0x0, 0xc0, 0x3f, 0xc0, 0x3f, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0xc0, 0x3f, 0xc0, 0x3f, 0x0, 0x0,
  // 'V' @ offset 1098
  0x0, 0x0, 0xc0, 0xf, 0xc0, 0xf, 0x0, 0x30, 0x0, 0x30, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x30, 0x0, 0x30, 0xc0, 0xf, 0xc0, 0xf, 0x0, 0x0,
  // 'W' @ offset 1122
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x30, 0x0, 0x30, 0x0, 0xc, 0x0, 0xc, 0x0, 0x30, 0x0, 0x30, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // 'X' @ offset 1146
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x33, 0x0, 0x33, 0x0, 0xc, 0x0, 0xc, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // 'Y' @ offset 1170
  0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x3, 0x0, 0x3, 0x0, 0xfc, 0x0, 0xfc, 0x0, 0x3, 0x0, 0x3, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x0,
  // 'Z' @ offset 1194
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xf0, 0xc0, 0xf0, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xc3, 0xc0, 0xc3, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // '[' @ offset 1218
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // '\' @ offset 1230
  0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x3, 0x0, 0x3, 0x0, 0xc, 0x0, 0xc, 0x0, 0x30, 0x0, 0x30, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0,
  // ']' @ offset 1254
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // '^' @ offset 1266
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // '_' @ offset 1280
  0x0, 0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0,
  // '`' @ offset 1304
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
  // 'a' @ offset 1318
  0x0, 0x0, 0x0, 0xff, 0x0, 0xff, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0x0, 0xff, 0x0, 0xff, 0x0, 0x0,
  // 'b' @ offset 1342
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0x0, 0x33, 0x0, 0x33, 0x0, 0x0,
  // 'c' @ offset 1366
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x33, 0x0, 0x33, 0x0, 0x0,
  // 'd' @ offset 1390
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x3f, 0x0, 0x3f, 0x0, 0x0,
  // 'e' @ offset 1414
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // 'f' @ offset 1438
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x0,
  // 'g' @ offset 1462
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0x3c, 0xc0, 0x3c, 0x0, 0x0,
  // 'h' @ offset 1486
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // 'i' @ offset 1510
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // 'j' @ offset 1534
  0x0, 0x0, 0x0, 0x3c, 0x0, 0x3c, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x3f, 0xc0, 0x3f, 0x0, 0x0,
  // 'k' @ offset 1558
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0xc, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // 'l' @ offset 1582
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0,
  // 'm' @ offset 1606
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x3, 0x0, 0x3, 0x0, 0xc, 0x0, 0xc, 0x0, 0x3, 0x0, 0x3, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // 'n' @ offset 1630
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x3, 0x0, 0x3, 0x0, 0xc, 0x0, 0xc, 0x0, 0x30, 0x0, 0x30, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // 'o' @ offset 1654
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x3f, 0x0, 0x3f, 0x0, 0x0,
  // 'p' @ offset 1678
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0x0, 0x3, 0x0, 0x3, 0x0, 0x0,
  // 'q' @ offset 1702
  0x0, 0x0, 0x0, 0x3f, 0x0, 0x3f, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0x30, 0xc0, 0x30, 0x0, 0xcf, 0x0, 0xcf, 0x0, 0x0,
  // 'r' @ offset 1726
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0xc0, 0xc, 0x0, 0xf3, 0x0, 0xf3, 0x0, 0x0,
  // 's' @ offset 1750
  0x0, 0x0, 0x0, 0xc3, 0x0, 0xc3, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0x30, 0xc0, 0x30, 0x0, 0x0,
  // 't' @ offset 1774
  0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x0,
  // 'u' @ offset 1798
  0x0, 0x0, 0xc0, 0x3f, 0xc0, 0x3f, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0xc0, 0xc0, 0x3f, 0xc0, 0x3f, 0x0, 0x0,
  // 'v' @ offset 1822
  0x0, 0x0, 0xc0, 0xf, 0xc0, 0xf, 0x0, 0x30, 0x0, 0x30, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x30, 0x0, 0x30, 0xc0, 0xf, 0xc0, 0xf, 0x0, 0x0,
  // 'w' @ offset 1846
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x30, 0x0, 0x30, 0x0, 0xc, 0x0, 0xc, 0x0, 0x30, 0x0, 0x30, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // 'x' @ offset 1870
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x33, 0x0, 0x33, 0x0, 0xc, 0x0, 0xc, 0x0, 0x33, 0x0, 0x33, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // 'y' @ offset 1894
  0x0, 0x0, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x3, 0x0, 0x3, 0x0, 0xfc, 0x0, 0xfc, 0x0, 0x3, 0x0, 0x3, 0xc0, 0x0, 0xc0, 0x0, 0x0, 0x0,
  // 'z' @ offset 1918
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xf0, 0xc0, 0xf0, 0xc0, 0xcc, 0xc0, 0xcc, 0xc0, 0xc3, 0xc0, 0xc3, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // '{' @ offset 1942
  0x0, 0x0, 0x0, 0xc, 0x0, 0xc, 0xc0, 0xff, 0xc0, 0xff, 0xc0, 0xc0, 0xc0, 0xc0, 0x0, 0x0,
  // '|' @ offset 1958
  0x0, 0x0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0x0,
  // '}' @ offset 1966
  0x0, 0x0, 0xc0, 0xc0, 0xc0, 0xc0, 0xc0, 0xff, 0xc0, 0xff, 0x0, 0xc, 0x0, 0xc, 0x0, 0x0,
  // '~' @ offset 1982
  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
};

const uint16_t font_retro_computer_14_off[] = {
  0, // ' '
  10, // '!'
  18, // '"'
  32, // '#'
  56, // '$'
  80, // '%'
  104, // '&'
  132, // '''
  140, // '('
  152, // ')'
  164, // '*'
  178, // '+'
  194, // ','
  202, // '-'
  218, // '.'
  226, // '/'
  250, // '0'
  274, // '1'
  286, // '2'
  310, // '3'
  334, // '4'
  358, // '5'
  382, // '6'
  406, // '7'
  430, // '8'
  454, // '9'
  478, // ':'
  486, // ';'
  494, // '<'
  510, // '='
  530, // '>'
  546, // '?'
  570, // '@'
  594, // 'A'
  618, // 'B'
  642, // 'C'
  666, // 'D'
  690, // 'E'
  714, // 'F'
  738, // 'G'
  762, // 'H'
  786, // 'I'
  810, // 'J'
  834, // 'K'
  858, // 'L'
  882, // 'M'
  906, // 'N'
  930, // 'O'
  954, // 'P'
  978, // 'Q'
  1002, // 'R'
  1026, // 'S'
  1050, // 'T'
  1074, // 'U'
  1098, // 'V'
  1122, // 'W'
  1146, // 'X'
  1170, // 'Y'
  1194, // 'Z'
  1218, // '['
  1230, // '\'
  1254, // ']'
  1266, // '^'
  1280, // '_'
  1304, // '`'
  1318, // 'a'
  1342, // 'b'
  1366, // 'c'
  1390, // 'd'
  1414, // 'e'
  1438, // 'f'
  1462, // 'g'
  1486, // 'h'
  1510, // 'i'
  1534, // 'j'
  1558, // 'k'
  1582, // 'l'
  1606, // 'm'
  1630, // 'n'
  1654, // 'o'
  1678, // 'p'
  1702, // 'q'
  1726, // 'r'
  1750, // 's'
  1774, // 't'
  1798, // 'u'
  1822, // 'v'
  1846, // 'w'
  1870, // 'x'
  1894, // 'y'
  1918, // 'z'
  1942, // '{'
  1958, // '|'
  1966, // '}'
  1982, // '~'
  1996, // ' '
};

const SSD1306::Fontinfo_t font_retro_computer_14 = {
  // uint8_t character_height; // in pages
  2,
  // table containing the offset into the bitmap for the character
  font_retro_computer_14_off,
  // pixel values directly writable in vertical mode
  // setup the characters start and end (offset_table[char + 1])
  font_retro_computer_14_bmp
};

