#include "ssd1306.h"
// microfont minecraft with height 8
const uint8_t font_minecraft_8_bmp[] = {
  // ' ' @ offset 0
  0x0, 0x0,
  // '!' @ offset 2
  0x2f, 0x0,
  // '"' @ offset 4
  0x1, 0x0, 0x1, 0x0,
  // '#' @ offset 8
  0xa, 0x3f, 0xa, 0x3f, 0xa, 0x0,
  // '$' @ offset 14
  0x12, 0x15, 0x35, 0x15, 0x9, 0x0,
  // '%' @ offset 20
  0x21, 0x18, 0x4, 0x3, 0x30, 0x0,
  // '&' @ offset 26
  0x18, 0x25, 0x2e, 0x19, 0x24, 0x0,
  // ''' @ offset 32
  0x1, 0x0,
  // '(' @ offset 34
  0xe, 0x11, 0x20, 0x20, 0x0,
  // ')' @ offset 39
  0x20, 0x20, 0x11, 0xe, 0x0,
  // '*' @ offset 44
  0x2, 0x1, 0x1, 0x2, 0x0,
  // '+' @ offset 49
  0x8, 0x8, 0x3e, 0x8, 0x8, 0x0,
  // ',' @ offset 55
  0x70, 0x0,
  // '-' @ offset 57
  0x8, 0x8, 0x8, 0x8, 0x8, 0x0,
  // '.' @ offset 63
  0x30, 0x0,
  // '/' @ offset 65
  0x20, 0x18, 0x4, 0x3, 0x0, 0x0,
  // '0' @ offset 71
  0x1f, 0x28, 0x24, 0x22, 0x1f, 0x0,
  // '1' @ offset 77
  0x20, 0x21, 0x3f, 0x20, 0x20, 0x0,
  // '2' @ offset 83
  0x31, 0x28, 0x24, 0x24, 0x23, 0x0,
  // '3' @ offset 89
  0x11, 0x20, 0x24, 0x24, 0x1b, 0x0,
  // '4' @ offset 95
  0xc, 0xa, 0x9, 0x8, 0x3f, 0x0,
  // '5' @ offset 101
  0x13, 0x22, 0x22, 0x22, 0x1c, 0x0,
  // '6' @ offset 107
  0x1e, 0x25, 0x24, 0x24, 0x18, 0x0,
  // '7' @ offset 113
  0x1, 0x0, 0x38, 0x4, 0x3, 0x0,
  // '8' @ offset 119
  0x1b, 0x24, 0x24, 0x24, 0x1b, 0x0,
  // '9' @ offset 125
  0x3, 0x24, 0x24, 0x14, 0xf, 0x0,
  // ':' @ offset 131
  0x33, 0x0,
  // ';' @ offset 133
  0x73, 0x0,
  // '<' @ offset 135
  0x4, 0xa, 0x11, 0x20, 0x0,
  // '=' @ offset 140
  0x12, 0x12, 0x12, 0x12, 0x12, 0x0,
  // '>' @ offset 146
  0x20, 0x11, 0xa, 0x4, 0x0,
  // '?' @ offset 151
  0x1, 0x0, 0x28, 0x4, 0x3, 0x0,
  // '@' @ offset 157
  0x1f, 0x20, 0x2e, 0x2e, 0x28, 0x2f, 0x0,
  // 'A' @ offset 164
  0x3f, 0x2, 0x2, 0x2, 0x3f, 0x0,
  // 'B' @ offset 170
  0x3f, 0x22, 0x22, 0x22, 0x1d, 0x0,
  // 'C' @ offset 176
  0x1f, 0x20, 0x20, 0x20, 0x11, 0x0,
  // 'D' @ offset 182
  0x3f, 0x20, 0x20, 0x20, 0x1f, 0x0,
  // 'E' @ offset 188
  0x3f, 0x22, 0x22, 0x20, 0x20, 0x0,
  // 'F' @ offset 194
  0x3f, 0x2, 0x2, 0x0, 0x0, 0x0,
  // 'G' @ offset 200
  0x1f, 0x20, 0x22, 0x22, 0x1e, 0x0,
  // 'H' @ offset 206
  0x3f, 0x2, 0x2, 0x2, 0x3f, 0x0,
  // 'I' @ offset 212
  0x20, 0x3f, 0x20, 0x0,
  // 'J' @ offset 216
  0x10, 0x20, 0x20, 0x20, 0x1f, 0x0,
  // 'K' @ offset 222
  0x3f, 0x2, 0x2, 0x5, 0x38, 0x0,
  // 'L' @ offset 228
  0x3f, 0x20, 0x20, 0x20, 0x20, 0x0,
  // 'M' @ offset 234
  0x3f, 0x1, 0x2, 0x1, 0x3f, 0x0,
  // 'N' @ offset 240
  0x3f, 0x1, 0x2, 0x4, 0x3f, 0x0,
  // 'O' @ offset 246
  0x1f, 0x20, 0x20, 0x20, 0x1f, 0x0,
  // 'P' @ offset 252
  0x3f, 0x2, 0x2, 0x2, 0x1, 0x0,
  // 'Q' @ offset 258
  0x1f, 0x20, 0x20, 0x10, 0x2f, 0x0,
  // 'R' @ offset 264
  0x3f, 0x2, 0x2, 0x2, 0x3d, 0x0,
  // 'S' @ offset 270
  0x11, 0x22, 0x22, 0x22, 0x1c, 0x0,
  // 'T' @ offset 276
  0x0, 0x0, 0x3f, 0x0, 0x0, 0x0,
  // 'U' @ offset 282
  0x1f, 0x20, 0x20, 0x20, 0x1f, 0x0,
  // 'V' @ offset 288
  0x7, 0x18, 0x20, 0x18, 0x7, 0x0,
  // 'W' @ offset 294
  0x3f, 0x10, 0x8, 0x10, 0x3f, 0x0,
  // 'X' @ offset 300
  0x38, 0x5, 0x2, 0x5, 0x38, 0x0,
  // 'Y' @ offset 306
  0x0, 0x1, 0x3e, 0x1, 0x0, 0x0,
  // 'Z' @ offset 312
  0x30, 0x28, 0x24, 0x22, 0x21, 0x0,
  // '[' @ offset 318
  0x3f, 0x20, 0x20, 0x0,
  // '\' @ offset 322
  0x0, 0x3, 0x4, 0x18, 0x20, 0x0,
  // ']' @ offset 328
  0x20, 0x20, 0x3f, 0x0,
  // '^' @ offset 332
  0x2, 0x1, 0x0, 0x1, 0x2, 0x0,
  // '_' @ offset 338
  0x40, 0x40, 0x40, 0x40, 0x40, 0x0,
  // '`' @ offset 344
  0x0, 0x0, 0x0,
  // 'a' @ offset 347
  0x10, 0x2a, 0x2a, 0x2a, 0x3c, 0x0,
  // 'b' @ offset 353
  0x3f, 0x24, 0x22, 0x22, 0x1c, 0x0,
  // 'c' @ offset 359
  0x1c, 0x22, 0x22, 0x22, 0x14, 0x0,
  // 'd' @ offset 365
  0x1c, 0x22, 0x22, 0x24, 0x3f, 0x0,
  // 'e' @ offset 371
  0x1c, 0x2a, 0x2a, 0x2a, 0x2c, 0x0,
  // 'f' @ offset 377
  0x2, 0x3f, 0x2, 0x2, 0x0,
  // 'g' @ offset 382
  0x4c, 0x52, 0x52, 0x52, 0x3e, 0x0,
  // 'h' @ offset 388
  0x3f, 0x4, 0x2, 0x2, 0x3c, 0x0,
  // 'i' @ offset 394
  0x3e, 0x0,
  // 'j' @ offset 396
  0x30, 0x40, 0x40, 0x40, 0x3e, 0x0,
  // 'k' @ offset 402
  0x3f, 0x8, 0x14, 0x22, 0x0,
  // 'l' @ offset 407
  0x1f, 0x20, 0x0,
  // 'm' @ offset 410
  0x3e, 0x2, 0xc, 0x2, 0x3c, 0x0,
  // 'n' @ offset 416
  0x3e, 0x2, 0x2, 0x2, 0x3c, 0x0,
  // 'o' @ offset 422
  0x1c, 0x22, 0x22, 0x22, 0x1c, 0x0,
  // 'p' @ offset 428
  0x7e, 0x14, 0x12, 0x12, 0xc, 0x0,
  // 'q' @ offset 434
  0xc, 0x12, 0x12, 0x14, 0x7e, 0x0,
  // 'r' @ offset 440
  0x3e, 0x4, 0x2, 0x2, 0x4, 0x0,
  // 's' @ offset 446
  0x24, 0x2a, 0x2a, 0x2a, 0x12, 0x0,
  // 't' @ offset 452
  0x1, 0x1f, 0x21, 0x0,
  // 'u' @ offset 456
  0x1e, 0x20, 0x20, 0x20, 0x3e, 0x0,
  // 'v' @ offset 462
  0xe, 0x10, 0x20, 0x10, 0xe, 0x0,
  // 'w' @ offset 468
  0x1e, 0x20, 0x38, 0x20, 0x3e, 0x0,
  // 'x' @ offset 474
  0x22, 0x14, 0x8, 0x14, 0x22, 0x0,
  // 'y' @ offset 480
  0x4e, 0x50, 0x50, 0x50, 0x3e, 0x0,
  // 'z' @ offset 486
  0x22, 0x32, 0x2a, 0x26, 0x22, 0x0,
  // '{' @ offset 492
  0x4, 0x1b, 0x20, 0x20, 0x0,
  // '|' @ offset 497
  0x7f, 0x0,
  // '}' @ offset 499
  0x20, 0x20, 0x1b, 0x4, 0x0,
  // '~' @ offset 504
  0x1, 0x0, 0x0, 0x1, 0x1, 0x0, 0x0,
};

const uint16_t font_minecraft_8_off[] = {
  0, // ' '
  2, // '!'
  4, // '"'
  8, // '#'
  14, // '$'
  20, // '%'
  26, // '&'
  32, // '''
  34, // '('
  39, // ')'
  44, // '*'
  49, // '+'
  55, // ','
  57, // '-'
  63, // '.'
  65, // '/'
  71, // '0'
  77, // '1'
  83, // '2'
  89, // '3'
  95, // '4'
  101, // '5'
  107, // '6'
  113, // '7'
  119, // '8'
  125, // '9'
  131, // ':'
  133, // ';'
  135, // '<'
  140, // '='
  146, // '>'
  151, // '?'
  157, // '@'
  164, // 'A'
  170, // 'B'
  176, // 'C'
  182, // 'D'
  188, // 'E'
  194, // 'F'
  200, // 'G'
  206, // 'H'
  212, // 'I'
  216, // 'J'
  222, // 'K'
  228, // 'L'
  234, // 'M'
  240, // 'N'
  246, // 'O'
  252, // 'P'
  258, // 'Q'
  264, // 'R'
  270, // 'S'
  276, // 'T'
  282, // 'U'
  288, // 'V'
  294, // 'W'
  300, // 'X'
  306, // 'Y'
  312, // 'Z'
  318, // '['
  322, // '\'
  328, // ']'
  332, // '^'
  338, // '_'
  344, // '`'
  347, // 'a'
  353, // 'b'
  359, // 'c'
  365, // 'd'
  371, // 'e'
  377, // 'f'
  382, // 'g'
  388, // 'h'
  394, // 'i'
  396, // 'j'
  402, // 'k'
  407, // 'l'
  410, // 'm'
  416, // 'n'
  422, // 'o'
  428, // 'p'
  434, // 'q'
  440, // 'r'
  446, // 's'
  452, // 't'
  456, // 'u'
  462, // 'v'
  468, // 'w'
  474, // 'x'
  480, // 'y'
  486, // 'z'
  492, // '{'
  497, // '|'
  499, // '}'
  504, // '~'
  511, // ' '
};

const SSD1306::Fontinfo_t font_minecraft_8 = {
  // uint8_t character_height; // in pages
  1,
  // table containing the offset into the bitmap for the character
  font_minecraft_8_off,
  // pixel values directly writable in vertical mode
  // setup the characters start and end (offset_table[char + 1])
  font_minecraft_8_bmp
};
