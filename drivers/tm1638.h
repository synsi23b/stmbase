#pragma once

#ifdef STM8S103
#include "../synhal8/synhal.h"
#endif

class TM1638
{
public:
  // initialize the Gpios and the device
  // clears and turns on with middle intensity
  void init(int8_t dio_port, uint8_t dio_pin,
            int8_t clk_port, uint8_t clk_pin,
            int8_t stb_port, uint8_t stb_pin);

  // clears all stored data and synchronizes display
  void clear();

  // write buffered data to the device
  void sync();

  // display Err and error number and synchronize
  // doesn't overwrite regular display buffer
  void writeError(uint16_t num);

  // display LOAD and number and synchronize
  // doesn't overwrite regular display buffer
  void writeLoad(uint16_t num);

  // write leftDisplay 4 bytes
  // doesn't sync the output, just buffers
  void writeLeft(const uint8_t *data);

  // write right Display number
  // doesn't sync the output, just buffers
  void writeRight(uint16_t num);

  // write signed integer and remainder (0 to 99)
  // into the display buffer
  // if the remainder is to big, it will be no error,
  // but the display will be wrecked
  void writeSignedInt(int16_t num, uint8_t remainder = 0);

  // write unsigned integer and remainder (0 to 99)
  // into the display buffer
  // if the remainder is to big, it will be no error,
  // but the display will be wrecked
  void writeUnsignedInt(uint16_t num, uint8_t remainder = 0);

  // return the value of the currently pressed keys
  uint8_t readKeys();

  // value can range between 0x0 and 0xF
  // 0x8 has to be set to turn on the display at all
  void intensity(uint8_t value = 0x0E);

  // turn on or off the leds according to the mask
  void setLed(uint8_t mask)
  {
    _leds = mask;
  }

private:
  void _writeByte(uint8_t byte);
  uint8_t _readByte();
  void _delay();
  void _delay_fast_device();
  void _write_int(uint16_t num, uint8_t *data, bool remainder);
  void _sync(uint8_t *data);

  // const uint8_t _hex[16] = {
  //   0x3F, // (48)   0
  //   0x06, // (49)   1
  //   0x5B, // (50)   2
  //   0x4F, // (51)   3
  //   0x66, // (52)   4
  //   0x6D, // (53)   5
  //   0x7D, // (54)   6
  //   0x27, // (55)   7
  //   0x7F, // (56)   8
  //   0x6F, // (57)   9
  //   0x77, // (65)   A
  //   0x7C, // (98)   b
  //   0x39, // (67)   C
  //   0x5E, // (100)  d
  //   0x79, // (69)   E
  //   0x71, // (70)   F
  // };

  syn::Gpio _dio;
  syn::Gpio _clk;
  syn::Gpio _stb;
  uint8_t _display[8];
  uint8_t _leds;
};

inline void TM1638::_writeByte(uint8_t byte)
{
  // shift lsb first
  for (uint8_t mask = 0x01; mask != 0; mask <<= 1)
  {
    // rising edge device will read the line
    if (byte & mask)
      _dio.set();
    else
      _dio.clear();
    _delay_fast_device();
    // data is ready to be read
    _clk.set();
    _delay();
    _clk.clear();
  }
}

inline uint8_t TM1638::_readByte()
{
  uint8_t result = 0;
  // shift lsb first
  for (uint8_t mask = 0x01; mask != 0; mask <<= 1)
  {
    // falling edge device will prepare data to be read
    // we can read it on the rising edge
    // on first run, the data will be prepared from the commands last clock
    _delay_fast_device();
    _clk.set();
    if (_dio.read())
      result |= mask;
    // delay();
    _delay_fast_device();
    // clear clock, device prepares next data
    _clk.clear();
  }
  return result;
}

inline void TM1638::_delay()
{
#ifdef STM8S103
  nop();
  nop();
  nop();
  nop();
#endif
#if (defined(STM32F103xB) || defined(STM32F401xC))
  syn::Utility::udelay(1);
#endif
}

void TM1638::_delay_fast_device()
{
#if (defined(STM32F103xB) || defined(STM32F401xC))
  syn::Utility::udelay(1);
#endif
}

void TM1638::_write_int(uint16_t num, uint8_t *data, bool remainder)
{
  static const uint8_t _nums[10] = {
      0x3F, // (48)   0
      0x06, // (49)   1
      0x5B, // (50)   2
      0x4F, // (51)   3
      0x66, // (52)   4
      0x6D, // (53)   5
      0x7D, // (54)   6
      0x27, // (55)   7
      0x7F, // (56)   8
      0x6F, // (57)   9
  };

  if (num == 0)
  {
    if (remainder)
    {
      *data-- = _nums[0];
      *data-- = _nums[0];
      *data |= 0x80; // set decimal dot
    }
    else
    {
      *data = _nums[0];
    }
  }
  else
  {
    while (num != 0)
    {
      uint8_t res = num % 10;
      *data-- = _nums[res];
      num /= 10;
    }
    if (remainder)
    {
      if (*data == 0)
        *data-- = _nums[0];
      *data |= 0x80; // set decimal dot
    }
  }
}

inline void TM1638::_sync(uint8_t *data)
{
  // reset Address
  _stb.clear();
  _writeByte(0xC0);
  _stb.set();
  _delay();
  // write data
  _stb.clear();
  uint8_t led_shift = _leds;
  // display ram write command
  _writeByte(0x40);
  for (uint8_t i = 0; i < 8; ++i)
  {
    _writeByte(data[i]);
    _writeByte(led_shift & 0x01);
    led_shift >>= 1;
  }
  _stb.set();
}

void TM1638::init(int8_t dio_port, uint8_t dio_pin,
                  int8_t clk_port, uint8_t clk_pin,
                  int8_t stb_port, uint8_t stb_pin)
{
  _dio.init(dio_port, dio_pin)
      .opendrain();
  _clk.init(clk_port, clk_pin)
    .pushpull();
  _stb.init(stb_port, stb_pin)
    .pushpull()
    .set();
  this->clear();
  intensity();
}

void TM1638::clear()
{
  for (uint8_t i = 0; i < 8; ++i)
    _display[i] = 0;
  _leds = 0;
  sync();
}

inline void TM1638::sync()
{
  _sync(_display);
}

void TM1638::writeError(uint16_t num)
{
  uint8_t data[8] = { 0x79, 0x50, 0x50, 0, 0, 0, 0, 0 };
  _write_int(num, data + 7, false);
  _sync(data);
}

void TM1638::writeLoad(uint16_t num)
{
  uint8_t data[8] = {0x38, 0x5C, 0x77, 0x5E, 0, 0, 0, 0};
  _write_int(num, data + 7, false);
  _sync(data);
}

void TM1638::writeLeft(const uint8_t *data)
{
  for (uint8_t i = 0; i < 4; ++i)
    _display[i] = data[i];
}

void TM1638::writeRight(uint16_t num)
{
  for (uint8_t i = 4; i < 8; ++i)
    _display[i] = 0;
  _write_int(num, _display + 7, false);
}

void TM1638::writeSignedInt(int16_t num, uint8_t remainder)
{
  for (uint8_t i = 0; i < 7; ++i)
    _display[i] = 0;
  if (num < 0)
  {
    num = -num;
    _display[0] = 0x40;
  }
  _write_int(num, _display + 5, false);
  _write_int(remainder, _display + 7, true);
}

void TM1638::writeUnsignedInt(uint16_t num, uint8_t remainder)
{
  for (uint8_t i = 0; i < 7; ++i)
    _display[i] = 0;
  _write_int(num, _display + 5, false);
  _write_int(remainder, _display + 7, true);
}

inline uint8_t TM1638::readKeys()
{
  _stb.clear();
  uint8_t result = 0;
  _writeByte(0x42);
  _dio.set(); // open drain config, read possible now
  for (uint8_t i = 0; i < 4; ++i)
  {
    result |= (_readByte() << i);
  }
  _stb.set();
  return result;
}

void TM1638::intensity(uint8_t value)
{
  _stb.clear();
  _writeByte(0x80 | value);
  _stb.set();
}

// how to tell IAR to put array into RO memory
//#pragma location = 0x1fc8
//__root const uint16_t array[] = { 0x11, 0x22, 0x33, 0x44, 0x55 };
// definition for the displayable ASCII chars
// extern const uint8_t FONT_DEFAULT[] = {
//   0x00, // (32)  <space>
//   0x86, // (33)	!
//   0x22, // (34)	"
//   0x7E, // (35)	#
//   0x6D, // (36)	$
//   0x00, // (37)	%
//   0x00, // (38)	&
//   0x02, // (39)	'
//   0x30, // (40)	(
//   0x06, // (41)	)
//   0x63, // (42)	*
//   0x00, // (43)	+
//   0x04, // (44)	,
//   0x40, // (45)	-
//   0x80, // (46)	.
//   0x52, // (47)	/
//   0x3F, // (48)	0
//   0x06, // (49)	1
//   0x5B, // (50)	2
//   0x4F, // (51)	3
//   0x66, // (52)	4
//   0x6D, // (53)	5
//   0x7D, // (54)	6
//   0x27, // (55)	7
//   0x7F, // (56)	8
//   0x6F, // (57)	9
//   0x00, // (58)	:
//   0x00, // (59)	;
//   0x00, // (60)	<
//   0x48, // (61)	=
//   0x00, // (62)	>
//   0x53, // (63)	?
//   0x5F, // (64)	@
//   0x77, // (65)	A
//   0x7F, // (66)	B
//   0x39, // (67)	C
//   0x3F, // (68)	D
//   0x79, // (69)	E
//   0x71, // (70)	F
//   0x3D, // (71)	G
//   0x76, // (72)	H
//   0x06, // (73)	I
//   0x1F, // (74)	J
//   0x69, // (75)	K
//   0x38, // (76)	L
//   0x15, // (77)	M
//   0x37, // (78)	N
//   0x3F, // (79)	O
//   0x73, // (80)	P
//   0x67, // (81)	Q
//   0x31, // (82)	R
//   0x6D, // (83)	S
//   0x78, // (84)	T
//   0x3E, // (85)	U
//   0x2A, // (86)	V
//   0x1D, // (87)	W
//   0x76, // (88)	X
//   0x6E, // (89)	Y
//   0x5B, // (90)	Z
//   0x39, // (91)	[
//   0x64, // (92)	\ (this can't be the last char on a line, even in comment or it'll concat)
//   0x0F, // (93)	]
//   0x00, // (94)	^
//   0x08, // (95)	_
//   0x20, // (96)	`
//   0x5F, // (97)	a
//   0x7C, // (98)	b
//   0x58, // (99)	c
//   0x5E, // (100)	d
//   0x7B, // (101)	e
//   0x31, // (102)	f
//   0x6F, // (103)	g
//   0x74, // (104)	h
//   0x04, // (105)	i
//   0x0E, // (106)	j
//   0x75, // (107)	k
//   0x30, // (108)	l
//   0x55, // (109)	m
//   0x54, // (110)	n
//   0x5C, // (111)	o
//   0x73, // (112)	p
//   0x67, // (113)	q
//   0x50, // (114)	r
//   0x6D, // (115)	s
//   0x78, // (116)	t
//   0x1C, // (117)	u
//   0x2A, // (118)	v
//   0x1D, // (119)	w
//   0x76, // (120)	x
//   0x6E, // (121)	y
//   0x47, // (122)	z
//   0x46, // (123)	{
//   0x06, // (124)	|
//   0x70, // (125)	}
//   0x01, // (126)	~
// };
