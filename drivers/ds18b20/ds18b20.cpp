#include "ds18b20.h"

syn::Gpio OneWire::_pin;

// global search state
unsigned char OneWire::ROM_NO[8];
uint8_t OneWire::LastDiscrepancy;
uint8_t OneWire::LastFamilyDiscrepancy;
bool OneWire::LastDeviceFlag;

void OneWire::init(char port, uint8_t pin)
{
  _pin.init(port, pin);
  _pin.floating();
  _pin.clear();
#if ONEWIRE_SEARCH
  reset_search();
#endif
}

// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a 0;
//
// Returns 1 if a device asserted a presence pulse, 0 otherwise.
//
bool OneWire::reset(void)
{
  // IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
  // volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
  // uint8_t r;
  // uint8_t retries = 125;

  // noInterrupts();
  // DIRECT_MODE_INPUT(reg, mask);
  // interrupts();
  // // wait until the wire is high... just in case
  uint8_t retries = 120;
  while (!_pin.read())
  {
    if (--retries == 0)
      return false;
    syn::Utility::udelay(2);
  }

  bool result;
  {
    syn::Atomic a;
    // pull output line line and wait for the minimum reset time
    _pin.set_direction_bit();
    syn::Utility::udelay(480);
    // let the pin float again to read the slaves
    _pin.clear_direction_bit();
    syn::Utility::udelay(70);
    result = !_pin.read();
    // noInterrupts();
    // DIRECT_WRITE_LOW(reg, mask);
    // DIRECT_MODE_OUTPUT(reg, mask); // drive output low
    // interrupts();
    // delayMicroseconds(480);
    // noInterrupts();
    // DIRECT_MODE_INPUT(reg, mask); // allow it to float
    // delayMicroseconds(70);
    // r = !DIRECT_READ(reg, mask);
    // interrupts();
  }
  //delayMicroseconds(410);
  syn::Utility::udelay(410);
  return result;
}

//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
void OneWire::write_bit(bool bit)
{
  uint8_t low_time = 60;
  if (bit)
  {
    low_time = 10;
  }
  {
    syn::Atomic a;
    _pin.set_direction_bit();
    syn::Utility::udelay(low_time);
    _pin.clear_direction_bit();
  }
  syn::Utility::udelay(65 - low_time);
}

//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
bool OneWire::read_bit(void)
{
  bool result;
  {
    syn::Atomic a;
    _pin.set_direction_bit();
    syn::Utility::udelay(2);
    _pin.clear_direction_bit();
    syn::Utility::udelay(8);
    // data is valid for 15 uSec after falling edge
    result = _pin.read();
  }
  syn::Utility::udelay(55);
  return result;
}

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
//
void OneWire::write(uint8_t v, bool power_up)
{
  for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1)
  {
    write_bit((bitMask & v));
  }
}

void OneWire::write_bytes(const uint8_t *buf, uint8_t count)
{
  while (count--)
  {
    write(*buf++);
  }
}

//
// Read a byte
//
uint8_t OneWire::read()
{
  uint8_t result = 0;
  for (uint8_t bitMask = 0x01; bitMask; bitMask <<= 1)
  {
    if (read_bit())
    {
      result |= bitMask;
    }
  }
  return result;
}

void OneWire::read_bytes(uint8_t *buf, uint8_t count)
{
  while (count--)
  {
    *buf++ = read();
  }
}

//
// Do a ROM select
//
void OneWire::select(const uint8_t rom[8])
{
  write(0x55); // Choose ROM
  write_bytes(rom, 8);
}

//
// Do a ROM skip
//
void OneWire::skip()
{
  write(0xCC); // Skip ROM
}

void OneWire::reset_search()
{
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  for (uint8_t i = 0; i < 8; ++i)
  {
    ROM_NO[i] = 0;
  }
}

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
//
void OneWire::target_search(uint8_t family_code)
{
  // set the search state to find SearchFamily type devices
  ROM_NO[0] = family_code;
  for (uint8_t i = 1; i < 8; ++i)
  {
    ROM_NO[i] = 0;
  }
  LastDiscrepancy = 64;
  LastFamilyDiscrepancy = 0;
  LastDeviceFlag = false;
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
bool OneWire::search(uint8_t *newAddr) //, bool alarm_search /* = false */)
{
  uint8_t id_bit_number;
  uint8_t last_zero, rom_byte_number;
  bool search_result;
  uint8_t id_bit, cmp_id_bit;

  unsigned char rom_byte_mask, search_direction;

  // initialize for search
  id_bit_number = 1;
  last_zero = 0;
  rom_byte_number = 0;
  rom_byte_mask = 1;
  search_result = false;

  // if the last call was not the last one
  if (!LastDeviceFlag)
  {
    // 1-Wire reset
    if (!reset())
    {
      // reset the search
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      return false;
    }

    // issue the search command
    // if (alarm_search)
    // {
    //   write(0xEC); // Search only thermometers that surpassed high or low temperature threasholds
    // }
    // else
    // {
    write(0xF0); // NORMAL SEARCH
    //}

    // loop to do the search
    do
    {
      // read a bit and its complement
      id_bit = read_bit();
      cmp_id_bit = read_bit();

      // check for no devices on 1-wire
      if ((id_bit == 1) && (cmp_id_bit == 1))
      {
        break;
      }
      else
      {
        // all devices coupled have 0 or 1
        if (id_bit != cmp_id_bit)
        {
          search_direction = id_bit; // bit write value for search
        }
        else
        {
          // if this discrepancy if before the Last Discrepancy
          // on a previous next then pick the same as last time
          if (id_bit_number < LastDiscrepancy)
          {
            search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
          }
          else
          {
            // if equal to last pick 1, if not then pick 0
            search_direction = (id_bit_number == LastDiscrepancy);
          }
          // if 0 was picked then record its position in LastZero
          if (search_direction == 0)
          {
            last_zero = id_bit_number;

            // check for Last discrepancy in family
            if (last_zero < 9)
              LastFamilyDiscrepancy = last_zero;
          }
        }

        // set or clear the bit in the ROM byte rom_byte_number
        // with mask rom_byte_mask
        if (search_direction == 1)
          ROM_NO[rom_byte_number] |= rom_byte_mask;
        else
          ROM_NO[rom_byte_number] &= ~rom_byte_mask;

        // serial number search direction write bit
        write_bit(search_direction);

        // increment the byte counter id_bit_number
        // and shift the mask rom_byte_mask
        id_bit_number++;
        rom_byte_mask <<= 1;

        // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
        if (rom_byte_mask == 0)
        {
          rom_byte_number++;
          rom_byte_mask = 1;
        }
      }
    } while (rom_byte_number < 8); // loop until through all ROM bytes 0-7

    // if the search was successful then
    if (!(id_bit_number < 65))
    {
      // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
      LastDiscrepancy = last_zero;

      // check for last device
      if (LastDiscrepancy == 0)
      {
        LastDeviceFlag = true;
      }
      search_result = true;
    }
  }

  // if no device found then reset counters so next 'search' will be like a first
  if (!search_result || !ROM_NO[0])
  {
    LastDiscrepancy = 0;
    LastDeviceFlag = false;
    LastFamilyDiscrepancy = 0;
    search_result = false;
  }
  else
  {
    for (int i = 0; i < 8; i++)
      newAddr[i] = ROM_NO[i];
  }
  return search_result;
}

static const uint8_t dscrc2x16_table[] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
    0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74};

uint8_t OneWire::crc8_by_table(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  while (len--)
  {
    crc = *addr++ ^ crc; // just re-using crc as intermediate
    crc = dscrc2x16_table[(crc & 0x0f)] ^ dscrc2x16_table[16 + ((crc >> 4) & 0x0f)];
  }

  return crc;
}

uint8_t OneWire::crc8(const uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;

  while (len--)
  {
    uint8_t inbyte = *addr++;
    for (uint8_t i = 8; i > 0; i--)
    {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix)
      {
        crc ^= 0x8C;
      }
      inbyte >>= 1;
    }
  }
  return crc;
}

bool DS18B20::read_scratchpad(uint8_t *scratchpad)
{
  if (!OneWire::reset())
  {
    return false;
  }
  OneWire::select(_address);
  OneWire::write(0xBE); // write read scratrch command
  OneWire::read_bytes(scratchpad, 9);
  // byte 0: temperature LSB
  // byte 1: temperature MSB
  // byte 2: high alarm temp
  // byte 3: low alarm temp
  // byte 4: DS18S20: store for crc
  //         DS18B20 & DS1822: configuration register
  // byte 5: internal use & crc
  // byte 6: DS18S20: COUNT_REMAIN
  //         DS18B20 & DS1822: store for crc
  // byte 7: DS18S20: COUNT_PER_C
  //         DS18B20 & DS1822: store for crc
  // byte 8: SCRATCHPAD_CRC
  return OneWire::crc8(scratchpad, 8) == scratchpad[8] && OneWire::reset();
}

void DS18B20::start_conversion()
{
  OneWire::reset();
  OneWire::select(_address);
  OneWire::write(0x44);
}

int16_t DS18B20::read_raw()
{
  uint8_t scratchpad[9];
  if (read_scratchpad(scratchpad))
  {
    int16_t result = scratchpad[1];
    result <<= 8;
    result |= scratchpad[0];
    return result;
  }
  return -30000;
}

uint8_t DS18B20Manager::init(char port, uint8_t pin)
{
  _found_devices = 0;
  OneWire::init(port, pin);
  select_device(255); // max index to count the devices
  return _found_devices;
}

void DS18B20Manager::start_convert_all()
{
  OneWire::reset();
  OneWire::skip();
  OneWire::write(0x44); // take temp and write to scratchpad
}

bool DS18B20Manager::_validate_address()
{
  uint8_t crc = OneWire::crc8(_selected_device._address, 7);
  return crc == _selected_device._address[7];
}

bool DS18B20Manager::select_device(uint8_t index)
{
  uint8_t depth = 0;
  OneWire::target_search(0x28); // search only DS18B20
  while (depth <= index && OneWire::search(_selected_device._address))
  {
    // confirm crc is correct
    if (_validate_address())
    {
      if (index == 255)
      {
        // if we go with a max index, try to count the found devices
        ++_found_devices;
      }
      if (depth == index)
      {
        return true;
      }
    }
    ++depth;
  }
  return false;
}