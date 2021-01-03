#pragma once

#ifdef STM8S103
#include "../synos8/synos.h"
#endif

class OneWire
{
public:
  static void init(char port, uint8_t pin);

  // Perform a 1-Wire reset cycle. Returns 1 if a device responds
  // with a presence pulse.  Returns 0 if there is no device or the
  // bus is shorted or otherwise held low for more than 250uS
  static bool reset(void);

  // Issue a 1-Wire rom select command, you do the reset first.
  static void select(const uint8_t rom[8]);

  // Issue a 1-Wire rom skip command, to address all on bus.
  static void skip(void);

  // Write a byte.
  // and wether or not to power up the bus after compelting the operation
  static void write(uint8_t v, bool power_up = false);

  static void write_bytes(const uint8_t *buf, uint8_t count);

  // Read a byte.
  static uint8_t read(void);

  static void read_bytes(uint8_t *buf, uint8_t count);

  // Write a bit. The bus is always left powered at the end, see
  // note in write() about that.
  static void write_bit(bool bit);

  // Read a bit.
  static bool read_bit(void);

  // Clear the search state so that if will start from the beginning again.
  static void reset_search();

  // Setup the search to find the device type 'family_code' on the next call
  // to search(*newAddr) if it is present.
  static void target_search(uint8_t family_code);

  // Look for the next device. Returns 1 if a new address has been
  // returned. A zero might mean that the bus is shorted, there are
  // no devices, or you have already retrieved all of them.  It
  // might be a good idea to check the CRC to make sure you didn't
  // get garbage.  The order is deterministic. You will always get
  // the same devices in the same order.
  static bool search(uint8_t *newAddr);
  //static bool search(uint8_t *newAddr, bool alarm_search = false);

  // Compute a Dallas Semiconductor 8 bit CRC, these are used in the
  // ROM and scratchpad registers.
  static uint8_t crc8_by_table(const uint8_t *addr, uint8_t len);
  static uint8_t crc8(const uint8_t *addr, uint8_t len);

private:
  static syn::Gpio _pin;

  // global search state
  static unsigned char ROM_NO[8];
  static uint8_t LastDiscrepancy;
  static uint8_t LastFamilyDiscrepancy;
  static bool LastDeviceFlag;
};

class DS18B20Manager;
class DS18B20
{
  friend DS18B20Manager;

public:
  // reads scratchpad value into the buffer
  // at least 9 bytes requiered
  bool read_scratchpad(uint8_t *scratchpad);

  // start a conversion on the device
  // usefull for multibus without extra powersupply
  // should wait at least 750 milliseconds before trying to get the result
  void start_conversion();

  // returns the raw value of the device.
  // fixed point integer SSSS-SXXX-XXXX.XXXX
  int16_t read_raw();

  // convert a raw value quickly to a int8 degree celsius
  static int8_t degree_c(int16_t raw)
  {
    return raw / 16;
  }

  const uint8_t *get_address() const
  {
    return _address;
  }

private:
  uint8_t _address[8];
};

class DS18B20Manager
{
public:
  // initializes the bus and counts sensors on the bus.
  // returns number of found devices
  uint8_t init(char port, uint8_t pin);

  uint8_t get_device_count() const
  {
    return _found_devices;
  }

  // select the device at the given index (starting at 0)
  // is temporarily stored in the manager for quick access
  // can be copied
  bool select_device(uint8_t index);

  // returns the currently selected device to perform actions
  // or copy it to save the address for later
  DS18B20 *get_selected_device()
  {
    return &_selected_device;
  }

  // start a conversion on all connected sensors at the same time
  // should wait at least 750 milliseconds before trying to get the result
  void start_convert_all();

private:
  bool _validate_address();

  uint8_t _found_devices;
  DS18B20 _selected_device;
};