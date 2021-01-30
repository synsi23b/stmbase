#pragma once

#ifdef STM8S103
#include "../../synos8/synos.h"
#endif

class RF24
{
public:
  // setup all config registers and assign this modules receiving address (4 byte)
  // in case of STM8s uses port A1 for csn and A2 for ce
  // returns true if the setup appears to be successfull
  static bool init(const char *this_address, syn::SpiNC::eBaudrate speed = syn::SpiNC::MHz8);

  // write a 4 byte address to the transmit pipe (zero)
  static void set_destination(const char *address);

  // this method writes count bytes of data to the tx buffer and tries to transmit
  // before this, it is detrimental to call stop listen, if listening
  // and set a destination by set_destination. this only has to be done once if the
  // receiver never changes
  // on stm8 this method is calling sleep(1) in between the checks if the packet was ACKed
  // therefore it's not possible to call it outside of a routine
  // will return at maximum of about 10 milliseconds with true = packet ACKed
  static bool write(const uint8_t *data, uint8_t count);

  // when in listening, write ACK payloads (pipe 1)
  static bool write_ack_payload(const uint8_t *data, uint8_t count);

  // start listening on our address to enable perpetual receiving
  static void listen();
  // stop to listen on this address, call before write
  static void stop_listen();

  // try to read the RX FIFO, returns number of bytes read, up to 32.
  // the buffer should support that ammount of data.
  // returns 0 and doesn't alter the buffer if there was no data available
  static uint8_t read(uint8_t *buffer);

  // same as the regular write, but calculate MIC as follows
  // the payload size is fixed to 24 bytes and gets extended by an 8 byte MIC
  // therefore it's important to use a 32byte input buffer. the MIC is calculated
  // by using AES-128-CBC, the first 16 byte act as a IV and should contain a NONCE
  // the last 8 byte of the buffer will be zeroed before calculating the MIC
  static bool write_mic(uint8_t *data_32_byte);

  // same as read, but the buffer has to be 32 byte.
  // it will contain the 24byte payload and the 8 byte mic, if the sender is using the same system
  // if there was a message and the MIC checked out, will return 32
  // if there was a message that was less than 32 bytes, it will also be returned, without MIC check
  // if there was no message, will return 0
  // if the mic was not correct, will return -1
  static int8_t read_mic(uint8_t *buffer_32_byte);
private:
  // returns the status
  static uint8_t _write_command(uint8_t command);
  static void _write_register(uint8_t address, uint8_t data);
  // count is the ammount of bytes to write excluding the register address!!
  static void _write_register(uint8_t address, const uint8_t *data, uint8_t count);
  static uint8_t _read_register(uint8_t address);

  static syn::Gpio _csel;
  static syn::Gpio _ce;
};