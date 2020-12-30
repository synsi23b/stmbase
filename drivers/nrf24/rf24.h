#ifdef STM8S103
#include "../synos8/synos.h"
#endif

class RF24
{
public:
  // setup all config registers and assign this modules receiving address (4 byte)
  // in case of STM8s uses port A1 for csn and A2 for ce
  // returns true if the setup appears to be successfull
  bool init(const char *this_address, syn::SpiNC::eBaudrate speed = syn::SpiNC::MHz8);

  void set_destination(const char *address);

  bool write(const uint8_t *data, uint8_t count);

  bool write_ack_payload(const uint8_t *data, uint8_t count);

  void listen();
  void stop_listen();

  // try to read the RX FIFO, returns number of bytes read, up to 32.
  // the buffer should support that ammount of data.
  // returns 0 and doesn't alter the buffer if there was no data available
  uint8_t read(uint8_t *buffer);

private:
  // returns the status
  uint8_t _write_command(uint8_t command);
  void _write_register(uint8_t address, uint8_t data);
  // count is the ammount of bytes to write excluding the register address!!
  void _write_register(uint8_t address, const uint8_t *data, uint8_t count);
  uint8_t _read_register(uint8_t address);

  syn::Gpio _csel;
  syn::Gpio _ce;
};