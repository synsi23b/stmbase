#include "rf24.h"
#include "nRF24L01.h"

// setup all config registers and assign this modules receiving address (3 byte)
// in case of STM8s uses port A1 for csn and A2 for ce
bool RF24::init(const char *this_address, syn::SpiNC::eBaudrate speed)
{
  syn::SpiNC::init(speed, false, false);
  _csel.init('A', 3);
  _csel.mode(true, true);
  _csel.set();
  _ce.init('A', 2);
  _ce.mode(true, true);
  _ce.clear();

  // Must allow the radio time to settle else configuration bits will not necessarily stick.
  // This is actually only required following power up but some settling time also appears to
  // be required after resets too. For full coverage, we'll always assume the worst.
  // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
  // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
  // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  syn::System::delay(5);
  // setup this modules receiving address and address length
  _write_register(RX_ADDR_P1, (const uint8_t *)this_address, 3);
  _write_register(SETUP_AW, 1); // 3 bytes
  // set retranstmit retry delay to 0.75 millisecond and 10 retries
  _write_register(SETUP_RETR, 0x2A);
  // set datarate 1Mbps full power
  _write_register(RF_SETUP, 0x07);
  // set feature dynamic payloads
  _write_register(FEATURE, 0x04);
  _write_register(DYNPD, 0x3F); // dynamic payload all pipes

  // enable receiving on pipe 0 and 1
  _write_register(EN_RXADDR, 0x03);
  // set the payload size to maximum
  _write_register(RX_PW_P0, 32);
  _write_register(RX_PW_P1, 32);
  // set the device channel
  _write_register(RF_CH, 78);

  // clear interrupt bits in status register
  _write_register(NRF_STATUS, 0x70);
  // flush FIFOs
  _write_command(FLUSH_TX);
  _write_command(FLUSH_RX);

  // set crc handling and power up as primary TX
  // takes only a few micro amperes of power (running plls in stanby I)
  _write_register(NRF_CONFIG, 0x0E);

  // after powering up, wait for device to stabilize
  // github says it can take up to 5ms, datasheet says 1.5ms
  // but there are a lot of clones out there apparently, that rather sucks.
  syn::System::delay(3);

  // return true if powerup and 16bit crc
  return _read_register(NRF_CONFIG) == 0x0E;
}

void RF24::set_destination(const char *address)
{
  // write tx and pipe 0 address for enabling auto-ack
  _write_register(RX_ADDR_P0, (const uint8_t *)address, 3);
  _write_register(TX_ADDR, (const uint8_t *)address, 3);
}

bool RF24::write(uint8_t *data, uint8_t count)
{
  _write_register(W_TX_PAYLOAD, data, count);
  _ce.set();
  uint8_t status = 0;
  while (!(status & 0x30))
  {
    syn::Routine::sleep(1);
    status = _write_command(RF24_NOP);
  }
  _ce.clear();

  // clear all status bits before leaving this routine
  _write_register(NRF_STATUS, 0x30);

  if (status & 0x10)
  {
    //  didn't get a ACK within retry window
    _write_command(FLUSH_TX);
    return false;
  }
  return true;
}

bool RF24::set_ack_payload(uint8_t *data, uint8_t count)
{
  bool ret = false;
  return ret;
}

void RF24::listen()
{
  // enable listening only on pipe 1
  _write_register(EN_RXADDR, 0x02);
  // set PRIM_RX bit in config register
  _write_register(NRF_CONFIG, 0x0F);
  // clear irq status bits
  _write_register(NRF_STATUS, 0x70);
  // set CE pin to enable receiver
  _ce.set();
}

void RF24::stop_listen()
{
  _ce.clear();
  // delay because of some shenaningans were people send before the device is ready??
  // TODO: check if this is really neccesary!
  syn::udelay(130);
  // only flush tx if ack payloads are enabled
  //_write_command(FLUSH_TX);
  // clear the prim rx bit from the config
  _write_register(NRF_CONFIG, 0x0E);
  // re-enable listening only on pipe 0 to make acually hear ACKs
  _write_register(EN_RXADDR, 0x03);
}

uint8_t RF24::read(uint8_t *buffer)
{
  uint8_t count = 0;

  return count;
}

uint8_t RF24::_write_command(uint8_t command)
{
  _csel.clear();
  uint8_t status = syn::SpiNC::transceive1(command);
  _csel.set();
  return status;
}

void RF24::_write_register(uint8_t address, uint8_t data)
{
  // set write register bit in register address
  uint8_t bytes[2] = {W_REGISTER | address, data};
  _csel.clear();
  syn::SpiNC::write(bytes, 2);
  _csel.set();
}

void RF24::_write_register(uint8_t address, const uint8_t *data, uint8_t count)
{
  _csel.clear();
  syn::SpiNC::transceive1(W_REGISTER | address);
  syn::SpiNC::write(data, count);
  _csel.set();
}

uint8_t RF24::_read_register(uint8_t address)
{
  // set write register bit in register address
  uint8_t bytes[2] = {R_REGISTER | address, 0x00};
  _csel.clear();
  syn::SpiNC::transceive2_01(bytes);
  _csel.set();
  return bytes[1];
}