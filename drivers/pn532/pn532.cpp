#include "pn532.h"

using namespace syn;

uint8_t Pn532Rfid::_inbuffer[32];

void Pn532Rfid::init()
{
  Uart::init(Uart::bd115200);
}

bool Pn532Rfid::_write_com(const uint8_t *data, uint8_t len)
{
  _prepare_ack();
  uint8_t preamble[] = {
      PN532_PREAMBLE,
      PN532_STARTCODE1,
      PN532_STARTCODE2,
      len,
      ~len + 1,
      PN532_HOSTTOPN532};
  Uart::write_async(preamble, sizeof(preamble));
  Uart::write_async(data, len);
  uint8_t chksum = PN532_HOSTTOPN532;
  while (len != 0)
  {
    chksum += *data++;
    --len;
  }
  Uart::putc(~chksum + 1);
  Uart::putc(PN532_POSTAMBLE);
  return _read_ack();
}

bool Pn532Rfid::_read_resp(uint8_t *buffer, uint8_t len)
{
}

void Pn532Rfid::_prepare_ack()
{
  while (Uart::rx_busy())
    ;
  for (uint8_t i = 0; i < 6; ++i)
  {
    _inbuffer[i] = 0;
  }
  Uart::read_async(_inbuffer, 6);
}

bool Pn532Rfid::_read_ack()
{
  const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};

  uint8_t timeout = PN532_TIMEOUT;
  while (--timeout != 0)
  {
    if (!Uart::rx_busy())
    {
      return Utility::memcmp(PN532_ACK, _inbuffer, sizeof(PN532_ACK));
    }
    sleep(1);
  }
  return false;
}
