#pragma once

#ifdef STM8S103
#include "../synos8/synos.h"
#endif

#define PN532_PREAMBLE (0x00)
#define PN532_STARTCODE1 (0x00)
#define PN532_STARTCODE2 (0xFF)
#define PN532_POSTAMBLE (0x00)

#define PN532_HOSTTOPN532 (0xD4)
#define PN532_PN532TOHOST (0xD5)

#define PN532_ACK_WAIT_TIME (10) // ms, timeout of waiting for ACK

#define PN532_INVALID_ACK (-1)
#define PN532_TIMEOUT (-2)
#define PN532_INVALID_FRAME (-3)
#define PN532_NO_SPACE (-4)

class Pn532Rfid
{
public:
  static void init();

private:
  static bool _write_com(const uint8_t *data, uint8_t len);
  static bool _read_resp(uint8_t *buffer, uint8_t len);
  static void _prepare_ack();
  static bool _read_ack();

  static uint8_t _inbuffer[32];
};