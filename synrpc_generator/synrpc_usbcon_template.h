#pragma once

#include "synhal.h"

namespace syn {
  const int SYNRPC_USBCON_MAX = 255;
class PacketHandler;
class Packet {
  friend class PacketHandler;
public:
  Packet(){
    _meta = 0;
  }

  uint16_t size() const {
    return _meta & 0xFF;
  }

  uint16_t type() const {
    return (_meta >> 8 & 0xFF);
  }

  uint16_t sha1() const {
    return _meta >> 16;
  }

  uint8_t* data() {
    return _data;
  }

  const uint8_t* data() const {
    return _data;
  }

  template<typename newType>
  newType* cast(){
    return (newType*)this;
  }

  template<typename newType>
  const newType* cast() const{
    return (newType*)this;
  }
private:
  void form(uint8_t size, uint8_t type, uint16_t sha1){
    _meta = (sha1 << 16) | (type << 8) | size; // little endian [size | type | sha-1 | payload | revsize]
    _data[size] = SYNRPC_USBCON_MAX - uint16_t(size);
  }

  uint16_t rawSize() const {
    return size() + 5; // add 5 because of size + type + sha-1 + payload + rev_size
  }

  uint8_t* rawData() {
    return (uint8_t*)&_meta;
  }

  const uint8_t* rawData() const {
    return (const uint8_t*)&_meta;
  }

  uint32_t _meta; // 32bit for alignment
  uint8_t _data[SYNRPC_USBCON_MAX + 1];
};

class PacketHandler : public syn::Thread {
public:
  PacketHandler();
  template<typename MsgType>
  static void sendMessage(MsgType& msg)
  {
    Packet* p = (Packet*)((uint8_t*)&msg);
    p->form(MsgType::_size, MsgType::_type, MsgType::_sha1);
    syn::UsbCdc::write(p->rawData(), p->rawSize());
  }
  private:
  void run();
#if (SYN_USBRPC_USELED == 1)
  class LedTimer : public syn::SoftTimer{
  public:
    LedTimer(syn::Led& led);
  private:
    void execute();
    syn::Led& _led;
  };
#endif
  Packet _packetbuff;
};

struct SynRPCError{
  static const uint8_t _size = 58;
  // fits in single usb transfer. When sending 63 bytes, the host knows
  // the operation was finished and will not ask the controller again for any data
  // if transmitting 64 or more bytes, it will take 2 transfers to make sure the
  // transaction completed. size = 58 -> 4 start + payload + 1 end = 63
  static const uint8_t _type = 0;
  static const uint16_t _sha1 = 0xffff;
  uint32_t _packetstart;
  uint8_t intype;
  static const uint8_t ERRORMSG_SIZE = 57;
  char errormsg[ERRORMSG_SIZE];
  void write_errormsg(const char* str){
    strncpy(errormsg, str, ERRORMSG_SIZE - 1);
    errormsg[ERRORMSG_SIZE - 1] = 0;
  }
  uint8_t _packetend;
};

//// CUSTOM PACKET DEFINITIONS ////