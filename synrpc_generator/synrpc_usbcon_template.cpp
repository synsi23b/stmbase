#include <synrpc_usbcon.h>
#include <cstring> // strcopy for error msg
using namespace syn;

PacketHandler::PacketHandler() :
    Thread("PacketHandler", SYN_USBRPC_PRIORITY, SYN_USBRPC_STACKSIZE) {
}

const char* handlePacket(const Packet& p);

void PacketHandler::run() {
  UsbCdc::init();
#if (SYN_USBRPC_USELED == 1)
  Led led;
  LedTimer ledtimer(led);
#endif
  while (true) {
    // check if we have at least one byte (payloadsize)
    // else chill for up to 10 millis and then check again in case we missed it
    uint16_t incount = UsbCdc::in_avail();
    if (incount) {
      uint8_t plcount = 0;
      UsbCdc::peek(plcount, 0);
      uint16_t payloadsize = plcount;
      while (incount < payloadsize + 5) { // count + type + payload + rev_count
        UsbCdc::waitData(3);
        incount = UsbCdc::in_avail();
      }
      UsbCdc::peek(plcount, payloadsize + 4);
      if (payloadsize == (uint16_t(SYNRPC_USBCON_MAX) - uint16_t(plcount))) {
        // Packet is complete. read it and evoke handler and switch led
#if (SYN_USBRPC_USELED == 1)
        led.on();
        ledtimer.start();
#endif
        UsbCdc::read(_packetbuff.rawData(), payloadsize + 5);
        const char* err = handlePacket(_packetbuff);
        if(err){
          // an error was reported, copy it to packetbuffer and report back to host
          SynRPCError* pe = _packetbuff.cast<SynRPCError>();
          pe->intype = _packetbuff.type();
          pe->write_errormsg(err);
          _packetbuff.form(SynRPCError::_size, SynRPCError::_type, SynRPCError::_sha1);
          UsbCdc::write(_packetbuff.rawData(), _packetbuff.rawSize());
        }
      } else {
        UsbCdc::flush(1); // something is wrong, discard erroneous size byte
      }
    } else {
      UsbCdc::waitData(10);
    }
  }
}

#if (SYN_USBRPC_USELED == 1)
PacketHandler::LedTimer::LedTimer(syn::Led& led) :
  SoftTimer("LedTimer", 5, false), _led(led){
    _led.off();
  }

void PacketHandler::LedTimer::execute(){
  _led.off();
}
#endif

template<typename MsgType, typename MsgHandler>
const char* Converter(MsgHandler hndl, const Packet& p) {
  if(p.sha1() == MsgType::_sha1)
      return hndl(*(p.cast<MsgType>()));
  return "Sha-1 checksum mismatch";
}
typedef const char* (*converter_t)(const Packet& p);
