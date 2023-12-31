#include "synrpc_usbcon.h"
#include <cstring> // strcopy for error msg
using namespace syn;

// the mailbox into which the usb device will push messages
// it is public, but really isn't.
MailBox<UsbRpc::Packet, SYN_USBRPC_BUFFSIZE> UsbRpc::Handler::_mailbox;

UsbRpc::Handler::Handler() : Thread("PacketHandler", SYN_USBRPC_PRIORITY, SYN_USBRPC_STACKSIZE)
{
}

const char *handlePacket(const UsbRpc::Packet &p);

void UsbRpc::Handler::run()
{
  UsbRpc::_start_usb();
#if (SYN_USBRPC_USELED == 1)
  LedTimer ledtimer;
#endif
  Packet *pmsg;
  while (true)
  {
    _mailbox.peek(&pmsg);
#if (SYN_USBRPC_USELED == 1)
    ledtimer.on();
#endif
    const char *err = handlePacket(*pmsg);
    if (err)
    {
      // an error was reported, copy it to packetbuffer and report back to host
      SynRPCError *pe = pmsg->cast<SynRPCError>();
      pe->intype = pmsg->type();
      pe->write_errormsg(err);
      sendMessage(*pe);
    }
    bool mailbox_full = _mailbox.is_full();
    _mailbox.purge();
    if (mailbox_full)
    {
      // we purged a message, it's not full anymore
      UsbRpc::_enable_rx();
    }
  }
}

#if (SYN_USBRPC_USELED == 1)
UsbRpc::Handler::LedTimer::LedTimer() : SoftTimer(5, false)
{
  _led.off();
}

void UsbRpc::Handler::LedTimer::on()
{
  _led.on();
  restart();
}

void UsbRpc::Handler::LedTimer::execute()
{
  _led.off();
}
#endif

template <typename MsgType, typename MsgHandler>
const char *Converter(MsgHandler hndl, const UsbRpc::Packet &p)
{
  if (p.sha1() == MsgType::_sha1)
    return hndl(*(p.cast<MsgType>()));
  return "Sha-1 checksum mismatch";
}
typedef const char *(*converter_t)(const UsbRpc::Packet &p);

template <typename MsgType>
uint16_t Checker(const UsbRpc::Packet &p)
{
  if (p.sha1() == MsgType::_sha1)
    return p.rawSize();
  return 0;
}
typedef uint16_t (*checker_t)(const UsbRpc::Packet &p);
