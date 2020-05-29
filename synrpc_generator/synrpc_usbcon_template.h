#pragma once

#include "synhal.h"

namespace syn
{{
  const int SYNRPC_USBCON_MAX = 122;
  const int SYNRPC_USBCON_GEN = {SYNRPC_USBCON_MAX_GEN};

  class UsbRpc::Packet
  {{
    friend class Handler;

  public:

    uint16_t size() const
    {{
      return _meta & 0xFF;
    }}

    uint16_t type() const
    {{
      return (_meta >> 8 & 0xFF);
    }}

    uint16_t sha1() const
    {{
      return _meta >> 16;
    }}

    uint16_t rawSize() const
    {{
      return size() + 5; // add 5 because of size + type + sha-1 + payload + rev_size
    }}

    template <typename NewType>
    NewType *cast()
    {{
      return (NewType *)this;
    }}

    template <typename NewType>
    const NewType *cast() const
    {{
      return (const NewType *)this;
    }}

  private:
    void form(uint8_t size, uint8_t type, uint16_t sha1)
    {{
      _meta = (sha1 << 16) | (type << 8) | size; // little endian [size | type | sha-1 | payload | revsize]
      _data[size] = SYNRPC_USBCON_MAX - uint16_t(size);
    }}

    const uint8_t *rawData() const
    {{
      return (const uint8_t *)&_meta;
    }}

    uint32_t _meta; // 32bit for alignment
    uint8_t _data[SYNRPC_USBCON_GEN + 1];
  }};

  class UsbRpc::Handler : public syn::Thread
  {{
  public:
    Handler();
    template <typename MsgType>
    static void sendMessage(MsgType &msg)
    {{
      Packet *p = (Packet *)((uint8_t *)&msg);
      p->form(MsgType::_size, MsgType::_type, MsgType::_sha1);
      syn::UsbRpc::write(p->rawData(), p->rawSize());
    }}

    // test a packet for plausibility
    // either returns expected rawSize, or 0 if it is not a packet
    static uint16_t plausible(const Packet &p);
    // the mailbox into which the usb device will push messages
    // it is public, but really isn't.
    static MailBox<UsbRpc::Packet, SYN_USBRPC_BUFFSIZE> _mailbox;

  private:
    void run();
#if (SYN_USBRPC_USELED == 1)
    class LedTimer : public syn::SoftTimer
    {{
    public:
      LedTimer(syn::Led &led);

    private:
      void execute();
      syn::Led &_led;
    }};
#endif
  }};

  struct SynRPCError
  {{
    static const uint8_t _size = 27;
    static const uint8_t _type = 0;
    static const uint16_t _sha1 = 0xffff;
    uint32_t _packetstart;
    uint8_t intype; // the type of the package that caused the error
    static const uint8_t ERRORMSG_SIZE = 26;
    char errormsg[ERRORMSG_SIZE];
    void write_errormsg(const char *str)
    {{
      strncpy(errormsg, str, ERRORMSG_SIZE - 1);
      errormsg[ERRORMSG_SIZE - 1] = 0;
    }}
    uint8_t _packetend;
  }};

  //// CUSTOM PACKET DEFINITIONS ////