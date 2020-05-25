#include "synhal.h"
#if (SYN_ENABLE_USBCDC != 0)
using namespace syn;

namespace usb
{
  static Ringbuffer<uint8_t, SYN_USBCDC_BUFFSIZE, Atomic> rxringbuffer;
  static Signal sig_tx_done;
  static Signal sig_rx_buff;
  static Mutex tx_mutex;

  // start of the special usb memory region. 1.25kb
  static uint32_t *const usb_memory_start = (uint32_t *)0x40006000;

  typedef bool (*reception_handler_t)(const uint8_t *buffer, uint16_t size);

  bool rxCtrlEndpoint(const uint8_t *buffer, uint16_t size);
  bool rxCdcEndpoint(const uint8_t *buffer, uint16_t size);
  bool rxCmdEndpoint(const uint8_t *buffer, uint16_t size);

  typedef struct
  {
    uint16_t tx_address;
    uint16_t reserved_0;
    uint16_t tx_count;
    uint16_t reserved_1;
    uint16_t rx_address;
    uint16_t reserved_2;
    volatile uint16_t rx_count;
    uint16_t reserved_3;

    void setTX(uint32_t *buffer)
    {
      tx_address = (buffer - usb_memory_start) * 2;
    }

    void setRx(uint32_t *buffer, uint16_t size)
    {
      rx_address = (buffer - usb_memory_start) * 2;
      setRxCount(size);
    }

    void setRxCount(uint16_t size)
    {
      if (size & 0x1)
        size += 1;
      if (size < 64)
      {
        size >>= 1;
        rx_count = size << 10;
      }
      else
      {
        size /= 32;
        rx_count = USB_COUNT0_RX_BLSIZE | ((size - 1) << 10);
      }
    }

    uint32_t *getBufferTx()
    {
      return usb_memory_start + (tx_address / 2);
    }

    uint32_t *getBufferRx()
    {
      return usb_memory_start + (rx_address / 2);
    }

    uint16_t getRxCount()
    {
      return rx_count & 0x3FF;
    }
  } BufferTableEntry_t;

  typedef struct
  {
    volatile uint16_t rx_remaining;
    uint16_t reserved_1;
    volatile uint16_t tx_buff_adr_hi;
    uint16_t reserved_2;
    volatile uint16_t tx_buff_adr_lo;
    uint16_t reserved_3;
    volatile uint16_t tx_remaining;
    uint16_t reserved_4;

    void setTxAddress(const uint8_t *buf)
    {
      uint32_t b = (uint32_t)buf;
      tx_buff_adr_hi = uint16_t(b >> 16);
      tx_buff_adr_lo = uint16_t(b);
    }

    const uint8_t *getTxAddress()
    {
      uint32_t b = tx_buff_adr_hi;
      b <<= 16;
      b |= tx_buff_adr_lo;
      return (const uint8_t *)b;
    }
  } BufferState_t;

  static const uint32_t endpoint_count = 3;
  static const uint32_t max_packet = 64;
  static const uint32_t max_cdc_command = 8;
  static const reception_handler_t rx_handlers[endpoint_count] = {&rxCtrlEndpoint, &rxCdcEndpoint, &rxCmdEndpoint};
  static uint8_t rx_buffer[max_packet];

  typedef struct
  {
    // this is a description of the special memory area reserved
    // for USB communication. First the internal buffers are described
    // by BufferTableEntries, Hardwarwe will read them (maybe?)
    // After that, their individual states are stored
    // than come the actual buffers
    // and last, internally used variables using the remainin space and not
    // wasting application memory
    BufferTableEntry_t bufdesc[endpoint_count];
    BufferState_t bufstate[endpoint_count];
    // TODO ctrl and command packets should only be 8 byte, why use 32 * 4 bytes?
    // max packet is 64 bytes, but the memory is 16 bit wide
    // however access is 32 bit wide. so by usint 32bit int, we guarantee alignment
    // and by dividing by 2 we guarantee the correct size in bytes
    // but why is cdc command with 8 bytes not divided, this is a 16 byte buffer
    uint32_t buffer_ctrl_ep_out[max_packet / 2];
    uint32_t buffer_ctrl_ep_in[max_packet / 2];
    uint32_t buffer_cdc_out[max_packet / 2];
    uint32_t buffer_cdc_in[max_packet / 2]; 
    uint32_t buffer_cdc_com_in[max_cdc_command];
    volatile uint32_t address;     // store the device address when it gets set by host
    volatile uint32_t stage;       // store the current stage of ctrl endpoint
    volatile uint32_t status;      // store the device status (usb standard)
    volatile uint32_t config;      // store the current selected config
    volatile uint32_t data_rx_off; // store the state of the data rx (on /off) for flow control (host to fast)
    volatile uint32_t ctrl_opcode; // store opcode from setup request when we are receiving ctrl data during in stage
    volatile uint32_t ctrl_epval;  // store bits of ctrl endpoint so we can access them in the reception handler

    uint16_t readEndpoint(uint16_t endpoint)
    {
      uint16_t count = bufdesc[endpoint].getRxCount();
      uint16_t *writeptr = (uint16_t *)rx_buffer;
      uint32_t *readptr = bufdesc[endpoint].getBufferRx();
      uint32_t *end = readptr + ((count + 1) >> 1);
      while (readptr < end)
        *writeptr++ = *readptr++;
      return count;
    }

    void writeEndpoint(uint16_t endpoint, const uint8_t *buffer, uint16_t size)
    {
      uint16_t *readptr = (uint16_t *)buffer;
      uint32_t *writeptr = bufdesc[endpoint].getBufferTx();
      if (max_packet < size)
        size = max_packet;
      //else if (size == max_packet)
      //  size = 63;
      uint16_t *end = readptr + ((size + 1) >> 1);
      while (readptr < end)
        *writeptr++ = *readptr++;
      bufdesc[endpoint].tx_count = size;
    }
  } Memory_t;

  static Memory_t *const mem = (Memory_t *)0x40006000;

  namespace lowlevel
  {
    volatile uint16_t *getEpReg(uint16_t endpoint)
    {
      return (uint16_t *)((uint32_t *)USB_EP0R + endpoint);
    }

    uint16_t getDefault(uint16_t epval, uint16_t additional_mask = 0)
    {
      return epval & (USB_EP0R_EP_TYPE_Msk | USB_EP0R_EP_KIND_Msk | USB_EP0R_EA_Msk | additional_mask);
    }

    enum eEndpointType
    {
      eBulk = 0,
      eControl = 1,
      eIsochronous = 2,
      eInterrupt = 3
    };

    void initial(uint16_t endpoint, uint16_t address, eEndpointType type)
    {
      volatile uint16_t *ep = getEpReg(endpoint);
      *ep = (uint16_t(type) << USB_EP0R_EP_TYPE_Pos) | address;
    }

    void clear_ctr_rx(uint16_t endpoint)
    {
      volatile uint16_t *ep = getEpReg(endpoint);
      uint16_t val = getDefault(*ep);
      val |= USB_EP0R_CTR_TX; // set tx, we only want to clear rx
      *ep = val;
    }

    void clear_ctr_tx(uint16_t endpoint)
    {
      volatile uint16_t *ep = getEpReg(endpoint);
      uint16_t val = getDefault(*ep);
      val |= USB_EP0R_CTR_RX; // set rx, we only want to clear tx
      *ep = val;
    }

    enum eEndpointStatus
    {
      eDisabled = 0,
      eStall = 1,
      eNak = 2,
      eValid = 3
    };

    void set_rx_stat(uint16_t endpoint, eEndpointStatus status, bool clear_dtog = false)
    {
      uint16_t extramask;
      if (clear_dtog)
        extramask = USB_EPRX_STAT | USB_EP0R_DTOG_RX;
      else
        extramask = USB_EPRX_STAT;
      volatile uint16_t *ep = getEpReg(endpoint);
      uint16_t val = getDefault(*ep, extramask);
      if (status & 0x1)
      {
        val ^= USB_EP0R_STAT_RX_0;
      }
      if (status & 0x2)
      {
        val ^= USB_EP0R_STAT_RX_1;
      }
      *ep = val | USB_EP0R_CTR_RX | USB_EP0R_CTR_TX;
    }

    void set_tx_stat(uint16_t endpoint, eEndpointStatus status, bool clear_dtog = false)
    {
      uint16_t extramask;
      if (clear_dtog)
        extramask = USB_EPTX_STAT | USB_EP0R_DTOG_TX;
      else
        extramask = USB_EPTX_STAT;
      volatile uint16_t *ep = getEpReg(endpoint);
      uint16_t val = getDefault(*ep, extramask);
      if (status & 0x1)
      {
        val ^= USB_EP0R_STAT_TX_0;
      }
      if (status & 0x2)
      {
        val ^= USB_EP0R_STAT_TX_1;
      }
      *ep = val | USB_EP0R_CTR_RX | USB_EP0R_CTR_TX;
    }

    void ctrlError()
    {
      // TODO
      // stall the ctrl endpoint
      // only stall opposite direction
      // this is fucking weird man
      // it works right now so whatever
      set_rx_stat(0, usb::lowlevel::eStall);
      set_tx_stat(0, usb::lowlevel::eStall);
    }

    static const uint16_t ctrl_stage_rdy = 0;
    static const uint16_t ctrl_stage_data_in = 0x01;
    static const uint16_t ctrl_stage_data_out = 0x81;
    static const uint16_t ctrl_stage_state_in = 0x02;
    static const uint16_t ctrl_stage_state_out = 0x82;

    void ctrlSendStatus()
    {
      mem->stage = ctrl_stage_state_in;
      mem->bufdesc[0].tx_count = 0;
      set_tx_stat(0, usb::lowlevel::eValid);
    }

    void ctrlReceiveStatus()
    {
      mem->stage = ctrl_stage_state_out;
      volatile uint16_t *ep = getEpReg(0);
      *ep = USB_EP0R_CTR_RX | USB_EP0R_CTR_TX | USB_EP0R_EP_TYPE_0 | USB_EP0R_EP_KIND;
      set_rx_stat(0, usb::lowlevel::eValid);
    }

    void ctrlReceiveReady()
    {
      mem->stage = ctrl_stage_rdy;
      volatile uint16_t *ep = getEpReg(0);
      *ep = USB_EP0R_CTR_RX | USB_EP0R_CTR_TX | USB_EP0R_EP_TYPE_0;
      mem->bufstate[0].tx_remaining = 0;
      set_tx_stat(0, usb::lowlevel::eNak);
      set_rx_stat(0, usb::lowlevel::eValid);
    }

    void ctrlReceiveData(uint16_t length)
    {
      mem->stage = ctrl_stage_data_out;
      mem->bufstate[0].rx_remaining = length;
      set_rx_stat(0, usb::lowlevel::eValid);
    }

    void dataSendZlength()
    {
      mem->bufdesc[1].tx_count = 0;
      set_tx_stat(1, usb::lowlevel::eValid);
    }

    void receiveComplete(uint16_t endpoint)
    {
      mem->ctrl_epval = USB->EP0R; // do this to save the setup bit for ctrl endpoint
      clear_ctr_rx(endpoint);      // because it gets cleared here, but there were problems not clearing ctr_rx right away
      uint16_t count = mem->readEndpoint(endpoint);
      if (rx_handlers[endpoint](rx_buffer, count)) // does the handler want us to enable receiving again?
        set_rx_stat(endpoint, usb::lowlevel::eValid);
    }

    void transmit(uint16_t endpoint, const uint8_t *buffer)
    {
      BufferState_t *pbs = &mem->bufstate[endpoint];
      pbs->setTxAddress(buffer);
      mem->writeEndpoint(endpoint, buffer, pbs->tx_remaining);
      set_tx_stat(endpoint, usb::lowlevel::eValid);
      // now the host should be able to retrieve the buffer we just wrote.
      // when he did it will trigger a CTR_TX interrupt
      // we can call this function again if the buffer was bigger than max_paket
    }

    void transmitComplete(uint16_t endpoint)
    {
      clear_ctr_tx(endpoint);
      BufferState_t *pbs = &mem->bufstate[endpoint];
      uint16_t count = mem->bufdesc[endpoint].tx_count;
      uint16_t remain = pbs->tx_remaining;
      if (endpoint == 0)
      {
        uint16_t stage = mem->stage;
        OS_ASSERT((stage & 0x80) == 0, ERR_DEVICE_NOT_ENABLED); // transmitted something while we are in an OUT stage, huh?
        if (stage == ctrl_stage_state_in)
        { // we have completed sending a positive state to the host
          if (mem->address != 0)
          { // address has to be set after confimring its reception
            USB->DADDR = USB_DADDR_EF | mem->address;
            mem->address = 0;
          }
          ctrlReceiveReady(); // ready for next setup
          return;
        }
        if (stage == ctrl_stage_data_in && remain <= max_packet)
        { // we have send data and are done with the IN stage
          pbs->tx_remaining = 0;
          // ready the receiver to receive status stage ACK
          ctrlReceiveStatus();
          return;
        }
        // just continue sending the data which didn't fit into single packet
      }
      remain = remain - count;
      pbs->tx_remaining = remain;
      if (remain)
      { // still data left to send, advance read pointer and go on sending
        const uint8_t *buffer = mem->bufstate[endpoint].getTxAddress() + count;
        transmit(endpoint, buffer);
      }
      else if (endpoint == 1)
      {
        if (count == max_packet)
        { // last packet was exactly 64 bytes, send zero-length to confirm to host transaction is done
          dataSendZlength();
        }
        else
        {
          sig_tx_done.set();
        }
      }
    }
  } // namespace lowlevel
  
  void transmit(uint16_t endpoint, const uint8_t *buffer, uint16_t size)
  {
    BufferState_t *pbs = &mem->bufstate[endpoint];
    if (endpoint == 0)
    {
      OS_ASSERT(mem->stage == lowlevel::ctrl_stage_rdy, ERR_IMPOSSIBRU);
      mem->stage = lowlevel::ctrl_stage_data_in;
    }
    pbs->tx_remaining = size;
    lowlevel::transmit(endpoint, buffer);
  }

  typedef struct
  {
    uint8_t bmRequest;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
  } SetupRequest_t;

  typedef bool (*RequestHandler_t)(SetupRequest_t &req);

  bool notAvailable(SetupRequest_t &req)
  {
    req = req;
    // TODO
    //lowlevel::ctrlError();
    asm("bkpt 255");
    return false;
  }

  namespace Device
  {
    namespace Descriptor
    {
      typedef const uint8_t *(*DescriptorHandler_t)(SetupRequest_t &req, uint16_t &len);

      const uint8_t *device(SetupRequest_t &req, uint16_t &len)
      {
        req = req;
        static const uint8_t hUSBDDeviceDesc[18] = {
            0x12,       /* bLength */
            0x01,       /* bDescriptorType */
            0x00,       /* bcdUSB */
            0x02, 0x02, /* bDeviceClass */
            0x02,       /* bDeviceSubClass */
            0x00,       /* bDeviceProtocol */
            max_packet, /* bMaxPacketSize */
            0x83,       /* idVendor */
            0x04,       /* idVendor */
            0x40,       /* idVendor */
            0x57,       /* idVendor */
            0x00,       /* bcdDevice rel. 2.00 */
            0x02, 0x01, /* Index of manufacturer string */
            0x02,       /* Index of product string */
            0x03,       /* Index of serial number string */
            0x01        /* bNumConfigurations */
        };              /* USB_DeviceDescriptor */
        len = sizeof(hUSBDDeviceDesc);
        return hUSBDDeviceDesc;
      }

      const uint8_t *configuration(SetupRequest_t &req, uint16_t &len)
      {
        req = req;
        static const uint8_t USBD_CDC_CfgFSDesc[0x43] = {
            /*Configuration Descriptor*/
            0x09,       /* bLength: Configuration Descriptor size */
            0x02,       /* bDescriptorType: Configuration */
            0x43,       /* wTotalLength:no of returned bytes */
            0x00, 0x02, /* bNumInterfaces: 2 interface */
            0x01,       /* bConfigurationValue: Configuration value */
            0x00,       /* iConfiguration: Index of string descriptor describing the configuration */
            0xC0,       /* bmAttributes: self powered */
            0x32,       /* MaxPower 0 mA */
            /*---------------------------------------------------------------------------*/
            /*Interface Descriptor */
            0x09, /* bLength: Interface Descriptor size */
            0x04, /* bDescriptorType: Interface */
            /* Interface descriptor type */
            0x00, /* bInterfaceNumber: Number of Interface */
            0x00, /* bAlternateSetting: Alternate setting */
            0x01, /* bNumEndpoints: One endpoints used */
            0x02, /* bInterfaceClass: Communication Interface Class */
            0x02, /* bInterfaceSubClass: Abstract Control Model */
            0x01, /* bInterfaceProtocol: Common AT commands */
            0x00, /* iInterface: */

            /*Header Functional Descriptor*/
            0x05, /* bLength: Endpoint Descriptor size */
            0x24, /* bDescriptorType: CS_INTERFACE */
            0x00, /* bDescriptorSubtype: Header Func Desc */
            0x10, /* bcdCDC: spec release number */
            0x01,

            /*Call Management Functional Descriptor*/
            0x05, /* bFunctionLength */
            0x24, /* bDescriptorType: CS_INTERFACE */
            0x01, /* bDescriptorSubtype: Call Management Func Desc */
            0x00, /* bmCapabilities: D0+D1 */
            0x01, /* bDataInterface: 1 */

            /*ACM Functional Descriptor*/
            0x04, /* bFunctionLength */
            0x24, /* bDescriptorType: CS_INTERFACE */
            0x02, /* bDescriptorSubtype: Abstract Control Management desc */
            0x02, /* bmCapabilities */

            /*Union Functional Descriptor*/
            0x05, /* bFunctionLength */
            0x24, /* bDescriptorType: CS_INTERFACE */
            0x06, /* bDescriptorSubtype: Union func desc */
            0x00, /* bMasterInterface: Communication class interface */
            0x01, /* bSlaveInterface0: Data Class Interface */

            /*Endpoint 2 Descriptor*/
            0x07,            /* bLength: Endpoint Descriptor size */
            0x05,            /* bDescriptorType: Endpoint */
            0x82,            /* bEndpointAddress */
            0x03,            /* bmAttributes: Interrupt */
            max_cdc_command, /* wMaxPacketSize: */
            0x00, 0x10,      /* bInterval: */
            /*---------------------------------------------------------------------------*/
            /*Data class interface descriptor*/
            0x09, /* bLength: Endpoint Descriptor size */
            0x04, /* bDescriptorType: */
            0x01, /* bInterfaceNumber: Number of Interface */
            0x00, /* bAlternateSetting: Alternate setting */
            0x02, /* bNumEndpoints: Two endpoints used */
            0x0A, /* bInterfaceClass: CDC */
            0x00, /* bInterfaceSubClass: */
            0x00, /* bInterfaceProtocol: */
            0x00, /* iInterface: */

            /*Endpoint OUT Descriptor*/
            0x07,       /* bLength: Endpoint Descriptor size */
            0x05,       /* bDescriptorType: Endpoint */
            0x01,       /* bEndpointAddress */
            0x02,       /* bmAttributes: Bulk */
            max_packet, /* wMaxPacketSize: */
            0x00, 0x00, /* bInterval: ignore for Bulk transfer */

            /*Endpoint IN Descriptor*/
            0x07,       /* bLength: Endpoint Descriptor size */
            0x05,       /* bDescriptorType: Endpoint */
            0x81,       /* bEndpointAddress */
            0x02,       /* bmAttributes: Bulk */
            max_packet, /* wMaxPacketSize: */
            0, 0x00     /* bInterval: ignore for Bulk transfer */
        };

        len = sizeof(USBD_CDC_CfgFSDesc);
        return USBD_CDC_CfgFSDesc;
      }

      const uint8_t *string(SetupRequest_t &req, uint16_t &len)
      {
        // static const uint8_t dummy[] = { 0x02, 0x03 }; // not sure why this is here
        static const uint8_t USBD_LangIDDesc[] = {0x04, 0x03, 0x09, 0x04};
        static const uint8_t USBD_Manu[] = {6, 0x03, 'S', 0, 'T', 0};
        static const uint8_t USBD_PRODUCT_STRING[] = {46, 0x03, 'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, ' ', 0, 'V', 0, '-', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'b',
                                                      0, 'y', 0, ' ', 0, 's', 0, 'y', 0, 'n', 0, 's', 0, 'i', 0};
        static uint8_t USBD_SERIAL[18] = {
            18,
            0x03,
        };
        static const uint8_t *strings[] = {USBD_LangIDDesc, USBD_Manu, USBD_PRODUCT_STRING, USBD_SERIAL};
        uint16_t strnum = req.wValue & 0xFF;
        if (strnum < (sizeof(strings) / sizeof(void *)))
        {
          const uint8_t *str = strings[strnum];
          if (strnum == 3)
          {
            uint32_t devid = (*(uint32_t *)0x1FFFF7E8) + (*(uint32_t *)0x1FFFF7EC) + (*(uint32_t *)0x1FFFF7F0);
            mtl::IntToUnicode(devid, (uint8_t *)(str + 2), 8);
          }
          len = str[0];
          return str;
        }
        len = 0;
        return 0;
      }

      const DescriptorHandler_t descriptor_requesthandler[] = {&device, &configuration, &string};
    } // namespace Descriptor

    bool handle(SetupRequest_t &req);

    // opcode 0x00
    bool getStatus(SetupRequest_t &req)
    {
      req = req;
      uint16_t status = mem->status;
      transmit(0, (const uint8_t *)&status, 2);
      return true;
    }

    // opcode 0x01
    bool clrFeature(SetupRequest_t &req)
    {
      // TODO
      req = req;
      asm("bkpt 255");
      return false;
    }

    // opcode 0x03
    bool setFeature(SetupRequest_t &req)
    {
      // TODO
      req = req;
      asm("bkpt 255");
      return false;
    }

    // opcode 0x05
    bool setAddress(SetupRequest_t &req)
    {
      if ((req.wIndex == 0) && (req.wLength == 0))
      {
        // temporarily save the new address and ACK
        // set the deivce address only after the setup stage is completed
        mem->address = req.wValue;
        lowlevel::ctrlSendStatus();
        return true;
      }
      return false;
    }

    // opcode 0x06
    bool getDescriptor(SetupRequest_t &req)
    {
      uint16_t len = 0;
      // setting a buffer of zero is ok, if the request couldn't be found, nothing will be send
      const uint8_t *pbuf = 0;
      uint16_t x = (req.wValue >> 8) - 1;
      if (x < (sizeof(Descriptor::descriptor_requesthandler) / sizeof(void *)))
      {
        pbuf = Descriptor::descriptor_requesthandler[x](req, len);
      }
      if (len)
      {
        if (req.wLength < len)
          len = req.wLength;
        transmit(0, pbuf, len);
        return true;
      }
      return false;
    }

    // opcode 0x08
    bool getConfiguration(SetupRequest_t &req)
    {
      // TODO
      req = req;
      asm("bkpt 255");
      return false;
    }

    // opcode 0x09
    bool setConfiguration(SetupRequest_t &req)
    {
      bool success = false;
      if (USB->DADDR != USB_DADDR_EF)
      {
        uint16_t cfg = req.wValue;
        if (cfg == mem->config)
        { // allready in correct configuration
          success = true;
        }
        else if (cfg == 1)
        { // currently uncofigured, we want configured
          mem->bufstate[1].tx_remaining = 0;
          lowlevel::initial(1, 1, usb::lowlevel::eBulk);
          lowlevel::set_rx_stat(1, usb::lowlevel::eValid, true);
          lowlevel::set_tx_stat(1, usb::lowlevel::eNak, true);
          mem->bufstate[2].tx_remaining = 0;
          lowlevel::initial(2, 2, usb::lowlevel::eInterrupt);
          lowlevel::set_tx_stat(2, usb::lowlevel::eNak, true);
          success = true;
        }
        else if (cfg == 0)
        { // currently configured, we want to uncofigure
          lowlevel::set_rx_stat(1, usb::lowlevel::eDisabled, true);
          lowlevel::set_tx_stat(1, usb::lowlevel::eDisabled, true);
          lowlevel::set_tx_stat(2, usb::lowlevel::eDisabled, true);
          success = true;
        }
        if (success)
        {
          mem->config = cfg;
          lowlevel::ctrlSendStatus();
        }
      }
      return success;
    }

    const RequestHandler_t device_requesthandler[] = {&getStatus, &clrFeature, &notAvailable, &setFeature, &notAvailable, &setAddress, &getDescriptor,
                                                      &notAvailable, &getConfiguration, &setConfiguration};
  } /* namespace Device */

  namespace Interface
  {
    typedef struct
    {
      uint32_t baudrate;
      uint8_t stopbits;
      uint8_t parity;
      uint8_t bitsperbyte;
    } LineEncoding_t;

    static LineEncoding_t linenecoding = {9600, 0, 0, 8};

    bool handle(SetupRequest_t &req);
  } /* namespace Interface */

  namespace Endpoint
  {
    bool handle(SetupRequest_t &req);
  } /* namespace Endpoint */

  const RequestHandler_t class_requesthandler[] = {&Device::handle, &Interface::handle, &Endpoint::handle};

  void requestStallEndpoint(uint16_t endpoint)
  {
    // no idea whats the point of this
    if (endpoint == 0)
      lowlevel::set_rx_stat(endpoint, lowlevel::eStall);
    lowlevel::set_tx_stat(endpoint, lowlevel::eStall);
  }

  void isr_lp();
} // namespace usb

bool usb::Device::handle(usb::SetupRequest_t &req)
{
  if (req.bRequest < (sizeof(device_requesthandler) / sizeof(void *)))
    return device_requesthandler[req.bRequest](req);
  else
    return false;
}

bool usb::Interface::handle(usb::SetupRequest_t &req)
{
  // line encodings and other bullshitery we dont need, but Hosts want to configure us
  if (req.wLength == 0)
  {
    lowlevel::ctrlSendStatus();
    return true;
  }
  else if ((req.bmRequest & 0x60) == 0x20) // request type class
  {
    if (req.bmRequest & 0x80)
    { // get request
      switch (req.bRequest)
      {
      case 0x21: // get lineencoding
        transmit(0, (uint8_t *)&linenecoding, req.wLength);
        break;
      default:
        return false;
      }
    }
    else
    { // set request
      //if(req.bRequest == 0x20){ // set line coding. garbage!
      mem->ctrl_opcode = req.bRequest;
      lowlevel::ctrlReceiveData(req.wLength);
      //} else if(req.bRequest == 0x22){ // set line status, useless!
      //  lowlevel::ctrlSendStatus();
      //}
    }
    return true;
  }
  return false;
}

bool usb::Endpoint::handle(usb::SetupRequest_t &req)
{
  req = req;
  // TODO
  asm("bkpt 255");
  return false;
}

bool usb::rxCtrlEndpoint(const uint8_t *buffer, uint16_t size)
{
  // received on the endpoint, check if it is a setup paket
  if (mem->ctrl_epval & USB_EP0R_SETUP)
  {
    mem->stage = lowlevel::ctrl_stage_rdy;
    SetupRequest_t *req = (SetupRequest_t *)buffer;
    if ((req->bmRequest & 0x1F) < (sizeof(class_requesthandler) / sizeof(void *)))
    {
      if (!class_requesthandler[req->bmRequest & 0x1F](*req))
        lowlevel::ctrlError();
    }
    else
    {
      requestStallEndpoint(req->bmRequest & 0x80);
    }
  }
  else if (mem->stage == usb::lowlevel::ctrl_stage_data_out)
  { // received some none setup data
    uint16_t remain = mem->bufstate[0].rx_remaining - size;
    if (remain == 0)
    { // done now, send an ACK to the host if the operaation was alright
      bool success = false;
      switch (mem->ctrl_opcode)
      {
      case 0x20: // set line encoding
        std::memcpy(&Interface::linenecoding, buffer, sizeof(Interface::LineEncoding_t));
        mem->ctrl_opcode = 0xFF;
        success = true;
        break;
      }

      if (success)
        lowlevel::ctrlSendStatus();
      else
        lowlevel::ctrlError();
    }
    else
    { // still some data to go
      lowlevel::ctrlReceiveData(remain);
    }
  }
  else if (mem->stage == usb::lowlevel::ctrl_stage_state_out)
  {                               // no setup and size == 0, check if it is an ACK from host
    lowlevel::ctrlReceiveReady(); // all good, ready for next transaction
  }
  else
  {
    // maybe assert error herre?
    OS_ASSERT(0, ERR_IMPOSSIBRU);
  }
  return false; // never use default receiver enable
}

bool usb::rxCdcEndpoint(const uint8_t *buffer, uint16_t size)
{
  if (rxringbuffer.batchPush_isr(buffer, size, false) == false)
  {
    OS_ASSERT(true == false, ERR_BUFFER_OVERFLOW); // should never happen
  }
  // assert the signal since we changed something on the rx buffer
  sig_rx_buff.set();
  // check if we have enough space left to write another full packet
  if (rxringbuffer.remainingSpace() < max_packet)
  {
    mem->data_rx_off = 1;
    return false; // not enough space, dont enable receive on endpoint again
  }
  return true;
}

bool usb::rxCmdEndpoint(const uint8_t *buffer, uint16_t size)
{
  OS_ASSERT(true == false, ERR_IMPOSSIBRU);
  buffer = buffer;
  size = size;
  // the cmd endpoint is only in, this should never ever occur!
  return false; // just dont activate the receiver, ever
}

void usb::isr_lp()
{
  // serve data transfer interrupts
  while (USB->ISTR & USB_ISTR_CTR)
  {
    uint16_t ep_num = USB->ISTR & 0x1F;
    uint16_t host2device = ep_num >> 4;
    ep_num &= 0xF;
    if (host2device)
      usb::lowlevel::receiveComplete(ep_num);
    else
      usb::lowlevel::transmitComplete(ep_num);
  }
  // reset the usb device
  if (USB->ISTR & USB_ISTR_RESET)
  {
    USB->ISTR = ~uint16_t(USB_ISTR_RESET);
    usb::mem->config = 0;         // reset active config
    lowlevel::ctrlReceiveReady(); // enable IN and OUT of CTRL Endpoint0
    USB->DADDR = USB_DADDR_EF;    // set address to 0 and enable the usb device
  }
  // wake up from suspend
  if (USB->ISTR & USB_ISTR_WKUP)
  {
    // enable IRQs
    USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_WKUPM | USB_CNTR_SUSPM;
    USB->ISTR = ~uint16_t(USB_ISTR_WKUP);
  }
  // suspend on bus detected, force suspend the device
  if (USB->ISTR & USB_ISTR_SUSP)
  {
    USB->ISTR = ~uint16_t(USB_ISTR_SUSP);
    USB->CNTR |= USB_CNTR_FSUSP;
  }
}

void UsbCdc::init()
{
  RCC->APB1ENR |= RCC_APB1ENR_USBEN; // enable APB clock
  usb::mem->bufdesc[0].setTX(usb::mem->buffer_ctrl_ep_in);
  usb::mem->bufdesc[0].setRx(usb::mem->buffer_ctrl_ep_out, usb::max_packet);
  usb::mem->bufdesc[1].setTX(usb::mem->buffer_cdc_in);
  usb::mem->bufdesc[1].setRx(usb::mem->buffer_cdc_out, usb::max_packet);
  usb::mem->bufdesc[2].setTX(usb::mem->buffer_cdc_com_in);
  usb::mem->address = 0;     // no address
  usb::mem->status = 0x01;   // self powered
  usb::mem->config = 0;      // no config
  usb::mem->data_rx_off = 0; // its on
  usb::mem->ctrl_opcode = 0xFF;
  // setup the eventset
  usb::sig_rx_buff.init();
  usb::sig_tx_done.init();
  usb::tx_mutex.init();
  // enable gpio
  Gpio pin_dm('A', 11);
  pin_dm.mode(Gpio::in_pullup_pulldown, Gpio::Input);
  pin_dm.set(); // pull up
  Gpio pin_dp('A', 12);
  pin_dp.mode(Gpio::in_pullup_pulldown, Gpio::Input);
  pin_dp.set(); // pull up
  // reset the usb device
  USB->CNTR = USB_CNTR_FRES; // reset everything
  USB->CNTR = 0;
  USB->ISTR = 0; // clear any interrupts
  // enable IRQs
  USB->CNTR = USB_CNTR_CTRM | USB_CNTR_RESETM | USB_CNTR_WKUPM | USB_CNTR_SUSPM;
  // set irq priority highest possible with beeing able to call free rtos functions
  Core::enable_isr(USB_LP_CAN1_RX0_IRQn, 128);
}

// returns the number of bytes in the inbuffer
uint16_t UsbCdc::in_avail()
{
  return usb::rxringbuffer.in_avail();
}

// look at some fancy values in the future
bool UsbCdc::peek(uint8_t &val, uint32_t offset)
{
  return usb::rxringbuffer.peek(val, offset);
}

// remove up to count values from the buffer
void UsbCdc::flush(uint32_t count)
{
  usb::rxringbuffer.flush(count);
  if (usb::mem->data_rx_off && (usb::rxringbuffer.remainingSpace() >= usb::max_packet))
  {
    usb::mem->data_rx_off = 0; // we freed enough space in the buffer for a full packet, enable the rx if it was off
    usb::lowlevel::set_rx_stat(1, usb::lowlevel::eValid);
  }
}

// wait for a change in received data, returns false if it timed out
bool UsbCdc::waitData(OS_TIME timeout)
{
  return usb::sig_rx_buff.wait(timeout);
}

// reads as much as possible
uint16_t UsbCdc::read(uint8_t *data, uint16_t size)
{
  if (size)
  {
    if (size == 1)
    {
      if (!usb::rxringbuffer.pop(*data))
        size = 0;
    }
    else
    {
      size = usb::rxringbuffer.batchPop(data, size);
    }
    if (usb::mem->data_rx_off && (usb::rxringbuffer.remainingSpace() >= usb::max_packet))
    {
      usb::mem->data_rx_off = 0; // we freed enough space in the buffer for a full packet, enable the rx if it was off
      usb::lowlevel::set_rx_stat(1, usb::lowlevel::eValid);
    }
  }
  return size;
}

// read until "\n" is detected, including the "\n" blocks up to timeout.
// if timed out, nothing will be written to buffer and returns 0
// returns -1 if the buffer was to small
int32_t UsbCdc::readline(uint8_t *data, uint16_t size, OS_TIME timeout)
{
  if (timeout != INT32_MAX)
  {
    timeout = System::milliseconds() + timeout;
  }

  uint16_t checked = 0;
  while (true)
  {
    uint16_t bc = in_avail();
    if (bc != 0 && bc != checked)
    {
      while (checked < bc)
      {
        uint8_t crctr;
        peek(crctr, checked);
        // we peeked at a valid position in the inputbuffer. if this is a "\n" we are done
        // if we can't find it, we update our "checked" to examine the next byte
        // if we reach the end of the current input, we go back to sleep and do this only again, if in_avail is != from checked
        if (crctr == '\n')
        {
          if (checked < size)
          {
            size = usb::rxringbuffer.batchPop(data, checked + 1);
            if (usb::mem->data_rx_off && (usb::rxringbuffer.remainingSpace() >= usb::max_packet))
            {
              usb::mem->data_rx_off = 0; // we freed enough space in the buffer for a full packet, enable the rx if it was off
              usb::lowlevel::set_rx_stat(1, usb::lowlevel::eValid);
            }
            return size;
          }
          else
          {
            return -1; // line to big for buffer
          }
        }
        ++checked;
      }
    }
    else
    {
      if (!waitData(10) && timeout < System::milliseconds())
      {
        // timed out and ran out of time
        return 0;
      }
    }
  }
}

// blocks until the buffer was written
void UsbCdc::write(const uint8_t *data, uint16_t size)
{
  usb::tx_mutex.lock();
  while (usb::mem->config != 1)
  {
    // usb device not numerated by host
  }
  usb::transmit(1, data, size);
  // managed to start the transmission, wait for donezo signal again
  while (false == !true)
  {
    if (usb::sig_tx_done.wait(100))
      break;
    if (usb::mem->bufstate[1].tx_remaining == 0)
      break;
  }
  usb::tx_mutex.unlock();
}

extern "C"
{
  void USB_LP_CAN1_RX0_IRQHandler()
  {
    Core::enter_isr();
    usb::isr_lp();
    Core::leave_isr();
  }
} // END extern C
#endif
