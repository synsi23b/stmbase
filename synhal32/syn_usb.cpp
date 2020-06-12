#include "synhal.h"
#if (SYN_ENABLE_USBRPC != 0)
using namespace syn;

#include "../../src/synrpc_usbcon.h"

#ifdef STM32F103
namespace usb
{
  // this signal is set when a transmission on endpoint 1 IN is possible
  static Signal sig_tx_ready;
  // writepointer to currently reserved mail
  static uint32_t *packetbuffer;

  static const uint32_t max_packet = 64;
  static const uint32_t max_descriptor = 68; // actually 67, but we need to round up
  static const uint32_t max_buffer_cdc = 128;
  static const uint32_t max_cdc_command = 8;

  // start of the special usb memory region. 512 byte
  static uint32_t *const usb_memory_start = (uint32_t *)0x40006000;

  // forward declaration of data reception packet handlers
  void rxCtrlEndpoint();
  void rxCdcEndpoint();

  typedef struct
  {
    uint16_t tx_address;
    uint16_t padding_0;
    uint16_t tx_count;
    uint16_t padding_1;
    uint16_t rx_address;
    uint16_t padding_2;
    volatile uint16_t rx_count;
    uint16_t padding_3;

    void setTX(uint32_t *buffer)
    {
      tx_address = (buffer - usb_memory_start) * 2;
    }

    void setRx(uint32_t *buffer)
    {
      rx_address = (buffer - usb_memory_start) * 2;
    }

    void setRx(uint32_t *buffer, uint16_t size)
    {
      rx_address = (buffer - usb_memory_start) * 2;
      setRxCount(size);
    }

    void setTxCount(uint16_t size)
    {
      if (size > max_packet)
        size = max_packet;
      tx_count = size;
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

    uint16_t getRxCount() const
    {
      return rx_count & 0x3FF;
    }

    uint16_t getTxCount() const
    {
      return tx_count & 0x3FF;
    }

    void advanceTx(uint16_t remaining)
    {
      tx_address += tx_count;
      tx_count = remaining;
    }
  } BufferTableEntry_t;

  typedef struct
  {
    volatile uint16_t rx_remaining;
    uint16_t padding_1;
    volatile uint16_t tx_remaining;
    uint16_t padding_3;
  } BufferState_t;

  typedef struct
  {
    // for reference https://beyondlogic.org/usbnutshell/usb4.shtml
    // this is a description of the special memory area reserved
    // for USB and CAN communication. First the internal buffers are described
    // by BufferTableEntries, Hardwarwe will read them to know where to expect
    // sending and receiving packages.
    // because this is a full speed device, the control endpoint has to receive
    // or send up to 64 bytes.
    // the configuration we are doing is a cdc devie, so we have one endpoint for
    // bulk data in and out, 64 byte packets each.
    // and an interrupt endpoint, but we are not using it.
    //
    // One Problem is, the memory is accessed as 32bit by the ABP, but actually
    // it is only 16 bit wide as the USB controler sees it. So we have to think
    // about padding bytes, and unfortunatelly, we can't just write directly to it.
    //
    // to shorten the time spend in the interrupt, we are going to cast the special
    // setuprequest package on the endpoint_0 buffer, as well as the LineEncoding package
    // that is the only one we really have to answer to windows / linux to not appear broken.
    //
    // anyway, first in the memory we store the buffer description
    BufferTableEntry_t buffer_table[2]; // 16 bytes
    BufferState_t buffer_state[2];      // 8 bytes
    // following are the buffers
    // since the control endpoint doesn't have to receive and send at the same time,
    // we will use the same buffer for sending and receiving.
    uint32_t buffer_ctrl_ep[max_descriptor / 2]; // 68 bytes
    uint32_t buffer_cdc_out_0[max_packet / 2];   // 64
    uint32_t buffer_cdc_out_1[max_packet / 2];   // 64
    uint32_t buffer_cdc_in_0[max_packet / 2];    // 64
    uint32_t buffer_cdc_in_1[max_packet / 2];    // 64
    // cdc interrupts from microcontroller to host, but we don't really need it.
    //uint32_t buffer_cdc_com_in[max_cdc_command];
    volatile uint16_t temp_address; // store the device address when it gets set by host
    uint16_t padding_1;
    volatile uint16_t usb_config; // store the current selected config
    uint16_t padding_2;
    volatile uint16_t usb_status; // store the device status (usb standard)
    uint16_t padding_3;
    volatile uint16_t ctrl_stage; // store the current stage of ctrl endpoint
    uint16_t padding_4;
    volatile uint16_t data_rx_off; // store the state of the data rx (on /off) for flow control (host to fast)
    uint16_t padding_5;
    volatile uint16_t ctrl_opcode; // store opcode from setup request when we are receiving ctrl data during in stage
    uint16_t padding_6;            // final padding to make sure we would can test over size off the buffer

    void readUsbRpcHeader(uint32_t *rx_buffer)
    {
      uint16_t *writeptr = (uint16_t *)rx_buffer;
      uint32_t *readptr = buffer_cdc_out_0;
      *writeptr++ = *readptr++;
      *writeptr++ = *readptr++;
    }

    void readUsbRpcPayload_0(uint32_t *rx_buffer, uint16_t count)
    {
      uint16_t *writeptr = (uint16_t *)(rx_buffer + 1);
      uint32_t *readptr = &buffer_cdc_out_0[2];
      count -= 4;
      uint32_t *end = readptr + ((count + 1) >> 1);
      while (readptr < end)
        *writeptr++ = *readptr++;
    }

    void readUsbRpcPayload_1(uint32_t *rx_buffer, uint16_t count)
    {
      uint16_t *writeptr = (uint16_t *)rx_buffer;
      uint32_t *readptr = buffer_cdc_out_1;
      uint32_t *end = readptr + ((count + 1) >> 1);
      while (readptr < end)
        *writeptr++ = *readptr++;
    }

    uint16_t readEndpoint(uint16_t endpoint, uint32_t *rx_buffer)
    {
      uint16_t count = buffer_table[endpoint].getRxCount();
      uint16_t *writeptr = (uint16_t *)rx_buffer;
      uint32_t *readptr = buffer_table[endpoint].getBufferRx();
      uint32_t *end = readptr + ((count + 1) >> 1);
      while (readptr < end)
        *writeptr++ = *readptr++;
      return count;
    }

    void writeEndpoint(uint16_t endpoint, const uint8_t *buffer, uint16_t size)
    {
      uint16_t *readptr = (uint16_t *)buffer;
      uint32_t *writeptr = buffer_table[endpoint].getBufferTx();
      uint16_t *end = readptr + ((size + 1) >> 1);
      while (readptr < end)
        *writeptr++ = *readptr++;
      buffer_table[endpoint].setTxCount(size);
    }
  } Memory_t;

  static Memory_t *const mem = (Memory_t *)0x40006000;

  namespace lowlevel
  {
    typedef volatile uint16_t *ep_reg_t;
    // return the specific endpoint register for r/w
    ep_reg_t getEpReg(uint16_t endpoint)
    {
      return (uint16_t *)((uint32_t *)USB_EP0R + endpoint);
    }

    // returns a sensible default value to write to the endpoint register.
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

    ep_reg_t initial(uint16_t endpoint, eEndpointType type)
    {
      ep_reg_t ep = getEpReg(endpoint);
      *ep = (uint16_t(type) << USB_EP0R_EP_TYPE_Pos) | endpoint;
      return ep;
    }

    void clear_ctr_rx(ep_reg_t ep_register)
    {
      uint16_t val = getDefault(*ep_register);
      val |= USB_EP0R_CTR_TX; // set tx, we only want to clear rx
      *ep_register = val;
    }

    void clear_ctr_tx(ep_reg_t ep_register)
    {
      uint16_t val = getDefault(*ep_register);
      val |= USB_EP0R_CTR_RX; // set rx, we only want to clear tx
      *ep_register = val;
    }

    enum eEndpointStatus
    {
      eDisabled = 0,
      eStall = 1,
      eNak = 2,
      eValid = 3
    };

    void set_rx_stat(ep_reg_t ep_register, eEndpointStatus status, bool clear_dtog = false)
    {
      uint16_t extramask;
      if (clear_dtog)
        extramask = USB_EPRX_STAT | USB_EP0R_DTOG_RX;
      else
        extramask = USB_EPRX_STAT;
      uint16_t val = getDefault(*ep_register, extramask);
      if (status & 0x1)
      {
        val ^= USB_EP0R_STAT_RX_0;
      }
      if (status & 0x2)
      {
        val ^= USB_EP0R_STAT_RX_1;
      }
      *ep_register = val | USB_EP0R_CTR_RX | USB_EP0R_CTR_TX;
    }

    void set_tx_stat(ep_reg_t ep_register, eEndpointStatus status, bool clear_dtog = false)
    {
      uint16_t extramask;
      if (clear_dtog)
        extramask = USB_EPTX_STAT | USB_EP0R_DTOG_TX;
      else
        extramask = USB_EPTX_STAT;
      uint16_t val = getDefault(*ep_register, extramask);
      if (status & 0x1)
      {
        val ^= USB_EP0R_STAT_TX_0;
      }
      if (status & 0x2)
      {
        val ^= USB_EP0R_STAT_TX_1;
      }
      *ep_register = val | USB_EP0R_CTR_RX | USB_EP0R_CTR_TX;
    }

    void ctrlError()
    {
      // TODO
      // stall the ctrl endpoint
      // only stall opposite direction
      // this is fucking weird man
      // it works right now so whatever
      set_rx_stat(&USB->EP0R, usb::lowlevel::eStall);
      set_tx_stat(&USB->EP0R, usb::lowlevel::eStall);
    }

    static const uint16_t ctrl_stage_rdy = 0;
    static const uint16_t ctrl_stage_data_in = 0x01;
    static const uint16_t ctrl_stage_data_out = 0x81;
    static const uint16_t ctrl_stage_state_in = 0x02;
    static const uint16_t ctrl_stage_state_out = 0x82;

    void ctrlSendStatus()
    {
      mem->ctrl_stage = ctrl_stage_state_in;
      mem->buffer_table[0].setTxCount(0);
      set_tx_stat(&USB->EP0R, usb::lowlevel::eValid);
    }

    void ctrlReceiveStatus()
    {
      mem->ctrl_stage = ctrl_stage_state_out;
      USB->EP0R = USB_EP0R_CTR_RX | USB_EP0R_CTR_TX | USB_EP0R_EP_TYPE_0 | USB_EP0R_EP_KIND;
      set_rx_stat(&USB->EP0R, usb::lowlevel::eValid);
    }

    void ctrlReceiveReady()
    {
      mem->ctrl_stage = ctrl_stage_rdy;
      USB->EP0R = USB_EP0R_CTR_RX | USB_EP0R_CTR_TX | USB_EP0R_EP_TYPE_0;
      mem->buffer_state[0].tx_remaining = 0;
      set_tx_stat(&USB->EP0R, usb::lowlevel::eNak);
      set_rx_stat(&USB->EP0R, usb::lowlevel::eValid);
    }

    void ctrlReceiveData(uint16_t length)
    {
      mem->ctrl_stage = ctrl_stage_data_out;
      mem->buffer_state[0].rx_remaining = length;
      set_rx_stat(&USB->EP0R, usb::lowlevel::eValid);
    }

    void ctrlStallEndpoint(bool stall_rx)
    {
      if (stall_rx)
      {
        lowlevel::set_rx_stat(&USB->EP0R, lowlevel::eStall);
      }
      lowlevel::set_tx_stat(&USB->EP0R, lowlevel::eStall);
    }

    void cdcSendZlength()
    {
      mem->buffer_table[1].tx_count = 0;
      set_tx_stat(&USB->EP1R, usb::lowlevel::eValid);
    }

    void receiveComplete(uint16_t endpoint)
    {
      if (endpoint == 0)
      {
        rxCtrlEndpoint();
      }
      else
      {
        rxCdcEndpoint();
      }
    }

    void ctrlTransmit(const uint8_t *buffer, uint16_t size)
    {
      OS_ASSERT(size <= max_descriptor, ERR_BUFFER_OVERFLOW);
      mem->buffer_state[0].tx_remaining = size;
      mem->buffer_table[0].setTX(mem->buffer_ctrl_ep);
      mem->writeEndpoint(0, buffer, size);
      mem->ctrl_stage = ctrl_stage_data_in;
      set_tx_stat(&USB->EP0R, usb::lowlevel::eValid);
    }

    void cdcTransmit(const uint8_t *buffer, uint16_t size)
    {
      OS_ASSERT(size <= max_buffer_cdc, ERR_BUFFER_OVERFLOW);
      mem->buffer_state[1].tx_remaining = size;
      mem->buffer_table[1].setTX(mem->buffer_cdc_in_0);
      mem->writeEndpoint(1, buffer, size);
      set_tx_stat(&USB->EP1R, usb::lowlevel::eValid);
    }

    void transmitComplete(uint16_t endpoint)
    {
      ep_reg_t ep = getEpReg(endpoint);
      clear_ctr_tx(ep);
      BufferState_t *pbs = &mem->buffer_state[endpoint];
      uint16_t remaining = pbs->tx_remaining;
      if (endpoint == 0)
      {
        // special handling for ctrl endpoint transmission
        uint16_t stage = mem->ctrl_stage;
        OS_ASSERT((stage & 0x80) == 0, ERR_DEVICE_NOT_ENABLED); // transmitted something while we are in an OUT stage, huh?
        if (stage == ctrl_stage_state_in)
        { // we have completed sending a positive state to the host
          if (mem->temp_address != 0)
          { // address has to be set after confirming its reception
            USB->DADDR = USB_DADDR_EF | mem->temp_address;
            mem->temp_address = 0;
          }
          ctrlReceiveReady(); // ready for next setup
          return;
        }
        if (stage == ctrl_stage_data_in && remaining <= max_packet)
        { // we have send data and are done with the IN stage
          pbs->tx_remaining = 0;
          // ready the receiver to receive status stage ACK
          ctrlReceiveStatus();
          return;
        }
        // just continue sending the data which didn't fit into single packet
      }
      uint16_t bytes_transmitted = mem->buffer_table[endpoint].getTxCount();
      remaining = remaining - bytes_transmitted;
      if (remaining)
      { // still data left to send, advance read pointer and go on sending
        mem->buffer_table[endpoint].advanceTx(remaining);
        set_tx_stat(ep, usb::lowlevel::eValid);
      }
      else if (endpoint == 1)
      {
        if (bytes_transmitted == max_packet)
        { // last packet was exactly 64 bytes, send zero-length to confirm to host transaction is done
          pbs->tx_remaining = 0;
          cdcSendZlength();
        }
        else
        {
          sig_tx_ready.set();
        }
      }
    }
  } // namespace lowlevel

  typedef struct
  {
    uint8_t bmRequest;
    uint8_t bRequest;
    uint16_t padding_0;
    uint16_t wValue;
    uint16_t padding_1;
    uint16_t wIndex;
    uint16_t padding_2;
    uint16_t wLength;
    uint16_t padding_3;
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

      static const uint8_t USBD_LangIDDesc[] = {0x04, 0x03, 0x09, 0x04};
      static const uint8_t USBD_Manu[] = {6, 0x03, 'S', 0, 'T', 0};
      static const uint8_t USBD_PRODUCT_STRING[] = {42, 0x03, 'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0, ' ', 0, 'V', 0, '-', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'b',
                                                    0, 'y', 0, ' ', 0, 's', 0, 'y', 0, 'n', 0, 's', 0, 'i', 0};
      static uint8_t USBD_SERIAL[50] = {
          50,
          0x03,
      };

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
        static const uint8_t USBD_CDC_CfgFSDesc[67] = {
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
            0x00, 0x00  /* bInterval: ignore for Bulk transfer */
        };

        len = sizeof(USBD_CDC_CfgFSDesc);
        return USBD_CDC_CfgFSDesc;
      }

      const uint8_t *string(SetupRequest_t &req, uint16_t &len)
      {
        static const uint8_t *strings[] = {USBD_LangIDDesc, USBD_Manu, USBD_PRODUCT_STRING, USBD_SERIAL};
        uint16_t strnum = req.wValue & 0xFF;
        if (strnum < (sizeof(strings) / sizeof(void *)))
        {
          const uint8_t *str = strings[strnum];
          len = str[0];
          return str;
        }
        len = 0;
        return 0;
      }

      void init()
      {
        // read out this chips unique serial number and create device id using it
        mtl::IntToUnicode((*(uint32_t *)0x1FFFF7E8), (USBD_SERIAL + 2), 8);
        mtl::IntToUnicode((*(uint32_t *)0x1FFFF7EC), (USBD_SERIAL + 18), 8);
        mtl::IntToUnicode((*(uint32_t *)0x1FFFF7F0), (USBD_SERIAL + 34), 8);
      }

      const DescriptorHandler_t descriptor_requesthandler[] = {&device, &configuration, &string};
    } // namespace Descriptor

    bool handle(SetupRequest_t &req);

    // opcode 0x00
    bool getStatus(SetupRequest_t &req)
    {
      req = req;
      lowlevel::ctrlTransmit((const uint8_t *)&mem->usb_status, 2);
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
        mem->temp_address = req.wValue;
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
        lowlevel::ctrlTransmit(pbuf, len);
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
        if (cfg == mem->usb_config)
        { // allready in correct configuration
          success = true;
        }
        else if (cfg == 1)
        { // currently uncofigured, we want configured
          mem->buffer_state[1].tx_remaining = 0;
          lowlevel::initial(1, usb::lowlevel::eBulk);
          lowlevel::set_rx_stat(&USB->EP1R, usb::lowlevel::eValid, true);
          lowlevel::set_tx_stat(&USB->EP1R, usb::lowlevel::eNak, true);
          //mem->buffer_state[2].tx_remaining = 0;
          lowlevel::initial(2, usb::lowlevel::eInterrupt);
          lowlevel::set_tx_stat(&USB->EP2R, usb::lowlevel::eNak, true);
          usb::sig_tx_ready.set(); // set the signal, we are configured, we can send
          success = true;
        }
        else if (cfg == 0)
        { // currently configured, we want to uncofigure
          lowlevel::set_rx_stat(&USB->EP1R, usb::lowlevel::eDisabled, true);
          lowlevel::set_tx_stat(&USB->EP1R, usb::lowlevel::eDisabled, true);
          lowlevel::set_tx_stat(&USB->EP2R, usb::lowlevel::eDisabled, true);
          usb::sig_tx_ready.try_wait(); // remove the ready signal if applicable
          success = true;
        }
        if (success)
        {
          mem->usb_config = cfg;
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

    // we have to remember the line encoding set by the host
    // start out with some default value.
    static LineEncoding_t linenecoding = {9600, 0, 0, 8};

    // when receiving a new lineencoding, we have to read it from
    // the special usb memory and have to watch the padding bytes
    typedef struct
    {
      uint16_t baudrate_lo;
      uint16_t padding_0;
      uint16_t baudrate_hi;
      uint16_t padding_1;
      uint8_t stopbits;
      uint8_t parity;
      uint16_t padding_2;
      uint8_t bitsperbyte;
      uint16_t padding_3;
    } LineEncodingRequest_t;

    bool handle(SetupRequest_t &req);
  } /* namespace Interface */

  namespace Endpoint
  {
    bool handle(SetupRequest_t &req);
  } /* namespace Endpoint */

  const RequestHandler_t class_requesthandler[] = {&Device::handle, &Interface::handle, &Endpoint::handle};

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
        lowlevel::ctrlTransmit((uint8_t *)&linenecoding, req.wLength);
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

void usb::rxCtrlEndpoint()
{
  bool is_setup = USB->EP0R & USB_EP0R_SETUP; // do this to save the setup bit for ctrl endpoint
  lowlevel::clear_ctr_rx(&USB->EP0R);         // because it gets cleared here, but there were problems not clearing ctr_rx right away
  if (is_setup)
  {
    mem->ctrl_stage = lowlevel::ctrl_stage_rdy;
    SetupRequest_t *req = (SetupRequest_t *)mem->buffer_ctrl_ep;
    uint16_t req_id = req->bmRequest & 0x1F;
    if (req_id < (sizeof(class_requesthandler) / sizeof(void *)))
    {
      if (!class_requesthandler[req_id](*req))
        lowlevel::ctrlError();
    }
    else
    {
      lowlevel::ctrlStallEndpoint(req->bmRequest & 0x80);
    }
  }
  else if (mem->ctrl_stage == usb::lowlevel::ctrl_stage_data_out)
  { // received some data packet of setup stage
    uint16_t remain = mem->buffer_state[0].rx_remaining - mem->buffer_table[0].getRxCount();
    if (remain == 0)
    { // done now, send an ACK to the host if the operaation was alright
      bool success = false;
      switch (mem->ctrl_opcode)
      {
      case 0x20: // set line encoding
        Interface::LineEncodingRequest_t *plenc = (Interface::LineEncodingRequest_t *)mem->buffer_ctrl_ep;
        Interface::linenecoding.baudrate = (uint32_t(plenc->baudrate_hi) << 16) | plenc->baudrate_lo;
        Interface::linenecoding.stopbits = plenc->stopbits;
        Interface::linenecoding.parity = plenc->parity;
        Interface::linenecoding.bitsperbyte = plenc->bitsperbyte;
        // reset ctrl opcode byte
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
  else if (mem->ctrl_stage == usb::lowlevel::ctrl_stage_state_out)
  {                               // no setup and size == 0, check if it is an ACK from host
    lowlevel::ctrlReceiveReady(); // all good, ready for next transaction
  }
  else
  {
    // maybe assert error herre?
    OS_ASSERT(0, ERR_IMPOSSIBRU);
  }
}

void mail_release()
{
  // if the buffer is zero, it means, we should release and try to get a new one
  UsbRpc::Handler::_mailbox.release();
  // packet was released, try to allocate a new one
  if (UsbRpc::Handler::_mailbox.try_reserve((UsbRpc::Packet **)&usb::packetbuffer))
  {
    *usb::packetbuffer = 0; // make sure to completely wipe packetheader
    // turn on the receiver again, we got space for messages
    set_rx_stat(&USB->EP1R, usb::lowlevel::eValid);
  }
  else
  {
    usb::packetbuffer = 0;
    // mailbox is full, dont enable receive on endpoint again
    usb::mem->data_rx_off = 1;
  }
}

void usb::rxCdcEndpoint()
{
  lowlevel::clear_ctr_rx(&USB->EP1R);

  OS_ASSERT(usb::packetbuffer != 0, ERR_NULL_POINTER);

  BufferState_t *pbs = &mem->buffer_state[1];

  uint16_t count = mem->buffer_table[1].getRxCount();

  if (pbs->rx_remaining == 0)
  {
    // this is a fresh packet, check if the packet is plausible
    mem->readUsbRpcHeader(usb::packetbuffer);
    UsbRpc::Packet *ppacket = (UsbRpc::Packet *)usb::packetbuffer;
    uint16_t expected_size = UsbRpc::Handler::plausible(*ppacket);
    if (expected_size >= count)
    {
      if (expected_size > count)
      {
        //this packet didn't fit, set the receving buffer to second rx buffer
        mem->buffer_table[1].setRx(mem->buffer_cdc_out_1);
        // and enable receiving
        set_rx_stat(&USB->EP1R, usb::lowlevel::eValid);
      }
      // this is a nice package, read the payload
      mem->readUsbRpcPayload_0(usb::packetbuffer, count);
      uint16_t remaining = expected_size - count;
      if (remaining == 0)
      {
        // this packet fit into a single transmission, release it to the handler
        mail_release();
      }
      else
      {
        OS_ASSERT(count == max_packet, ERR_FORBIDDEN);
        // package didn't fit, so set the remaining and it's done, wait for second part
        pbs->rx_remaining = remaining;
      }
    }
    else
    {
      // else this package has a malformed header, just re-enable receiving
      set_rx_stat(&USB->EP1R, usb::lowlevel::eValid);
    }
  }
  else
  {
    // the package is already started and the receiving buffer was moved to buffer_cdc_out_1
    // reset the buffer to cdc_0
    mem->buffer_table[1].setRx(mem->buffer_cdc_out_0);
    OS_ASSERT(pbs->rx_remaining == count, ERR_FORBIDDEN);
    pbs->rx_remaining = 0;
    // copy over the write pointer, than release the packet to re-enable the receiver, if there is space
    // after that, start to read from cdc_1 while cdc_0 can be written by the host already
    uint32_t *ptmp = usb::packetbuffer + 32;
    mail_release();
    // we can do it even thou the packet isn't fully written, because
    // this is an interrupt handler and there will be no task switch until its end
    mem->readUsbRpcPayload_1(ptmp, count);
  }
}

void usb::isr_lp()
{
  // serve data transfer interrupts
  if (USB->ISTR & USB_ISTR_CTR)
  {
    do
    {
      uint16_t ep_num = USB->ISTR & USB_ISTR_EP_ID;
      if (USB->ISTR & USB_ISTR_DIR)
        usb::lowlevel::receiveComplete(ep_num);
      else
        usb::lowlevel::transmitComplete(ep_num);
    } while (USB->ISTR & USB_ISTR_CTR);
  }
  else
  {
    // reset the usb device
    if (USB->ISTR & USB_ISTR_RESET)
    {
      USB->ISTR = ~uint16_t(USB_ISTR_RESET);
      usb::mem->usb_config = 0;     // reset active config
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
}

void UsbRpc::init()
{
  OS_ASSERT(sizeof(usb::Memory_t) < 1024, ERR_BUFFER_OVERFLOW);
  RCC->APB1ENR |= RCC_APB1ENR_USBEN; // enable APB clock
  usb::mem->buffer_table[0].setTX(usb::mem->buffer_ctrl_ep);
  usb::mem->buffer_table[0].setRx(usb::mem->buffer_ctrl_ep, usb::max_packet);
  usb::mem->buffer_table[1].setTX(usb::mem->buffer_cdc_in_0);
  usb::mem->buffer_table[1].setRx(usb::mem->buffer_cdc_out_0, usb::max_packet);
  usb::mem->temp_address = 0;  // no address
  usb::mem->usb_status = 0x01; // self powered
  usb::mem->usb_config = 0;    // not configured
  usb::mem->data_rx_off = 0;   // rx is ready after startup, mailbox empty
  usb::mem->ctrl_opcode = 0xFF;
  usb::Device::Descriptor::init();
  // setup the eventset
  usb::sig_tx_ready.init();
  // setup incoming packet buffer
  Handler::_mailbox.init();
  Handler::_mailbox.reserve((Packet **)&usb::packetbuffer);
  *usb::packetbuffer = 0;                     // make sure to completely wipe packetheader
  usb::mem->buffer_state[1].rx_remaining = 0; // fresh packet
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
  Core::enable_isr(USB_LP_CAN1_RX0_IRQn, 8);
}

// blocks until the buffer was written, or until timeout, if non-zero
bool UsbRpc::write(const uint8_t *data, uint16_t size, uint32_t timeout)
{
  if (timeout != 0)
  {
    if (!usb::sig_tx_ready.wait(timeout))
    {
      return false;
    }
  }
  else
  {
    usb::sig_tx_ready.wait();
  }
  usb::lowlevel::cdcTransmit(data, size);
  return true;
}

void UsbRpc::_enable_rx()
{
  if (usb::mem->data_rx_off != 0 && usb::packetbuffer == 0)
  {
    if (Handler::_mailbox.try_reserve((Packet **)&usb::packetbuffer))
    {
      usb::mem->data_rx_off = 0;
      set_rx_stat(&USB->EP1R, usb::lowlevel::eValid);
    }
  }
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
#endif // STM32f103
#ifdef STM32F401xC

#include "../cube_usb_cdc/usb_device.h"

void UsbRpc::init()
{
  RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
  // usb::mem->buffer_table[0].setTX(usb::mem->buffer_ctrl_ep);
  // usb::mem->buffer_table[0].setRx(usb::mem->buffer_ctrl_ep, usb::max_packet);
  // usb::mem->buffer_table[1].setTX(usb::mem->buffer_cdc_in_0);
  // usb::mem->buffer_table[1].setRx(usb::mem->buffer_cdc_out_0, usb::max_packet);
  // usb::mem->temp_address = 0;  // no address
  // usb::mem->usb_status = 0x01; // self powered
  // usb::mem->usb_config = 0;    // not configured
  // usb::mem->data_rx_off = 0;   // rx is ready after startup, mailbox empty
  // usb::mem->ctrl_opcode = 0xFF;
  // usb::Device::Descriptor::init();
  // // setup the eventset
  // usb::sig_tx_ready.init();
  // // setup incoming packet buffer
  // Handler::_mailbox.init();
  // Handler::_mailbox.reserve((Packet **)&usb::packetbuffer);
  // *usb::packetbuffer = 0;                     // make sure to completely wipe packetheader
  // usb::mem->buffer_state[1].rx_remaining = 0; // fresh packet
  /**USB_OTG_FS GPIO Configuration    
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP 
  */
  Gpio pin_dm('A', 11);
  pin_dm.mode(Gpio::out_alt_push_pull, Gpio::MHz_100, Gpio::OTG_FS);
  Gpio pin_dp('A', 12);
  pin_dp.mode(Gpio::out_alt_push_pull, Gpio::MHz_100, Gpio::OTG_FS);
  // set irq priority highest possible with beeing able to call free rtos functions
  Core::enable_isr(OTG_FS_IRQn, 8);
  // initialize USB using cube mx firmware
  MX_USB_DEVICE_Init();
}

extern "C"
{
  extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
/**
  * @brief This function handles USB On The Go FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

void Error_Handler(void)
{
  OS_ASSERT(true == false, ERR_CUBE_HAL_SUCKS);
}
}
#endif // STM32F401xC
#endif // enable USBRPC