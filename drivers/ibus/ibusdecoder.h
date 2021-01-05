#ifndef IBUS_DECODER_H
#define IBUS_DECODER_H
#include "stm8s_hal.h"

#include "assemblerhelpers.h"
 
// ready little endian words to write the channesl
#define IBUS_SYNC_WORD 0x2040
#define IBUS_CHANNEL_MIN 0xE803
#define IBUS_CHANNEL_MED 0xDC05
#define IBUS_CHANNEL_MAX 0xD007

 class IbusDecoder
 {
 public:
   void init()
   {
     // initialize uart to correct values
     // 115200 8N1
     Uart::init(Uart::bd115200);
     // initialize sync byte only once
     syncbyte_ = IBUS_SYNC_WORD;
     // init all channels with 1500
     for (uint8_t i = 0; i < 14; ++i)
     {
       buffer_[i] = IBUS_CHANNEL_MED;
     }
     successfullparse_ = false;
   }

   // try to parse the data of the message, if there is something wrong with it, return false
   void parse(const uint8_t *inbuff, uint8_t rssi)
   {
     // map channel 1 to 4 directly with scaling
     ibus_decode_proportinal_channels(rssi, inbuff, buffer_);
     
     // ignore turnwheel and poti channel for this app
     
     // put the switches on some channels
     uint8_t switchbyte = inbuff[6];
     // switch E
     if(switchbyte & 0x01)
       buffer_[4] = IBUS_CHANNEL_MIN;
     else if(switchbyte & 0x02)
       buffer_[4] = IBUS_CHANNEL_MAX;
     else
       buffer_[4] = IBUS_CHANNEL_MED;
     // switch A
     buffer_[5] = (switchbyte & 0x04) ? IBUS_CHANNEL_MAX : IBUS_CHANNEL_MIN;
     // switch D
     buffer_[6] = (switchbyte & 0x08) ? IBUS_CHANNEL_MAX : IBUS_CHANNEL_MIN;
     // switch G
     buffer_[7] = (switchbyte & 0x10) ? IBUS_CHANNEL_MAX : IBUS_CHANNEL_MIN;
     // switch H
     buffer_[8] = (switchbyte & 0x20) ? IBUS_CHANNEL_MAX : IBUS_CHANNEL_MIN;
     
     checksum_ = ibus_calculate_checksum(buffer_);
     
     successfullparse_ = true;
   }

   bool frameReady() const
   {
     return successfullparse_;
   }

   // write the current buffer out without blocking
   void write_async()
   {
     if (successfullparse_)
     {
       Uart::write_async(&syncbyte_, 16);
       successfullparse_ = false;
     }
   }
 private:
   uint16_t syncbyte_;
   uint16_t buffer_[14];
   uint16_t checksum_;
   bool successfullparse_;
 };
#endif