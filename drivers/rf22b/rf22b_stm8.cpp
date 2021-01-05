
//#include "RF22B.h"
//#include "hal/exti.h"
//
//#ifndef NDEBUG
//#include <stdio.h>
//#endif
//
//RF22B rf22b;
//
//// SSP can operate at up to 10MHz
//#define SSP_PORT_NUM        1
//#define CHIP_SELECT_PORT    'A'
//#define CHIP_SELECT_PIN     4
//#define SSP_CONFIG_VAL      SSP_CONFIG_MACRO(2, 0, 0, 0)  // 9MHz 8bit spi format
//
//#define EXTI_PIN            1
//#define EXTI_PORT           'B'
//#define EXTI_IRQ            1
//  
//#define CARRIER_SENS_THRESHOLD  150 // ~ -50dBm
//#define RETRANSMISSION_ATTEMPTS 10
//// settings used on operating mode register, wakeup timer always enabled
//#define IDLE_STATUS             0x21  // enable crystal (600µA or 5µA doesn't make a difference when you're sporting motors with 15A draw
//#define TX_STATUS               0x29  // enable TX. will start transmitting the packet and clear TX bit once finished
//#define RX_STATUS               0x25  
//
//#define REG_00_DEVICE_TYPE                         0x00
//#define REG_01_VERSION_CODE                        0x01
//#define REG_02_DEVICE_STATUS                       0x02
//#define REG_03_INTERRUPT_STATUS1                   0x03
//#define REG_04_INTERRUPT_STATUS2                   0x04
//#define REG_05_INTERRUPT_ENABLE1                   0x05
//#define REG_06_INTERRUPT_ENABLE2                   0x06
//#define REG_07_OPERATING_MODE1                     0x07
//#define REG_08_OPERATING_MODE2                     0x08
//#define REG_09_OSCILLATOR_LOAD_CAPACITANCE         0x09
//#define REG_0A_UC_OUTPUT_CLOCK                     0x0a
//#define REG_0B_GPIO_CONFIGURATION0                 0x0b
//#define REG_0C_GPIO_CONFIGURATION1                 0x0c
//#define REG_0D_GPIO_CONFIGURATION2                 0x0d
//#define REG_0E_IO_PORT_CONFIGURATION               0x0e
//#define REG_0F_ADC_CONFIGURATION                   0x0f
//#define REG_10_ADC_SENSOR_AMP_OFFSET               0x10
//#define REG_11_ADC_VALUE                           0x11
//#define REG_12_TEMPERATURE_SENSOR_CALIBRATION      0x12
//#define REG_13_TEMPERATURE_VALUE_OFFSET            0x13
//#define REG_14_WAKEUP_TIMER_PERIOD1                0x14
//#define REG_15_WAKEUP_TIMER_PERIOD2                0x15
//#define REG_16_WAKEUP_TIMER_PERIOD3                0x16
//#define REG_17_WAKEUP_TIMER_VALUE1                 0x17
//#define REG_18_WAKEUP_TIMER_VALUE2                 0x18
//#define REG_19_LDC_MODE_DURATION                   0x19
//#define REG_1A_LOW_BATTERY_DETECTOR_THRESHOLD      0x1a
//#define REG_1B_BATTERY_VOLTAGE_LEVEL               0x1b
//#define REG_1C_IF_FILTER_BANDWIDTH                 0x1c
//#define REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE         0x1d
//#define REG_1E_AFC_TIMING_CONTROL                  0x1e
//#define REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE   0x1f
//#define REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE    0x20
//#define REG_21_CLOCK_RECOVERY_OFFSET2              0x21
//#define REG_22_CLOCK_RECOVERY_OFFSET1              0x22
//#define REG_23_CLOCK_RECOVERY_OFFSET0              0x23
//#define REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1    0x24
//#define REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0    0x25
//#define REG_26_RSSI                                0x26
//#define REG_27_RSSI_THRESHOLD                      0x27
//#define REG_28_ANTENNA_DIVERSITY1                  0x28
//#define REG_29_ANTENNA_DIVERSITY2                  0x29
//#define REG_2A_AFC_LIMITER                         0x2a
//#define REG_2B_AFC_CORRECTION_READ                 0x2b
//#define REG_2C_OOK_COUNTER_VALUE_1                 0x2c
//#define REG_2D_OOK_COUNTER_VALUE_2                 0x2d
//#define REG_2E_SLICER_PEAK_HOLD                    0x2e
//#define REG_30_DATA_ACCESS_CONTROL                 0x30
//#define REG_31_EZMAC_STATUS                        0x31
//#define REG_32_HEADER_CONTROL1                     0x32
//#define REG_33_HEADER_CONTROL2                     0x33
//#define REG_34_PREAMBLE_LENGTH                     0x34
//#define REG_35_PREAMBLE_DETECTION_CONTROL1         0x35
//#define REG_36_SYNC_WORD3                          0x36
//#define REG_37_SYNC_WORD2                          0x37
//#define REG_38_SYNC_WORD1                          0x38
//#define REG_39_SYNC_WORD0                          0x39
//#define REG_3A_TRANSMIT_HEADER3                    0x3a
//#define REG_3B_TRANSMIT_HEADER2                    0x3b
//#define REG_3C_TRANSMIT_HEADER1                    0x3c
//#define REG_3D_TRANSMIT_HEADER0                    0x3d
//#define REG_3E_PACKET_LENGTH                       0x3e
//#define REG_3F_CHECK_HEADER3                       0x3f
//#define REG_40_CHECK_HEADER2                       0x40
//#define REG_41_CHECK_HEADER1                       0x41
//#define REG_42_CHECK_HEADER0                       0x42
//#define REG_43_HEADER_ENABLE3                      0x43
//#define REG_44_HEADER_ENABLE2                      0x44
//#define REG_45_HEADER_ENABLE1                      0x45
//#define REG_46_HEADER_ENABLE0                      0x46
//#define REG_47_RECEIVED_HEADER3                    0x47
//#define REG_48_RECEIVED_HEADER2                    0x48
//#define REG_49_RECEIVED_HEADER1                    0x49
//#define REG_4A_RECEIVED_HEADER0                    0x4a
//#define REG_4B_RECEIVED_PACKET_LENGTH              0x4b
//#define REG_50_ANALOG_TEST_BUS_SELECT              0x50
//#define REG_51_DIGITAL_TEST_BUS_SELECT             0x51
//#define REG_52_TX_RAMP_CONTROL                     0x52
//#define REG_53_PLL_TUNE_TIME                       0x53
//#define REG_55_CALIBRATION_CONTROL                 0x55
//#define REG_56_MODEM_TEST                          0x56
//#define REG_57_CHARGE_PUMP_TEST                    0x57
//#define REG_58_CHARGE_PUMP_CURRENT_TRIMMING        0x58
//#define REG_59_DIVIDER_CURRENT_TRIMMING            0x59
//#define REG_5A_VCO_CURRENT_TRIMMING                0x5a
//#define REG_5B_VCO_CALIBRATION                     0x5b
//#define REG_5C_SYNTHESIZER_TEST                    0x5c
//#define REG_5D_BLOCK_ENABLE_OVERRIDE1              0x5d
//#define REG_5E_BLOCK_ENABLE_OVERRIDE2              0x5e
//#define REG_5F_BLOCK_ENABLE_OVERRIDE3              0x5f
//#define REG_60_CHANNEL_FILTER_COEFFICIENT_ADDRESS  0x60
//#define REG_61_CHANNEL_FILTER_COEFFICIENT_VALUE    0x61
//#define REG_62_CRYSTAL_OSCILLATOR_POR_CONTROL      0x62
//#define REG_63_RC_OSCILLATOR_COARSE_CALIBRATION    0x63
//#define REG_64_RC_OSCILLATOR_FINE_CALIBRATION      0x64
//#define REG_65_LDO_CONTROL_OVERRIDE                0x65
//#define REG_66_LDO_LEVEL_SETTINGS                  0x66
//#define REG_67_DELTA_SIGMA_ADC_TUNING1             0x67
//#define REG_68_DELTA_SIGMA_ADC_TUNING2             0x68
//#define REG_69_AGC_OVERRIDE1                       0x69
//#define REG_6A_AGC_OVERRIDE2                       0x6a
//#define REG_6B_GFSK_FIR_FILTER_COEFFICIENT_ADDRESS 0x6b
//#define REG_6C_GFSK_FIR_FILTER_COEFFICIENT_VALUE   0x6c
//#define REG_6D_TX_POWER                            0x6d
//#define REG_6E_TX_DATA_RATE1                       0x6e
//#define REG_6F_TX_DATA_RATE0                       0x6f
//#define REG_70_MODULATION_CONTROL1                 0x70
//#define REG_71_MODULATION_CONTROL2                 0x71
//#define REG_72_FREQUENCY_DEVIATION                 0x72
//#define REG_73_FREQUENCY_OFFSET1                   0x73
//#define REG_74_FREQUENCY_OFFSET2                   0x74
//#define REG_75_FREQUENCY_BAND_SELECT               0x75
//#define REG_76_NOMINAL_CARRIER_FREQUENCY1          0x76
//#define REG_77_NOMINAL_CARRIER_FREQUENCY0          0x77
//#define REG_79_FREQUENCY_HOPPING_CHANNEL_SELECT    0x79
//#define REG_7A_FREQUENCY_HOPPING_STEP_SIZE         0x7a
//#define REG_7C_TX_FIFO_CONTROL1                    0x7c
//#define REG_7D_TX_FIFO_CONTROL2                    0x7d
//#define REG_7E_RX_FIFO_CONTROL                     0x7e
//#define REG_7F_FIFO_ACCESS                         0x7f
//
//RF22B::RF22B()
//  : ssp_(SSP_PORT_NUM, SSP_CONFIG_VAL, CHIP_SELECT_PORT, CHIP_SELECT_PIN),
//    wakeupfromtx_(0),
//    txbusy_(false)
//{
//}
//
//void RF22B::Socket::push_back(RF22B::Packet *pPacket)
//{
//  sys::Atomic now;
//  buffer_.push_back(pPacket);
//  if(full())
//    event_.Pulse(3);
//  else
//    event_.Pulse(1);
//}
//RF22B::Packet* RF22B::Socket::pop_front()
//{
//  sys::Atomic now;
//  Packet *pPacket(buffer_.front());
//  buffer_.pop_front();
//  return pPacket;
//}
//
//void RF22B::Init(const char *DeviceName)
//{
//  SetupPin<EXTI_PORT, EXTI_PIN, INPUT_PULLUP>();
//
//  ssp_.Configure();
//  events_.Init();
//
//  // delay 20mSec for the POR
//  sys::msleep(20);
//  // SW reset the device
//  WriteRegister(REG_07_OPERATING_MODE1, 0x80 );
//  sys::msleep(20);
//  // This assumes GPIO0(out) is connected to TX_ANT(in) to enable tx antenna during transmit
//  WriteRegister(REG_0B_GPIO_CONFIGURATION0,               0x12); // TX state
//  // This assumes GPIO1(out) is connected to RX_ANT(in) to enable rx antenna during receive
//  WriteRegister(REG_0C_GPIO_CONFIGURATION1,               0x15); // RX state
//  // set the remaining GPIO to input with pull up, just so it is in a defined state
//  WriteRegister(REG_0D_GPIO_CONFIGURATION2,               0x23);
//  // set the wake up timer exponent to 3 to set the wake up timer frequency to about 1mSec
//  WriteRegister(REG_14_WAKEUP_TIMER_PERIOD1,              0x03);
//  SetWakeupTimer(0xFFFF);
//  // copied from SILabs spreadsheet
//  WriteRegister(REG_1C_IF_FILTER_BANDWIDTH,               0x83);
//  WriteRegister(REG_1D_AFC_LOOP_GEARSHIFT_OVERRIDE,       0x40); // enable afc
//  WriteRegister(REG_1F_CLOCK_RECOVERY_GEARSHIFT_OVERRIDE, 0x03); // bcr loop before preamble crgain / 2^0. after crgain / 2^3
//  WriteRegister(REG_20_CLOCK_RECOVERY_OVERSAMPLING_RATE,  0x5E); // rx oversampling
//  WriteRegister(REG_21_CLOCK_RECOVERY_OFFSET2,            0x01);
//  WriteRegister(REG_22_CLOCK_RECOVERY_OFFSET1,            0x5D);
//  WriteRegister(REG_23_CLOCK_RECOVERY_OFFSET0,            0x86);
//  WriteRegister(REG_24_CLOCK_RECOVERY_TIMING_LOOP_GAIN1,  0x02);
//  WriteRegister(REG_25_CLOCK_RECOVERY_TIMING_LOOP_GAIN0,  0xB0);
//  // set rssi channel clear indicator threshold. if the irq is enabled it will be asserted when the strength is above the value
//  WriteRegister(REG_27_RSSI_THRESHOLD,  CARRIER_SENS_THRESHOLD);
//  // enable Packet Handler RX and TX, CRC CCITT check over everything
//  WriteRegister(REG_30_DATA_ACCESS_CONTROL,               0x8C);
//  // set broadcast and header check enable bits for header 3 aka address of this device
//  WriteRegister(REG_32_HEADER_CONTROL1,                   0x88);
//  // set header length 4 & sync length 2. no fixed packet length
//  WriteRegister(REG_33_HEADER_CONTROL2,                   0x42);
//  // send 8 nibbles for detection
//  WriteRegister(REG_34_PREAMBLE_LENGTH,                   0x08);
//  // process 6 nibbles during detection and +8dB in the RSSI calculation
//  WriteRegister(REG_35_PREAMBLE_DETECTION_CONTROL1,       0x2A);
//  // set the sync word to 2D B4 to avoid collision with most RFM applications
//  WriteRegister(REG_36_SYNC_WORD3,                        0x2D);
//  WriteRegister(REG_37_SYNC_WORD2,                        0xB4);
//  // put own device address into check header 0 and transmit header 1
//  ChangeDeviceName(DeviceName);
//  // TX power maximum
//  WriteRegister(REG_6D_TX_POWER,                          0x1F);
//  // copied from SILabs spreadsheet
//  WriteRegister(REG_6E_TX_DATA_RATE1,                     0x20); // datarate 38,4kbps
//  WriteRegister(REG_6F_TX_DATA_RATE0,                     0xC5);
//  WriteRegister(REG_70_MODULATION_CONTROL1,               0x01); // enable data whitening
//  WriteRegister(REG_71_MODULATION_CONTROL2,               0x2B); // FIFO mode, enable inverted, GFSK
//  WriteRegister(REG_72_FREQUENCY_DEVIATION,               0x68); // deviation 50KHz, in one of the sample programs they say when setting 50kHz it's actually 65kHz or something. but so be it, polute the ether >:D
//  WriteRegister(REG_75_FREQUENCY_BAND_SELECT,             0x53); // 433 MHz carrier frequency
//  WriteRegister(REG_76_NOMINAL_CARRIER_FREQUENCY1,        0x64);
//  WriteRegister(REG_77_NOMINAL_CARRIER_FREQUENCY0,        0x00);
//  // enable interrupts
//  WriteRegister(REG_05_INTERRUPT_ENABLE1,                 0x07); // enable irq pksent, pkvalid, crc-error
//  WriteRegister(REG_06_INTERRUPT_ENABLE2,                 0x08); // enable wake up irq
//  // enter default idle mode
//  WriteRegister(REG_07_OPERATING_MODE1,            IDLE_STATUS);
//  
//  // Read IRQ Registers to clear any pending
//#ifndef NDEBUG
//  printf("RF IRQ Stat:%X %X\n\r", ReadRegister(REG_03_INTERRUPT_STATUS1), ReadRegister(REG_04_INTERRUPT_STATUS2));
//#else
//  ReadRegister(REG_03_INTERRUPT_STATUS1);
//  ReadRegister(REG_04_INTERRUPT_STATUS2);
//#endif
//  // enable external interrupt
//  Exti::assign(EXTI_IRQ, EDGE_LOW, rf22b_isr);
//}
//
//void RF22B::Write(RF22B::Packet *pToWrite)
//{
//  WriteRegister(REG_3A_TRANSMIT_HEADER3, pToWrite->address());
//  WriteRegister(REG_3C_TRANSMIT_HEADER1, pToWrite->identifier());
//  WriteRegister(REG_3D_TRANSMIT_HEADER0, pToWrite->flags());
//  WriteRegister(REG_3E_PACKET_LENGTH, pToWrite->length());
//  *(pToWrite->datapointer()) = REG_7F_FIFO_ACCESS;
//  //WriteRegister(REG_08_OPERATING_MODE2, 0x01);  // clear TX FIFO
//  //WriteRegister(REG_08_OPERATING_MODE2, 0x00);
//  WriteMultiple(pToWrite->datapointer(), pToWrite->length()+1);
//}
//void RF22B::Read(RF22B::Packet *pToStore)
//{
//  pToStore->setaddress(   ReadRegister(REG_48_RECEIVED_HEADER2));
//  pToStore->setidentifier(ReadRegister(REG_49_RECEIVED_HEADER1));
//  pToStore->setflags(     ReadRegister(REG_4A_RECEIVED_HEADER0));
//  pToStore->setlength(    ReadRegister(REG_4B_RECEIVED_PACKET_LENGTH));
//  *(pToStore->datapointer()) = REG_7F_FIFO_ACCESS;
//  ReadMultiple(pToStore->datapointer(), pToStore->length()+1);
//  *(pToStore->datapointer()) = REG_7F_FIFO_ACCESS;  // ssp overwrites first byte, effectively freeing the packet
//}
//
//void RF22B::ChangeDeviceName(const char *NewName)
//{
//  devicename_ = NewName;
//  WriteRegister(REG_3F_CHECK_HEADER3,    devicename_[0]);
//  WriteRegister(REG_3B_TRANSMIT_HEADER2, devicename_[0]);
//}
//
//void RF22B::SetWakeupTimer(uint16_t mSec)
//{
//  WriteRegister(REG_15_WAKEUP_TIMER_PERIOD2, (mSec >> 8));
//  WriteRegister(REG_16_WAKEUP_TIMER_PERIOD3, (mSec & 0xFF));
//}
//
//uint16_t RF22B::GetWakeupTimer()
//{
//  return uint16_t((ReadRegister(REG_17_WAKEUP_TIMER_VALUE1) << 8) | ReadRegister(REG_18_WAKEUP_TIMER_VALUE2));
//}
//
//void RF22B::PushPacket(RF22B::Packet *pPacket)
//{
//  sys::disable_irqs();
//  while(packetbuffer_.full())
//  {
//    sys::enable_irqs();
//    sys::msleep(5);
//    sys::disable_irqs();
//  }
//  packetbuffer_.push_back(pPacket);
//  if(!txbusy_)
//    events_.Set(RF22B_EVENT_TXFFNE);
//  sys::enable_irqs();
//}
//
//void RF22B::thread(void)
//{
//  unsigned event_status;
//  uint8_t irq_status_1, irq_status_2;
//  
//  WriteRegister(REG_07_OPERATING_MODE1, RX_STATUS);
//  // start the periodical timer
//  SetWakeupTimer(0x8000);
//  
//  while(true)
//  {
//    event_status = events_.WaitOn(RF22B_EVENT_ANY);
//    
//    if(event_status & RF22B_EVENT_IRQ)
//    {
//      events_.Clear(RF22B_EVENT_IRQ);
//      irq_status_1 = ReadRegister(REG_03_INTERRUPT_STATUS1);
//      irq_status_2 = ReadRegister(REG_04_INTERRUPT_STATUS2);
//      
//      if(irq_status_1 & 0x80)
//      { // FIFO error, reset and keep going. might need to check if this will cause issues, destroying packets and the like
//        // Had a lot of FIFO errors when there was a bug on the packet length calculation, should not really happen anymore
//        WriteRegister(REG_08_OPERATING_MODE2, 0x03);
//        WriteRegister(REG_08_OPERATING_MODE2, 0);
//        if(irq_status_1 & 0x02)
//        { // FIFO ERROR and valid packet don't really go along all to well, clear packet status
//          WriteRegister(REG_07_OPERATING_MODE1, RX_STATUS);
//          irq_status_1 &= 0xFD;
//        }
//      }
//      
//      if(irq_status_1 & 0x01)
//      { // CRC Error detected
//        errorcs_.inc_crc();
//      }
//      else if(irq_status_1 & 0x02)
//      { // valid packet received
//        Receive();
//        WriteRegister(REG_07_OPERATING_MODE1, RX_STATUS);
//        packetcs_.inc_recv();
//      }
//      
//      if(irq_status_1 & 0x04)
//      { // successfully transmitted a packet, device should be in normal operating mode
//        sys::disable_irqs();
//        packetbuffer_.front()->free();
//        packetbuffer_.pop_front();
//        if(!packetbuffer_.empty())
//        { // if the FIFO ain't empty, schedule another transmission
//          event_status |= RF22B_EVENT_TXFFNE;
//        }
//        else
//        {
//          txbusy_ = false;
//        }
//        sys::enable_irqs();
//        WriteRegister(REG_07_OPERATING_MODE1, RX_STATUS);
//        packetcs_.inc_send();
//      }
//
//      if(irq_status_2 & 0x08)
//      { // wakeup event
//        if(wakeupfromtx_ != 0)
//        { // TX FIFO isn't empty and the attempt to transmit was aborted due to blocked ether
//          // timeout is over now retry to send the packet by setting the event bit and going on
//          event_status = RF22B_EVENT_TXFFNE;
//        }
//        else
//        { // re-enable wakeup every 30 seconds and do whatever when routing is enabled
//          SetWakeupTimer(0x8000);
//        }
//      }
//    } // if(event_status & RF22B_EVENT_IRQ)
//    if(event_status & RF22B_EVENT_TXFFNE)
//    {
//      events_.Clear(RF22B_EVENT_TXFFNE);
//      Transmit();
//    }
//  } // while(true)
//}
//
//void RF22B::watchdog()
//{
//  PinDigital irq_pin(EXTI_PORT, EXTI_PIN);
//  if(!irq_pin)
//  {
//    events_.Set(RF22B_EVENT_IRQ);
//  }
//}
//
//void RF22B::Receive()
//{
//  Packet *pPacket = allocate();
//  Read(pPacket);
//  
//  if(pPacket->identifier() == 0xFF)
//  { // received packet is not meant for users
//    // initiate whatever communication between devices is needed here
//    pPacket->free();
//    return;
//  }
//  
//  Socket *pSocket = socketlist_.find(pPacket->identifier());
//    
//  if(pSocket == 0)
//  { // No Socket fitting this Packet, just discard it
//    pPacket->free();
//    return;
//  }
//  // else deliver it, user threads will handle the rest from here on
//  pSocket->push_back(pPacket);
//}
//
//void RF22B::Transmit()
//{
//  // reading out the RSSI value. No use trying to send a packet when this device might currently even be the Receiver
//  if(ReadRegister(REG_26_RSSI) > CARRIER_SENS_THRESHOLD)
//  { // make sure the wakeup condition will always be met when the timer is restored
//    if(wakeupfromtx_ == 0)
//      wakeupfromtx_ = GetWakeupTimer() + 10;
//    // transmission of a complete packet takes about 4ms at 128kbps
//    SetWakeupTimer(3);
//    errorcs_.inc_csens();
//    return;
//  }
//  Write(packetbuffer_.front());
//  // enable transmitter to transmit the packet
//  txbusy_ = true;
//  WriteRegister(REG_07_OPERATING_MODE1, TX_STATUS);
//  if(wakeupfromtx_ != 0)
//  {
//    SetWakeupTimer(wakeupfromtx_);
//    wakeupfromtx_ = 0;
//  }
//}
//
//void rf22b_isr(void)
//{
//  rf22b.isr();
//}
//
//void rf22b_thread(void *DeviceName)
//{
//  rf22b.Init((const char*)DeviceName);
//  rf22b.thread();
//}