#ifndef RF22B_STM8_H
#define RF22B_STM8_H
#include "stm8s_hal.h"

// and the interrupt out of the chip is represented on the miso line when chip select is high
//#define RF22B_USE_SDO_AS_NIRQ
// GPIO2 is set to regular output
#define RF22B_GPIO2_AS_OUTPUT
// GPIO2 display the channel clear value
//#define RF22B_GPIO2_AS_CHANNEL_CLEAR
// else GPIO2 as input with pullup

template<typename ChipSelect_t>
class Rf22b {
	// miso used as nIRQ when spi is not in use
	typedef Pin<GPIOC_BaseAddress, 7> IrqPin_t;
public:
	// initialize the Device and start receiving packets with address 
	static void init(uint8_t ownaddress) {
		SpiNC::init(SpiNC::MHz8);
		ChipSelect_t csel;
		csel.mode(true, true);
		csel.set();
		sState = 0;
		sTxCount = 0;
		sRxCount = 0;
		// wait a little for the POR to finish
		System::delay(20);
		// software reset the device
		write(0x07, 0x80);
		System::delay(1);
		// This assumes GPIO0(out) is connected to TX_ANT(in) to enable tx antenna during transmit
		// GPIO1(out) is connected to RX_ANT(in) to enable rx antenna during receive
#if defined RF22B_GPIO2_AS_OUTPUT
#if defined RF22B_USE_SDO_AS_NIRQ
		const uint8_t gpiocfg[5] = { 0x8B, 0x12, 0x15, 0xCA, 0x08 };
#else
		const uint8_t gpiocfg[5] = { 0x8B, 0x12, 0x15, 0xCA, 0x00 };
#endif
#elif defined RF22B_GPIO2_AS_CHANNEL_CLEAR
#ifdef RF22B_USE_SDO_AS_NIRQ
		const uint8_t gpiocfg[5] = { 0x8B, 0x12, 0x15, 0xDC, 0x08 };
#else
		const uint8_t gpiocfg[5] = { 0x8B, 0x12, 0x15, 0xDC, 0x00 };
#endif
#else
#ifdef RF22B_USE_SDO_AS_NIRQ
		const uint8_t gpiocfg[5] = { 0x8B, 0x12, 0x15, 0x23, 0x08 };
#else
		const uint8_t gpiocfg[5] = { 0x8B, 0x12, 0x15, 0x23, 0x00 };
#endif
#endif
		write(gpiocfg, 5);
		// set the wakeup timer to a period of 1ms per lsb and set the highest possible value
		// have the wakuep occure every 8ms, Transmitting a complete message takes about 5.5ms with ACK
		const uint8_t wkupcfg[3] = { 0x95, 0x00, 0x08 };
		write(wkupcfg, 3);
		// cfg value from datasheet gfsk, 128kbs, manchester and packethandler with crc
		const uint8_t afccfg[13] = { 0x9C, 0x8C, 0x40, 0x0A, 0x03, 0x2F, 0x01, 0x5D, 0x86, 0x05, 0x74, 0x00, 0x64 };
		write(afccfg, 13);
		const uint8_t pckcfg[9] = { 0xB0, 0x8C, 0x00, 0x08, 0x22, 0x05, 0x2A, 0x2D, 0xB4 };
		write(pckcfg, 9);
		const uint8_t rtxcfg[12] = { 0xED, 0x1F, 0x20, 0xC5, 0x0E, 0x2B, 0xCD, 0x00, 0x00, 0x53, 0x4B, 0x00 };
		write(rtxcfg, 12);
		// set own address in transmit header 2 and receive check 3
		write(0x3B, ownaddress);
		write(0x3F, ownaddress);
		// enable packet send, packet received, wakeup and sync word interrupt
		const uint8_t irqcfg[3] = { 0x85, 0x06, 0x88 };
		write(irqcfg, 3);
		// read irq status registers to clear them irq pending
		uint8_t irqstate[2];
		read(REGI_IRQS, irqstate, 2);
		// enter receiver mode with wake up timer enabled
		rxon();
#ifdef RF22B_USE_SDO_AS_NIRQ
		// enable the external interrupt on miso
		IrqPin_t irq;
		System::enableExti(irq, System::Exti_level_low);
		irq.mode(false, false);
		irq.exti(true);
#endif
	}

	// check if the last ordered transmition has been succesfully sent
	// returns 0 if the packet was succesfully send, else returns a number
	// returns 0xFF if the packet was send but didn't receive an ACK
	static int8_t checkTx() {
		return sTxCount;
	}

	// check if a packet is available for reading
	// returns 0 if no packet is available, else returns the data byte count
	static uint8_t in_avail() {
		return sRxCount;
	}

	// transmit a package to the given address
	static void send(uint8_t address, const uint8_t* data, uint8_t count) {
		sTxCount = count;
		sAddress = address;
#ifdef RF22B_USE_SDO_AS_NIRQ
		// disable exti on the irq pin before using spi
		IrqPin_t irq;
		irq.exti(false);
#endif
		write(REGI_FIFO, data, count);
		// set the TXNE bit 
		sState |= STATE_TXNE;
#ifdef RF22B_USE_SDO_AS_NIRQ
		// turn on exti again
		irq.exti(true);
#endif
	}

	// retransmit the last message
	static void resend(uint8_t count) {
		sTxCount = count;
		sState |= STATE_TXNE;
	}

	// receive the package with count bytes from the device. Returns source address
	static uint8_t recv(uint8_t* data, uint8_t count) {
#ifdef RF22B_USE_SDO_AS_NIRQ
		// disable exti on the irq pin before using spi
		IrqPin_t irq;
		irq.exti(false);
#endif
		read(REGI_FIFO, data, count);
		uint8_t src = read(REGI_SRC);
		// if we are currently not transmitting, turn on receiver again since rx fifo is empty now
		if (!(sState & STATE_BUSY))
			rxon();
#ifdef RF22B_USE_SDO_AS_NIRQ
		// enable exti again
		irq.exti(true);
#endif
		return src;

	}

	// interrupt that is controlling device operation
	static void isr() {
#ifdef RF22B_USE_SDO_AS_NIRQ
		// disable exti on the irq pin before using spi
		IrqPin_t irq;
		irq.exti(false);
#endif
		uint8_t irqstate[2];
		read(REGI_IRQS, irqstate, 2);
		if (irqstate[0] & 0x02) {
			// received a valid packet, check if we were waiting for ACK
			if (sState & STATE_WAIT_ACK) {
				// waiting for ACK, check if the received packet size is 0
				uint8_t count = read(REGI_RLEN);
				if (count == 0) {
					// the message was an ACK, clear TxCount since transaction was succesfull
					sState &= ~STATE_WAIT_ACK;
					sTxCount = 0;
					// if the receiver is allready empty, turn it on
					if (sRxCount == 0) {
						rxon();
					}
				}
				else {
					// instead of ACK we received another message, set TxCount to NACK and RxCount to count and answer with ack
					sState &= ~STATE_WAIT_ACK;
					sTxCount = 0xFF;
					sRxCount = count;
					sendAck();
				}
			}
			else {
				// not waiting for ACK, so we received a message. Answer with ACK
				sRxCount = read(REGI_RLEN);
				sendAck();
			}
		}
		else if (irqstate[0] & 0x04) {
			sState &= ~STATE_BUSY;
			// packet was transmitted, check if we had send a message or an ACK
			if (sState & STATE_ACK_SEND) {
				// ACK was transmitted, clear the state and stay in idle since rx fifo is filled
				sState &= ~STATE_ACK_SEND;
				// if the receiver is allready cleared turn it on again
				if (sRxCount == 0) {
					rxon();
				}
			}
			else {
				// set wait for ACK state and turn on receiver
				sState |= STATE_WAIT_ACK;
				rxon();
			}
		}
		else if (irqstate[1] & 0x80) {
			// valid sync word detected, reset the wakeup timer to sync with other device
			write(REGI_MODE, MODE_RXON & ~0x20);
			write(REGI_MODE, MODE_RXON);
		}

		if (irqstate[1] & 0x08) {
			// wake up irq
			if (sState & STATE_WAIT_ACK) {
				// if waiting on ACK but timeout came, ACK won't come -> NACK last message
				sState &= ~STATE_WAIT_ACK;
				sTxCount = 0xFF;
			}
			if (sState & STATE_TXNE) {
				// if TX isn't empty, check if the channel is open and than transmit the message
				if (!(irqstate[1] & 0x10)) {
					write(REGI_DEST, sAddress);
					write(REGI_TLEN, sTxCount);
					txon();
					sState |= STATE_BUSY;
					sState &= ~STATE_TXNE;
				}
			}
		}
#ifdef RF22B_USE_SDO_AS_NIRQ
		// enable exti again
		irq.exti(true);
#endif
	}

	// read the current wake up timer value
	static uint16_t readWakeup() {
		uint8_t cmd[3] = { 0x17 };
#ifdef RF22B_USE_SDO_AS_NIRQ
		// disable exti on the irq pin before using spi
		IrqPin_t irq;
		irq.exti(false);
#endif
		ChipSelect_t csel;
		csel.clear();
		SpiNC::transceive(cmd, 3);
		csel.set();
#ifdef RF22B_USE_SDO_AS_NIRQ
		// enable exti again
		irq.exti(true);
#endif
		return uint16_t(cmd[1] << 8) | cmd[2];
	}

	// read the current RSSI value
	static uint8_t readRssi() {
		uint8_t cmd[2] = { 0x26 };
#ifdef RF22B_USE_SDO_AS_NIRQ
		// disable exti on the irq pin before using spi
		IrqPin_t irq;
		irq.exti(false);
#endif
		ChipSelect_t csel;
		csel.clear();
		SpiNC::transceive(cmd, 2);
		csel.set();
		// enable exti again
		irq.exti(true);
		return cmd[1];
	}

	// if GPIO2 is configured as output, drive it high
	static void setGPIO2() {
#ifdef RF22B_USE_SDO_AS_NIRQ
		IrqPin_t irq;
		irq.exti(false);
		write(REGI_GPIOS, 0x0C);
		irq.exti(true);
#else
		write(REGI_GPIOS, 0x04);
#endif
	}

	// if GPIO2 is configuered as output, drive it low
	static void clearGPIO2() {
#ifdef RF22B_USE_SDO_AS_NIRQ
		IrqPin_t irq;
		irq.exti(false);
		write(REGI_GPIOS, 0x08);
		irq.exti(true);
#else
		write(REGI_GPIOS, 0x00);
#endif
	}
private:
	static void sendAck() {
		// answer the last received message with an empty message to ACK it
		write(REGI_DEST, read(REGI_SRC));
		write(REGI_TLEN, 0);
		txon();
		sState |= STATE_ACK_SEND | STATE_BUSY;
	}

	static void wakeup(uint8_t msb_millis, uint8_t lsb_millis) {
		ChipSelect_t csel;
		csel.clear();
		uint8_t cmd[3] = { 0x15 | 0x80, msb_millis, lsb_millis };
		SpiNC::write(cmd, 3);
		csel.set();
	}

	static void rxon() {
		write(REGI_MODE, MODE_RXON);
	}

	static void txon() {
		write(REGI_MODE, MODE_TXON);
	}

	static void idle() {
		write(REGI_MODE, MODE_IDLE);
	}

	static void write(uint8_t regi, uint8_t data) {
		ChipSelect_t csel;
		uint8_t cmd[2] = { regi | 0x80, data };
		csel.clear();
		SpiNC::write(cmd, 2);
		csel.set();
	}

	static void write(const uint8_t* data, uint8_t count) {
		ChipSelect_t csel;
		csel.clear();
		SpiNC::write(data, count);
		csel.set();
	}

	static void write(uint8_t regi, const uint8_t *data, uint8_t count) {
		ChipSelect_t csel;
		csel.clear();
		regi |= 0x80; // set the write bit of register address
		SpiNC::write(&regi);
		SpiNC::write(data, count);
		csel.set();
	}

	static uint8_t read(uint8_t regi) {
		ChipSelect_t csel;
		csel.clear();
		uint8_t cmd[2] = { regi };
		SpiNC::transceive21(cmd);
		csel.set();
		return cmd[1];
	}

	static void read(uint8_t regi, uint8_t *data, uint8_t count) {
		ChipSelect_t csel;
		csel.clear();
		SpiNC::write(&regi);
		SpiNC::read(data, count);
		csel.set();
	}

	static const int STATE_WAIT_ACK = 0x01;
	static const int STATE_ACK_SEND = 0x02;
	static const int STATE_TXNE = 0x04;
	static const int STATE_BUSY = 0x08;
	static const int STATE_SYNC_INC = 0x10;
	static const int STATE_SYNC_CMP = 0xE0;
	static const int MODE_IDLE = 0x23;
	static const int MODE_TXON = 0x2B;
	static const int MODE_RXON = 0x27;
	static const int REGI_IRQS = 0x03;
	static const int REGI_MODE = 0x07;
	static const int REGI_GPIOS = 0x0E;
	static const int REGI_FIFO = 0x7F;
	static const int REGI_DEST = 0x3A;
	static const int REGI_SRC = 0x48;
	static const int REGI_TLEN = 0x3E;
	static const int REGI_RLEN = 0x4B;

	static volatile uint8_t sState;
	static volatile uint8_t sAddress;
	static volatile uint8_t sTxCount;
	static volatile uint8_t sRxCount;
};

template<typename ChipSelect_t> volatile uint8_t Rf22b<ChipSelect_t>::sState;
template<typename ChipSelect_t> volatile uint8_t Rf22b<ChipSelect_t>::sAddress;
template<typename ChipSelect_t> volatile uint8_t Rf22b<ChipSelect_t>::sTxCount;
template<typename ChipSelect_t> volatile uint8_t Rf22b<ChipSelect_t>::sRxCount;

//
//class RF22B
//{
//public:
//  class Packet;
//  class Socket;
//
//  RF22B();
//  ~RF22B() {}
//    
//  void Init(const char *DeviceName);
//  
//  void AddSocket(Socket *pSocket);
//  void PushPacket(Packet *pPacket);
//  
//  Packet* allocate()
//  {
//    sys::Atomic now;
//    Packet* pFound = packetheap_.find_free();
//    if(pFound != 0)
//    {
//      pFound->allocate();
//      return pFound;
//    }
//    return 0;
//  }
//  
//  void ChangeDeviceName(const char *NewName);
//  
//  // bound to an exti in the .cpp allready
//  void isr() { events_.Set(RF22B_EVENT_IRQ); }
//  // used in the rf22b_thread function declared after the class declaration
//  void thread();
//  // make sure we don't miss an interrupt
//  void watchdog();
//  
//  class Packet
//  {
//  public:
//    typedef mtl::StaticArray<uint8_t, 65>::iterator iterator;
//    typedef mtl::StaticArray<uint8_t, 65>::const_iterator const_iterator;
//  
//    Packet() { free(); }
//    ~Packet() {}
//  
//    iterator       end()       { return data_.end(); }
//    const_iterator end() const { return data_.end(); }
//    iterator       back()       { return begin()+length_; }
//    const_iterator back() const { return begin()+length_; }
//    iterator       begin()       { return ++(data_.begin()); }
//    const_iterator begin() const { return ++(data_.begin()); }
//    
//    template<typename T>
//    iterator insert(iterator it, const T *pMem)
//    {
//      for(const uint8_t *s((const uint8_t*)pMem), *e(s+sizeof(T)); s != e; ++s, ++it)
//        *it = *s;
//      length_ += sizeof(T);
//      return it;
//    }
//    template<typename InIter>
//    iterator insert(iterator it, size_t num, InIter in)
//    {
//      for(iterator e(it+num); it != e; ++in, ++it)
//        *it = *in;
//      length_ += num; 
//      return it;
//    }
//    template<typename T>
//    const_iterator extract(const_iterator it, T *pMem) const
//    {
//      for(uint8_t *s((uint8_t*)pMem),*e(s+sizeof(T)); s != e; ++s, ++it)
//        *s = *it;
//      return it;
//    }
//    template<typename OutIter>
//    const_iterator extract(const_iterator it, size_t num, OutIter out) const
//    {
//      for(const_iterator e(it+num); it != e; ++it, ++out)
//        *out = *it;
//      return it;
//    }
//    
//    void clear() { length_ = 0; }
//    size_t free_bytes() const { return data_.size()-length_-1; }
//        
//    bool is_free() const { return datapointer()[0] == 0x00; }
//    void free() { datapointer()[0] = 0x00; }
//    void allocate() { datapointer()[0] = 0x01; clear(); } 
//    
//    uint8_t flags() const { return flags_; }
//    uint8_t length() const { return length_; }
//    uint8_t address() const { return address_; }
//    uint8_t identifier() const { return identifier_; }
//    uint8_t* datapointer() { return data_.make_pointer(); }
//    const uint8_t* datapointer() const { return data_.make_pointer(); }
//    void setflags(uint8_t val) { flags_ = val; }
//    void setlength(uint8_t val) { length_ = val; }
//    void setaddress(uint8_t val) { address_ = val; }
//    void setidentifier(uint8_t val) { identifier_ = val; }
//  private:
//    uint8_t flags_;
//    uint8_t length_;
//    uint8_t address_;
//    uint8_t identifier_;
//    mtl::StaticArray<uint8_t, 65> data_;
//  }; // END class Packet
//  class Socket
//  {
//  public:
//    Socket(uint8_t ID) : identifier_(ID) { event_.Init(); }
//    ~Socket() {}
//      
//    size_t size() const { return buffer_.size(); }
//    bool empty() const { return buffer_.empty(); }
//    bool full() const { return buffer_.full(); }
//            
//    void push_back(Packet *pPacket);
//    Packet* pop_front();
//    
//    void WaitOnNotEmpty() { event_.WaitOn(1); }
//    void WaitOnFull() { event_.WaitOn(2); }
//        
//    bool WaitOnNotEmpty(unsigned Timeout) { return event_.WaitOn(1, Timeout) != 0; }
//    bool WaitOnFull(unsigned Timeout) { return event_.WaitOn(2, Timeout) != 0; }
//    
//    bool operator==(uint8_t id) const { return identifier_ == id; }
//  private:
//    mtl::StaticList<Socket>::Element listelem_;
//    mtl::StaticBuffer<Packet*, 5> buffer_;
//    sys::Eventset event_;
//    uint8_t identifier_;
//  }; // END class Socket
//  class ErrorCounters
//  {
//  public:
//    ErrorCounters() : crc_(0), csens_(0) {}
//    ~ErrorCounters() {}
//    
//    void inc_crc() { ++crc_; }
//    void inc_csens() { ++csens_; }
//    
//    unsigned crc() const { return crc_; }
//    unsigned csens() const { return csens_; }
//  private:
//    unsigned crc_, csens_;
//  };
//  class PacketCounters
//  {
//  public:
//    PacketCounters() : send_(0), recv_(0), cng_send_(true), cng_recv_(true) {}
//    ~PacketCounters() {}
//      
//    void inc_send() { ++send_; cng_send_ = true; }
//    void inc_recv() { ++recv_; cng_recv_ = true; }
//    
//    bool changed_send() { return cng_send_; }
//    bool changed_recv() { return cng_recv_; }
//    
//    unsigned send() { cng_send_ = false; return send_; }
//    unsigned recv() { cng_recv_ = false; return recv_; }
//  private:
//    unsigned send_, recv_;
//    bool cng_send_, cng_recv_; 
//  };
//  const ErrorCounters& errorcs() const { return errorcs_; }
//  PacketCounters& packetcs() { return packetcs_; }
//private:
//  class Packetheap
//  {
//  public:
//    typedef mtl::StaticArray<RF22B::Packet, 50>::iterator iterator;
//    
//    Packetheap() {}
//    ~Packetheap() {}
//        
//    Packet* find_free();
//  private:
//    mtl::StaticArray<RF22B::Packet, 50> heap_;
//  }; // END class Packetheap
//  class Socketlist
//  {
//  public:
//    typedef mtl::StaticList<Socket>::iterator iterator;
//  
//    Socketlist() {}
//    ~Socketlist() {}
//      
//    void Add(Socket *ToAdd) { list_.push_back(ToAdd); }
//    void Remove(Socket *ToRemove);  
//    Socket* find(uint8_t Identifier);
//  private:
//    mtl::StaticList<Socket> list_;
//  }; // END class Socketlist
//
//  // private Functions Declarations
//  void WriteRegister(uint8_t RegNum, uint8_t Value);
//  uint8_t ReadRegister(uint8_t RegNum);
//  // Requires the Data to contain the Registers address on it's first position
//  void WriteMultiple(uint8_t *pData, uint8_t count);
//  void ReadMultiple(uint8_t *pData, uint8_t count);
//  void Write(Packet *pToWrite);
//  void Read(Packet *pToStore);
//  void SetWakeupTimer(uint16_t mSec);
//  uint16_t GetWakeupTimer();
//  void Receive();
//  void Transmit();
//  void ClearFIFO() { }
//  
//  // Memeber variables
//  SSP ssp_;
//  Socketlist socketlist_;
//  mtl::StaticBuffer<Packet*, 20> packetbuffer_;
//  sys::Eventset events_;
//  ErrorCounters errorcs_;
//  PacketCounters packetcs_;
//  const char *devicename_;
//  uint16_t wakeupfromtx_;
//  bool txbusy_;
//  Packetheap packetheap_;
//};
//
//extern RF22B rf22b;
//
//void rf22b_thread(void *DeviceName);
//void rf22b_isr(void);
//
////////////////////
//// Functor classes
////////////////////
//class RF22B_fn_packet_is_free
//{
//public:
//  bool operator()(const RF22B::Packet &Ref)
//  {
//    return Ref.is_free();
//  }
//};
//////////////////////
//// Inlined Functions
//////////////////////
//inline void RF22B::AddSocket(Socket *pSocket)
//{
//  sys::Atomic now;
//  socketlist_.Add(pSocket);
//}
//inline void RF22B::WriteRegister(uint8_t RegNum, uint8_t Value)
//{
//  uint8_t data[2] = { RegNum | 0x80, Value };
//  ssp_.Send(data, 2);
//}
//inline uint8_t RF22B::ReadRegister(uint8_t RegNum)
//{
//  uint8_t data[2] = { RegNum & ~0x80 };
//  ssp_.Recv(data, 2);
//  return data[1];
//}
//// Requires the Data to contain the Registers address on it's first position
//inline void RF22B::WriteMultiple(uint8_t *pData, uint8_t count)
//{
//  pData[0] |= 0x80;
//  ssp_.Send(pData, count);
//}
//inline void RF22B::ReadMultiple(uint8_t *pData, uint8_t count)
//{
//  pData[0] &= ~0x80;
//  ssp_.Recv(pData, count);
//}
//inline RF22B::Packet* RF22B::Packetheap::find_free()
//{
//  iterator found = mtl::find_if(heap_.begin(), heap_.end(), RF22B_fn_packet_is_free());
//  if(found == heap_.end())
//    return 0;
//  return found.make_pointer();
//}
//inline RF22B::Socket* RF22B::Socketlist::find(uint8_t Identifier)
//{
//  iterator found = mtl::find(list_.begin(), list_.end(), Identifier);
//  if(found == list_.end())
//    return 0;
//  return found.make_pointer();
//}

#endif /* RF22B_STM8_H */