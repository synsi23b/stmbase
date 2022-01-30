#include "synhal.h"
#include <synos.h>

using namespace syn;

volatile uint16_t System::_millis = 0;
#ifdef SYN_SYSTEM_SECONDS_SUPPORT
uint32_t System::_seconds = 0;
#endif

// block for the specidifed ammount of microseconds using a busy loop
void Utility::udelay(uint16_t micros)
{
  for (; micros != 0; --micros)
  {
    // 1 cycle takes 62,5 nanosec -> about 4 cycles for the loop -> 16 cycle = 1 microsec -> 12 nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
  }
}

// return number of characters excluding trailing zero
uint8_t Utility::strlen(const char *str)
{
  uint8_t count = 0;
  while (*str != 0)
  {
    ++str;
    ++count;
  }
  return count;
}

// copys the string until trailing zero and returns number of bytes copied
// doesn't add a trailing zero and doesn't check for anything really
char *Utility::strcpy(char *dst, const char *str)
{
  while (*str != 0)
  {
    *dst++ = *str++;
  }
  return dst;
}

// write signed integer as asci to the outbuffer, return bytes written
char *Utility::sprint_i16(char *dst, int16_t value)
{
  if (value < 0)
  {
    value *= -1;
    *dst++ = '-';
  }
  return sprint_u16(dst, value);
}

// write unsigned integer as asci to the outbuffer, return bytes written
char *Utility::sprint_u16(char *dst, uint16_t value)
{
  if (value == 0)
  {
    *dst = '0';
    return dst + 1;
  }
  uint16_t modulo = 10000;
  // find modulo start value
  uint8_t tmp;
  while (true)
  {
    tmp = value / modulo;
    value = value % modulo;
    if (tmp != 0)
      break;
    modulo /= 10;
  }
  // start add ingintegers to buffer, no leading zeros
  while (true)
  {
    *dst++ = tmp + '0';
    modulo /= 10;
    if (modulo == 0)
      break;
    tmp = value / modulo;
    value = value % modulo;
  }
  return dst;
}

char *_write_single_hex(char *dst, uint8_t value)
{
  if (value < 10)
  {
    *dst++ = value + '0';
  }
  else
  {
    *dst++ = value + ('A' - 10);
  }
  return dst;
}

char *Utility::sprint_hex(char *dst, uint8_t value)
{
  dst = _write_single_hex(dst, (value / 16));
  return _write_single_hex(dst, value & 0x0F);
}

bool Utility::memcmp(const uint8_t *a1, const uint8_t *a2, uint8_t len)
{
  while (len-- != 0)
  {
    if (*a1++ != *a2++)
    {
      return false;
    }
  }
  return true;
}

uint8_t *Utility::memcpy(uint8_t *dst, const uint8_t *src, uint8_t count)
{
  while (count-- != 0)
  {
    *dst++ = *src++;
  }
  return dst;
}

void Utility::clear_array(uint8_t *data, uint8_t count)
{
  while (count-- != 0)
  {
    *data++ = 0;
  }
}

void GpioBase::pushpull(GPIO_TypeDef *port, uint8_t mask)
{
  // _port->DDR |= _pinmask;
  // _port->CR1 |= _pinmask;
  // _port->CR2 |= _pinmask;
  uint8_t *port_base = (uint8_t *)&port->DDR;
  *port_base++ |= mask;
  //*port_base++ |= mask;
  *port_base |= mask;
}

Gpio &Gpio::pushpull()
{
  // _port->DDR |= _pinmask;
  // _port->CR1 |= _pinmask;
  // _port->CR2 |= _pinmask;
  GpioBase::pushpull(_port, _pinmask);
  return *this;
}

void GpioBase::opendrain(GPIO_TypeDef *port, uint8_t mask)
{
  // _port->DDR |= _pinmask;
  // _port->CR1 &= ~_pinmask;
  // _port->CR2 |= _pinmask;
  // DDR -> CR1 -> CR2
  uint8_t *port_base = (uint8_t *)&port->DDR;
  *port_base++ |= mask;
  //port_base += 2; // advance to CR2
  //*port_base |= mask;
  //--port_base; // decrease to CR1
  mask = ~mask;
  *port_base &= mask;
}

Gpio &Gpio::opendrain()
{
  // out_opendrain, max speed
  // _port->DDR |= _pinmask;
  // _port->CR1 &= ~_pinmask;
  // _port->CR2 |= _pinmask;
  GpioBase::opendrain(_port, _pinmask);
  return *this;
}

void GpioBase::input_pullup(GPIO_TypeDef *port, uint8_t mask)
{
  // _port->DDR &= ~_pinmask;
  // _port->CR1 |= _pinmask;
  // _port->CR2 &= ~_pinmask;
  // DDR -> CR1 -> CR2
  uint8_t *port_base = (uint8_t *)&port->DDR;
  *port_base++ &= ~mask;
  *port_base |= mask;
}

Gpio &Gpio::input_pullup()
{
  // _port->DDR &= ~_pinmask;
  // _port->CR1 |= _pinmask;
  // _port->CR2 &= ~_pinmask;
  GpioBase::input_pullup(_port, _pinmask);
  return *this;
}

void GpioBase::floating(GPIO_TypeDef *port, uint8_t mask)
{
  // _port->DDR &= ~_pinmask;
  // _port->CR1 &= ~_pinmask;
  // _port->CR2 &= ~_pinmask;
  mask = ~mask;
  // DDR -> CR1
  uint8_t *port_base = (uint8_t *)&port->DDR;
  *port_base++ &= mask;
  *port_base &= mask;
  //*port_base &= mask;
}

Gpio &Gpio::floating()
{
  // _port->DDR &= ~_pinmask;
  // _port->CR1 &= ~_pinmask;
  // _port->CR2 &= ~_pinmask;
  GpioBase::floating(_port, _pinmask);
  return *this;
}

void GpioBase::high_speed(GPIO_TypeDef *port, uint8_t mask)
{
  port->CR2 |= mask;
}

Gpio &Gpio::high_speed()
{
  _port->CR2 |= _pinmask;
  return *this;
}

// block for the specified amount of milliseconds using systick
void System::delay(uint16_t millis)
{
  uint16_t end = millis + _millis;
  while ((end - _millis) <= millis)
    ;
}

#ifdef SYN_HAL_AUTO_WAKEUP_UNIT
INTERRUPT_HANDLER(AWU_ISR, 1)
{
  // read csr to clear interrupt occurred bit
  uint8_t val = AWU->CSR;
  Autowakeup::stopsleep();
}
#endif

#ifdef SYN_HAL_SPI

// no need to check for TXE, because always make sure to leave
// the device gracefully and read to run when re-entering the method.
// it also means, it needs to be mutexed. But that should be obvious

void SpiNC::write_command(uint8_t command, const uint8_t *data, uint8_t count)
{
  SPI->DR = command;
  // keep on writing until reaching the frame count
  while (count != 0)
  {
    while (!(SPI->SR & SPI_SR_TXE))
      ;
    SPI->DR = *data++;
    --count;
  }
  // wait for the transactions to be finished
  // this flag can be late to be set by 2 cpu cycles after writing the DR
  // so lets hope that incrementing data, decrementing count and comparing takes
  // longer than getting to the load of that status register
  while (SPI->SR & SPI_SR_BSY)
    ;
  // dummy read rx to make sure RXNE is clear
  uint8_t dummy = SPI->DR;
}

void SpiNC::transceive(uint8_t *data, uint8_t count)
{
  // keep on writing until reaching the frame count
  while (count != 0)
  {
    SPI->DR = *data;
    --count;
    while (!(SPI->SR & SPI_SR_RXNE))
      ;
    // read the answer byte and save it
    *data++ = SPI->DR;
    // saves 8 bytes of PROGMEM, but is 28% slower at 8MHz SPI
    // uint8_t tmp = transceive1(*data);
    // *data++ = tmp;
    // --count;
  }
}

uint8_t SpiNC::transceive1(uint8_t data)
{
  SPI->DR = data;
  while (!(SPI->SR & SPI_SR_RXNE))
    ;
  return SPI->DR;
}

#endif

#ifdef SYN_HAL_UART
const uint8_t *Uart::sTxData = 0;
volatile uint8_t Uart::sTxCount = 0;
#if SYN_HAL_UART_RX_BUFFER_SIZE != 0
static AsyncBuffer<uint8_t, SYN_HAL_UART_RX_BUFFER_SIZE> UartsRxBuffer;

// receiver ISR, don't call manually
void Uart::rx_isr()
{
  // read data into temporary to make sure we read it, regardless off buffering.
  // else we get stuck in a ISR loop because UART_IRQ never gets cleared
  uint8_t data = UART1->SR;
  data = UART1->DR;
  UartsRxBuffer.put_isr(data);
}

// check the amount of bytes from last async read are still not read
uint8_t Uart::rx_avail()
{
  return UartsRxBuffer.available();
}

void Uart::rx_start()
{
  Atomic a;
  // clear the receiver not empty flag
  UART1->SR &= ~UART1_SR_RXNE;
  // enable UART interrupt to take care of all transmitting
  UART1->CR2 |= UART1_CR2_RIEN;
}

void Uart::rx_stop()
{
  Atomic a;
  UART1->CR2 &= ~UART1_CR2_RIEN;
}

void Uart::rx_flush()
{
  UartsRxBuffer.flush();
}

bool Uart::rx_overrun()
{
  return UartsRxBuffer.overrun();
}

uint8_t Uart::peek(uint8_t offset)
{
  return UartsRxBuffer.peek(offset);
}

// read up to count bytes into the buffer data
uint8_t Uart::read(uint8_t *data, uint8_t count, uint8_t offset)
{
  return UartsRxBuffer.pop(data, count, offset);
}
#else
// receiver isr, don't call manually
void Uart::rx_isr()
{
  UART1->CR2 &= ~UART1_CR2_RIEN; // disable RX interrupt
}
#endif
// transmitter isr, don't call manually
void Uart::tx_isr()
{
  UART1->DR = *sTxData++;
  if (--sTxCount == 0)
  {
    // when all bytes are transmitted, disable the interrupt again
    UART1->CR2 &= ~UART1_CR2_TIEN;
  }
}

INTERRUPT_HANDLER(UART1_TX_ISR, 17)
{
  Uart::tx_isr();
}
INTERRUPT_HANDLER(UART1_RX_ISR, 18)
{
  Uart::rx_isr();
}
#endif

#ifdef SYN_HAL_I2C
#ifndef SYN_HAL_I2C_SLAVE
uint8_t *I2c::sData = 0;
volatile uint8_t I2c::sCount = 0;
uint8_t I2c::sAddress = 0;
uint8_t I2c::sTxInjectByte = 0;

// initialize the device as master that has no address in either
// fast or slow mode
void I2c::init(Speed speed)
{
  // reset the device because glitches at startup
  I2C->CR2 = I2C_CR2_SWRST;
  Gpio pins;
  pins.init_multi('B', 0x30)
      .opendrain()
      .set();
  I2C->CR2 = 0;
  I2C->FREQR = 16; // 16 MHz main clock
  if (speed == slow)
  {
    I2C->CCRH = 0x00;
    I2C->CCRL = 0x50; // 100 kHz
    I2C->TRISER = 17; // max rise time 1000 ns
  }
  else if (speed == medium)
  {
    I2C->CCRH = I2C_CCRH_FS;
    I2C->CCRL = 0x1A; // 205 kHz
    I2C->TRISER = 6;  // max rise time 300 ns
  }
  else
  {
    I2C->CCRH = I2C_CCRH_FS;
    I2C->CCRL = 0x0E; // 380 kHz
    I2C->TRISER = 6;  // max rise time 300 ns
  }
  // set address configuration complete, has to be done, even in master mode
  I2C->OARL = 0;
  I2C->OARH = I2C_OARH_ADDCONF;
  // raise i2c interrupt to highest level because errata sheet
  ITC->ISPR5 |= 0xC0;
  // enable the interrupts
  I2C->ITR = I2C_ITR_ITBUFEN | I2C_ITR_ITEVTEN | I2C_ITR_ITERREN;
}

// Check if the Address specified is ACKed (true) or NACKed (false)
// bus error also returns false
bool I2c::checkSlave(uint8_t address)
{
  while (busy())
    ;
  // disable the interrupts
  I2C->ITR = 0;
  // enable the device
  I2C->CR1 = I2C_CR1_PE;
  // generate a start condition and enter a polling loop
  I2C->CR2 = I2C_CR2_START;
  bool slaveanswer = false;
  while (true)
  {
    // any error before even transmitting the address
    // means we couldn't even access the bus
    if (I2C->SR2 != 0)
      break;
    if (I2C->SR1 & I2C_SR1_SB)
    {
      // start condition generated, transmit address
      I2C->DR = address;
      while (true)
      {
        uint8_t state = I2C->SR2;
        // slave address not acknowledged
        if (state & I2C_SR2_AF)
          break;
        // any other error
        if (state != 0)
          break;
        // address acknowledged
        if (I2C->SR1 & I2C_SR1_ADDR)
        {
          // generate a stop condition
          I2C->CR2 = I2C_CR2_STOP;
          slaveanswer = true;
          break;
        }
      }
      break;
    }
  }
  // disable the device to clean up any flags that might have been set
  I2C->CR1 = 0;
  // enable the interrupts again
  I2C->ITR = I2C_ITR_ITBUFEN | I2C_ITR_ITEVTEN | I2C_ITR_ITERREN;
  return slaveanswer;
}

// read the amount of data specified by count. data needs to be at least that size
// the address of the slave is expected to be located in data[0] and will be overwritten
// by the data returned from slave
void I2c::read_async(uint8_t address, uint8_t *data, uint8_t count)
{
  //while(I2C->SR3 & I2C_SR3_BUSY)
  //  ;
  while (busy())
    ;
  sData = data;
  sCount = count;
  sAddress = address | 0x01; // set LSB of address to enter receiver mode
  // enable the device
  I2C->CR1 = I2C_CR1_PE;
  if (count > 2)
  {
    I2C->CR2 = I2C_CR2_ACK | I2C_CR2_START;
  }
  else if (count == 2)
  {
    // set ACK and POS to NACK the 2nd byte
    I2C->CR2 = I2C_CR2_POS | I2C_CR2_ACK | I2C_CR2_START;
  }
  else
  {
    // read only 1 byte, don't set the ACK bit if only one byte will be read
    I2C->CR2 = I2C_CR2_START;
  }
}

// read the amount of data specified by count. data needs to be at least that size
// the address of the slave is expected to be located in data[0] and will be overwritten
// by the data returned from slave
// returns true on completion and false in case of an error
bool I2c::read(uint8_t address, uint8_t *data, uint8_t count)
{
  read_async(address, data, count);
  while (true)
  {
    if (state() == 0xFF)
    {
      sCount = 0; // reset counter, not busy anymore
      return false;
    }
    if (state() == 0x00)
      return true;
  }
}

// write the amount of data specified by count - 1.
// data needs to be at least of the size count
void I2c::write_async(uint8_t address, const uint8_t *data, uint8_t count)
{
  //while(I2C->SR3 & I2C_SR3_BUSY)
  //  ;
  while (busy())
    ;
  sData = (uint8_t *)data;
  sCount = count + 1;
  sAddress = address;
  // enable the device
  I2C->CR1 = I2C_CR1_PE;
  I2C->CR2 = I2C_CR2_START;
}

// write the amount of data specified by count - 1.
// data needs to be at least of the size count
// the address of the slave is expected to be located in data[0].
// returns true on completion and false in case of an error
bool I2c::write(uint8_t address, const uint8_t *data, uint8_t count)
{
  write_async(address, data, count);
  while (true)
  {
    if (state() == 0xFF)
    {
      //sCount = 0; // reset counter, not busy anymore
      return false;
    }
    if (state() == 0x00)
      return true;
  }
}

void I2c::_isr()
{
  uint8_t state = I2C->SR2;
  if (state != 0)
  {
    // a wild error has appeared
    sCount = 0xFF;
    //I2C->SR2 = 0;
    I2C->CR2 = I2C_CR2_STOP;
    // disable the device again
    I2C->CR1 = 0;
  }
  else
  {
    state = I2C->SR1;
    if (state == 0)
      return;
    if (state & I2C_SR1_ADDR)
    {
      // address ACKed, read SR3 to clear the state
      state = I2C->SR3;
      if (state & I2C_SR3_TRA)
      {
        // entering transmit mode, because of address LSB was not set
        // put the injection byte
        I2C->DR = sTxInjectByte;
        if (--sCount == 0)
        {
          // on last byte transmitted, generate a stop condition
          I2C->CR2 = I2C_CR2_STOP;
        }
      }
      else
      {
        uint8_t count = sCount;
        if (count == 2)
        {
          // if receiver mode with exactly 2 data bytes. clear ACK to NACK 2nd byte
          I2C->CR2 = I2C_CR2_POS;
        }
        else if (count == 1)
        {
          // if only one byte is to received set a stop instead. The stop will come after the
          // already ongoing transmission
          I2C->CR2 = I2C_CR2_STOP;
        }
      }
    }
    else if (state & I2C_SR1_RXNE)
    {
      *sData++ = I2C->DR;
      if (--sCount == 1)
      {
        // on 2nd last byte, clear ACK to NACK last and send a stop after transmission is complete
        I2C->CR2 = I2C_CR2_STOP;
      }
    }
    else if (state & (I2C_SR1_TXE | I2C_SR1_BTF))
    {
      if (sCount == 0)
      {
        // why does the device get here?
        I2C->CR2 = I2C_CR2_STOP;
      }
      else
      {
        // transmitter empty, put data
        I2C->DR = *sData++;
        --sCount;
        //{
        // on last byte, generate a stop condition
        //I2C->CR2 = I2C_CR2_STOP;
        //}
      }
    }
    else if (state & I2C_SR1_SB)
    {
      // start condition send, transmit address
      I2C->DR = sAddress;
    }
  }
}

INTERRUPT_HANDLER(I2C_ISR, 19)
{
  I2c::_isr();
}
#else
I2cSlave::slaveTxHandle_t I2cSlave::sTxHandle;
I2cSlave::slaveRxHandle_t I2cSlave::sRxHandle;
uint8_t I2cSlave::sCurrentByte;
INTERRUPT_HANDLER(I2C_ISR, 19)
{
  I2cSlave::isr();
}
#endif
#endif

void Adc::configurepin(uint8_t channel)
{
  Gpio pin;
#ifndef SYN_HAL_32_PIN_DEVICE
  switch (channel)
  {
  case 2:
    pin.init('C', 4);
    break;
  case 3:
    pin.init('D', 2);
    break;
  case 4:
    pin.init('D', 3);
    break;
  case 5:
    pin.init('D', 5);
    break;
  case 6:
    pin.init('D', 6);
    break;
  default:
    return;
  };
  pin.floating();
#else
  pin.init('B', channel)
      .floating();
#endif
  ADC1->TDRL |= (1 << channel);
}

void Adc::random(uint8_t *buff, uint8_t count, uint8_t channel)
{
  initContinuous(channel);
  while (count != 0)
  {
    uint8_t tmp = 0;
    for (uint8_t mask = 0x80; mask != 0; mask >>= 1)
    {
      uint16_t av = syn::Adc::read();
      if (av & 0x01)
      {
        tmp |= mask;
      }
      syn::Utility::udelay(4);
    }
    *buff++ = tmp;
    --count;
  }
}

/* using the numbers from the manual works just fine, I dont know why they defined it this way */

#define AWU_vector 3                /* IRQ No. in STM8 manual:  1 */
#define CLK_CSS_vector 4            /* IRQ No. in STM8 manual:  2 */
#define CLK_SWITCH_vector 4         /* IRQ No. in STM8 manual:  2 */
#define EXTI0_vector 5              /* IRQ No. in STM8 manual:  3 */
#define EXTI1_vector 6              /* IRQ No. in STM8 manual:  4 */
#define EXTI2_vector 7              /* IRQ No. in STM8 manual:  5 */
#define EXTI3_vector 8              /* IRQ No. in STM8 manual:  6 */
#define EXTI4_vector 9              /* IRQ No. in STM8 manual:  7 */
#define SPI_CRCERR_vector 12        /* IRQ No. in STM8 manual: 10 */
#define SPI_MODF_vector 12          /* IRQ No. in STM8 manual: 10 */
#define SPI_OVR_vector 12           /* IRQ No. in STM8 manual: 10 */
#define SPI_RXNE_vector 12          /* IRQ No. in STM8 manual: 10 */
#define SPI_TXE_vector 12           /* IRQ No. in STM8 manual: 10 */
#define SPI_WKUP_vector 12          /* IRQ No. in STM8 manual: 10 */
#define TIM1_CAPCOM_BIF_vector 13   /* IRQ No. in STM8 manual: 11 */
#define TIM1_CAPCOM_TIF_vector 13   /* IRQ No. in STM8 manual: 11 */
#define TIM1_OVR_UIF_vector 13      /* IRQ No. in STM8 manual: 11 */
#define TIM1_CAPCOM_CC1IF_vector 14 /* IRQ No. in STM8 manual: 12 */
#define TIM1_CAPCOM_CC2IF_vector 14 /* IRQ No. in STM8 manual: 12 */
#define TIM1_CAPCOM_CC3IF_vector 14 /* IRQ No. in STM8 manual: 12 */
#define TIM1_CAPCOM_CC4IF_vector 14 /* IRQ No. in STM8 manual: 12 */
#define TIM1_CAPCOM_COMIF_vector 14 /* IRQ No. in STM8 manual: 12 */
#define TIM2_OVR_UIF_vector 15      /* IRQ No. in STM8 manual: 13 */
#define TIM2_CAPCOM_CC1IF_vector 16 /* IRQ No. in STM8 manual: 14 */
#define TIM2_CAPCOM_CC2IF_vector 16 /* IRQ No. in STM8 manual: 14 */
#define TIM2_CAPCOM_CC3IF_vector 16 /* IRQ No. in STM8 manual: 14 */
#define TIM2_CAPCOM_TIF_vector 16   /* IRQ No. in STM8 manual: 14 */
#define UART1_T_TC_vector 19        /* IRQ No. in STM8 manual: 17 */
#define UART1_T_TXE_vector 19       /* IRQ No. in STM8 manual: 17 */
#define UART1_R_IDLE_vector 20      /* IRQ No. in STM8 manual: 18 */
#define UART1_R_LBDF_vector 20      /* IRQ No. in STM8 manual: 18 */
#define UART1_R_OR_vector 20        /* IRQ No. in STM8 manual: 18 */
#define UART1_R_PE_vector 20        /* IRQ No. in STM8 manual: 18 */
#define UART1_R_RXNE_vector 20      /* IRQ No. in STM8 manual: 18 */
#define I2C_ADD10_vector 21         /* IRQ No. in STM8 manual: 19 */
#define I2C_ADDR_vector 21          /* IRQ No. in STM8 manual: 19 */
#define I2C_AF_vector 21            /* IRQ No. in STM8 manual: 19 */
#define I2C_ARLO_vector 21          /* IRQ No. in STM8 manual: 19 */
#define I2C_BERR_vector 21          /* IRQ No. in STM8 manual: 19 */
#define I2C_BTF_vector 21           /* IRQ No. in STM8 manual: 19 */
#define I2C_OVR_vector 21           /* IRQ No. in STM8 manual: 19 */
#define I2C_RXNE_vector 21          /* IRQ No. in STM8 manual: 19 */
#define I2C_SB_vector 21            /* IRQ No. in STM8 manual: 19 */
#define I2C_STOPF_vector 21         /* IRQ No. in STM8 manual: 19 */
#define I2C_TXE_vector 21           /* IRQ No. in STM8 manual: 19 */
#define I2C_WUFH_vector 21          /* IRQ No. in STM8 manual: 19 */
#define ADC1_AWDG_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS0_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS1_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS2_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS3_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS4_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS5_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS6_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS7_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS8_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_AWS9_vector 24         /* IRQ No. in STM8 manual: 22 */
#define ADC1_EOC_vector 24          /* IRQ No. in STM8 manual: 22 */
#define TIM4_OVR_UIF_vector 25      /* IRQ No. in STM8 manual: 23 */
#define FLASH_EOP_vector 26         /* IRQ No. in STM8 manual: 24 */
#define FLASH_WR_PG_DIS_vector 26   /* IRQ No. in STM8 manual: 24 */

#ifdef DEBUG_INTERRUPTS
INTERRUPT_HANDLER(AWU_ISR, 1)
{
  while (true)
  {
  }
}

INTERRUPT_HANDLER(EXTI0_ISR, 3)
{
  while (true)
  {
  }
}

INTERRUPT_HANDLER(EXTI1_ISR, 4)
{
  while (true)
  {
  }
}

INTERRUPT_HANDLER(EXTI2_ISR, 5)
{
  while (true)
  {
  }
}

INTERRUPT_HANDLER(EXTI3_ISR, 6)
{
  while (true)
  {
  }
}

INTERRUPT_HANDLER(EXTI4_ISR, 7)
{
  while (true)
  {
  }
}

INTERRUPT_HANDLER(SPI_ISR, 12)
{
  while (true)
  {
  }
}

INTERRUPT_HANDLER(UART1_TX_ISR, 17)
{
  while (true)
  {
  }
}

INTERRUPT_HANDLER(UART1_RX_ISR, 18)
{
  while (true)
  {
  }
}

INTERRUPT_HANDLER(I2C_ISR, 19)
{
  while (true)
  {
  }
}
#endif
