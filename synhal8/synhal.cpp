#include "synhal.h"

volatile uint16_t syn::System::sMillis = 0;

// block for the specidifed ammount of microseconds using a busy loop
void syn::Utility::udelay(uint16_t micros)
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

using namespace syn;

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

void GpioBase::pushpull(uint8_t *port_base, uint8_t mask)
{
  // _pPort->DDR |= _pinmask;
  // _pPort->CR1 |= _pinmask;
  // _pPort->CR2 |= _pinmask;
  port_base += 2; // advance to DDR
  *port_base++ |= mask;
  *port_base++ |= mask;
  *port_base |= mask;
}

void Gpio::pushpull()
{
  // _pPort->DDR |= _pinmask;
  // _pPort->CR1 |= _pinmask;
  // _pPort->CR2 |= _pinmask;
  GpioBase::pushpull((uint8_t *)_pPort, _pinmask);
}

void GpioBase::opendrain(uint8_t *port_base, uint8_t mask)
{
  // _pPort->DDR |= _pinmask;
  // _pPort->CR1 &= ~_pinmask;
  // _pPort->CR2 |= _pinmask;
  // DDR -> CR1 -> CR2
  port_base += 2; // advance to DDR
  *port_base |= mask;
  port_base += 2; // advance to CR2
  *port_base |= mask;
  --port_base; // decrease to CR1
  mask = ~mask;
  *port_base &= mask;
}

void Gpio::opendrain()
{
  // out_opendrain, max speed
  // _pPort->DDR |= _pinmask;
  // _pPort->CR1 &= ~_pinmask;
  // _pPort->CR2 |= _pinmask;
  GpioBase::opendrain((uint8_t *)_pPort, _pinmask);
}

void GpioBase::input_pullup(uint8_t *port_base, uint8_t mask)
{
  // _pPort->DDR &= ~_pinmask;
  // _pPort->CR1 |= _pinmask;
  // _pPort->CR2 &= ~_pinmask;
  // DDR -> CR1 -> CR2
  port_base += 3; // advance to CR1
  *port_base |= mask;
  mask = ~mask;
  --port_base; // decrease to DDR
  *port_base &= mask;
  port_base += 2; // increase to CR2
  *port_base &= mask;
}

void Gpio::input_pullup()
{
  // _pPort->DDR &= ~_pinmask;
  // _pPort->CR1 |= _pinmask;
  // _pPort->CR2 &= ~_pinmask;
  GpioBase::input_pullup((uint8_t *)_pPort, _pinmask);
}

void GpioBase::floating(uint8_t *port_base, uint8_t mask)
{
  // _pPort->DDR &= ~_pinmask;
  // _pPort->CR1 &= ~_pinmask;
  // _pPort->CR2 &= ~_pinmask;
  mask = ~mask;
  // DDR -> CR1 -> CR2
  port_base += 2; // advance pointer to DDR register
  *port_base++ &= mask;
  *port_base++ &= mask;
  *port_base &= mask;
}

void Gpio::floating()
{
  // _pPort->DDR &= ~_pinmask;
  // _pPort->CR1 &= ~_pinmask;
  // _pPort->CR2 &= ~_pinmask;
  GpioBase::floating((uint8_t *)_pPort, _pinmask);
}

// block for the specidifed ammount of millis using systick
void System::delay(uint16_t millis)
{
  uint16_t end = millis + sMillis;
  while ((end - sMillis) <= millis)
    ;
}

#ifdef SYN_HAL_AUTO_WAKEUP_UNIT
INTERRUPT_HANDLER(AWU_ISR, 1)
{
  // read csr to clear interrupt occured bit
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
  // so lets hope that incrementing data, decremtening count and comapring takes
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
uint8_t *Uart::sRxData = 0;
volatile uint8_t Uart::sTxCount = 0;
volatile uint8_t Uart::sRxCount = 0;

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

// receiver isr, don't call manually
void Uart::rx_isr()
{
  *sRxData++ = UART1->DR;
  if (--sRxCount == 0)
  {
    // when all bytes are read, disable the interrupt again
    UART1->CR2 &= ~UART1_CR2_RIEN;
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
INTERRUPT_HANDLER(I2C_ISR, 19)
{
  I2c::isr();
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
