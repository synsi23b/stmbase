#include "synhal.h"

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

void Utility::memcpy(uint8_t* dst, const uint8_t *src, uint8_t count)
{
  while(count-- != 0)
  {
    *dst++ = *src++;
  }
}

void Utility::clear_array(uint8_t* data, uint8_t count)
{
  while(count-- != 0)
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

// block for the specidifed ammount of millis using systick
void System::delay(uint16_t millis)
{
  uint16_t end = millis + _millis;
  while ((end - _millis) <= millis)
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
