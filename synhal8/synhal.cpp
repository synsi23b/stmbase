#include "synhal.h"

volatile uint16_t syn::System::sMillis = 0;


// block for the specidifed ammount of microseconds using a busy loop
void syn::udelay(uint16_t micros)
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

// block for the specidifed ammount of millis using systick
void System::delay(uint16_t millis)
{
  uint16_t end = millis + sMillis;
  while ((end - sMillis) <= millis)
    ;
}

INTERRUPT_HANDLER(AWU_ISR, 1)
{
  // read csr to clear interrupt occured bit
  uint8_t val = AWU->CSR;
  Autowakeup::stopsleep();
}

#ifdef SYN_HAL_SPI

void SpiNC::read(uint8_t *data, uint8_t count)
{
  // keep on writing until reaching the frame count
  while (count != 0)
  {
    // only write if transmitter is empty
    if (SPI->SR & SPI_SR_TXE)
    {
      SPI->DR = 0x00;
      --count;
      // wait for receiver to be not empty and read the data
      while (!(SPI->SR & SPI_SR_RXNE))
        ;
      *(uint8_t *)data++ = SPI->DR;
    }
  }
}

void SpiNC::write(const uint8_t *data, uint8_t count)
{
  // keep on writing until reaching the frame count
  while (count != 0)
  {
    // only write if transmitter is empty
    if (SPI->SR & SPI_SR_TXE)
    {
      SPI->DR = *(uint8_t *)data++;
      --count;
    }
  }
  // wait till the last written byte is inside the shift register
  while (!(SPI->SR & SPI_SR_TXE))
    ;
  // wait for the transactions to be finished
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
    // only write if transmitter is empty
    if (SPI->SR & SPI_SR_TXE)
    {
      SPI->DR = *data;
      --count;
      while (!(SPI->SR & SPI_SR_RXNE))
        ;
      // read the answer byte and save it
      *(uint8_t *)data++ = SPI->DR;
    }
  }
}

uint8_t SpiNC::transceive1(uint8_t data)
{
  while (!(SPI->SR & SPI_SR_TXE))
    ;
  SPI->DR = data;
  while (!(SPI->SR & SPI_SR_RXNE))
    ;
  return SPI->DR;
}

// write and read 2 bytes
void SpiNC::transceive2(uint8_t *data)
{
  while (!(SPI->SR & SPI_SR_TXE))
    ;
  SPI->DR = *data;
  // wait for receiver to be not empty and read the data
  while (!(SPI->SR & SPI_SR_RXNE))
    ;
  *data++ = SPI->DR;
  while (!(SPI->SR & SPI_SR_TXE))
    ;
  SPI->DR = *data;
  // wait for receiver to be not empty and read the data
  while (!(SPI->SR & SPI_SR_RXNE))
    ;
  *data = SPI->DR;
}

// write 2 bytes but only read back the last received
void SpiNC::transceive2_01(uint8_t *data)
{
  while (!(SPI->SR & SPI_SR_TXE))
    ;
  SPI->DR = *data++;
  while (!(SPI->SR & SPI_SR_TXE))
    ;
  SPI->DR = *data;
  while (SPI->SR & SPI_SR_BSY)
    ;
  *data = SPI->DR;
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
