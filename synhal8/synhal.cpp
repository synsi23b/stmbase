#include "synhal.h"

volatile uint16_t syn::System::sMillis = 0;

// block for x milliseconds using an estimation.
void syn::busy_delay(uint16_t millis) {
  while (millis) {
    for (uint16_t i = 3200; i != 0; --i) {
      // 1 cylce takes 62,5 nanosec -> about 5 cycles per loop -> ~3200 loops for 1ms 
      nop();
    }
    --millis;
  }
}

// block for x microseconds using an estimation.
void syn::busy_udelay(uint16_t micros) {
  for (; micros != 0; --micros) {
    // 1 cycle takes 62,5 nanosec -> about 4 cycles for the loop -> 16 cycle = 1 microsec -> 12 nop();
    nop(); nop(); nop(); nop();
    nop(); nop(); nop(); nop();
    nop(); nop(); nop(); nop();
  }
}

using namespace syn;

INTERRUPT_HANDLER(AWU_ISR, 1) {
  // read csr to clear interrupt occured bit
  uint8_t val = AWU->CSR;
  Autowakeup::stopsleep();
}

#ifdef SYN_HAL_UART
const uint8_t* Uart::sTxData = 0;
uint8_t* Uart::sRxData = 0;
volatile uint8_t Uart::sTxCount = 0;
volatile uint8_t Uart::sRxCount = 0;
INTERRUPT_HANDLER(UART1_TX_ISR, 17) {
  Uart::tx_isr();
}
INTERRUPT_HANDLER(UART1_RX_ISR, 18) {
  Uart::rx_isr();
}
#endif

#ifdef SYN_HAL_I2C
#ifndef SYN_HAL_I2C_SLAVE
uint8_t* I2c::sData = 0;
volatile uint8_t I2c::sCount = 0;
INTERRUPT_HANDLER(I2C_ISR, 19) {
  I2c::isr();
}
#else
I2cSlave::slaveTxHandle_t I2cSlave::sTxHandle;
I2cSlave::slaveRxHandle_t I2cSlave::sRxHandle;
uint8_t I2cSlave::sCurrentByte;
INTERRUPT_HANDLER(I2C_ISR, 19) {
  I2cSlave::isr();
}
#endif
#endif

