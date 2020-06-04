#include "synhal.h"

using namespace syn;

void SoftTimer::_oneshot(SoftTimer *this_timer)
{
  this_timer->execute();
}

void SoftTimer::_reload(SoftTimer *this_timer)
{
  this_timer->execute();
  this_timer->restart();
}

void Thread::runner(Thread *this_thread)
{
  this_thread->run();
  this_thread->terminate();
}

void System::init()
{
  OS_Init();
  OS_InitHW(); // sets up clock 72Mhz and systick 1kHz
  // set adc 12MHz and USB 48Mhz
  RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;
  // enable peripheral clocks for GPIOs and AFIO
  RCC->AHBENR = RCC_AHBENR_FLITFEN | RCC_AHBENR_SRAMEN | RCC_AHBENR_DMA1EN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
  // set the default remappings
  AFIO->MAPR = AFIO_MAPR_SWJ_CFG_NOJNTRST; // | AFIO_MAPR_TIM3_REMAP_PARTIALREMAP; // | AFIO_MAPR_USART1_REMAP
  //Gpio::remap(Gpio::tim3_remap_part);
  //Gpio::remap(Gpio::swj_no_nrst); // pb3 should be free.. should...
#if (SYN_CAN_1_REMAP != 0)
  Gpio::remap(Gpio::can_rx_pb8_tx_pb9)
#endif
#if (SYN_ENABLE_USBRPC != 0)
  UsbRpc::init();
#endif
}

