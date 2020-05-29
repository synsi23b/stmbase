#include "synhal.h"

// void * operator new(size_t n) {
//   return pvPortMalloc(n);
// }
// void * operator new[](size_t n) {
//   return pvPortMalloc(n);
// }
// void operator delete(void* p) _GLIBCXX_USE_NOEXCEPT {
//   vPortFree(p);
// }
// void operator delete[](void* p) _GLIBCXX_USE_NOEXCEPT {
//   vPortFree(p);
// }

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
  //   // turn on HSE and wait for it to be ready
  //   RCC->CR |= RCC_CR_HSEON;
  //   while (!(RCC->CR & RCC_CR_HSERDY))
  //     ;
  //   // configure and enable PLL to get 72 MHz ( 8MHz crystal x 9 )
  //   // adjust ABP1 / ADC / USB prescalers as well ( ADC running at 12 MHz )
  //   RCC->CFGR = RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLSRC | RCC_CFGR_ADCPRE_DIV6 | RCC_CFGR_PPRE1_DIV2;
  //   // enable the PLL
  //   RCC->CR |= RCC_CR_PLLON;
  //   // set the correct wait states for flash access
  //   FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;
  //   // wait for pll to be ready and switch
  //   while (!(RCC->CR & RCC_CR_PLLRDY))
  //     ;
  //   RCC->CFGR |= RCC_CFGR_SW_PLL;
  //   while (!(RCC->CFGR & RCC_CFGR_SWS_PLL))
  //     ;
  //   //SystemCoreClockUpdate(); // update the stored sys clock somewhere deep in the system files, just in case free rtos does shenanigans
  //   // setup nvic priority and systick to 1kHz
  //   NVIC_SetPriorityGrouping(3); // disable subpriorities
  //   //SysTick_Config(72000);
  //   //NVIC_SetPriority(SysTick_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY);
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
}

class Dma
{
public:
  Dma(uint16_t channel)
  {
    OS_ASSERT(channel < 8, ERR_BAD_INDEX);
    switch (channel)
    {
    case 1:
      _pChannel = DMA1_Channel1;
      break;
    case 2:
      _pChannel = DMA1_Channel2;
      break;
    case 3:
      _pChannel = DMA1_Channel3;
      break;
    case 4:
      _pChannel = DMA1_Channel4;
      break;
    case 5:
      _pChannel = DMA1_Channel5;
      break;
    case 6:
      _pChannel = DMA1_Channel6;
      break;
    case 7:
      _pChannel = DMA1_Channel7;
      break;
    }
  }

  // stop operation of the channel
  void stop()
  {
    _pChannel->CCR = 0;
  }

  // cylcic reading from a peripheral to memory. periheral stays the same, memory gets incremented
  template <typename Peri_t, typename Mem_t>
  void cyclicP2M(Peri_t *src, Mem_t *dst, uint16_t size)
  {
    stop();
    uint16_t psize = sizeof(Peri_t) >> 1;
    uint16_t msize = sizeof(Mem_t) >> 1;
    _pChannel->CCR = (msize << 10) | (psize << 8) | DMA_CCR1_MINC | DMA_CCR1_CIRC;
    _pChannel->CNDTR = size;
    _pChannel->CMAR = (uint32_t)dst;
    _pChannel->CPAR = (uint32_t)src;
    _pChannel->CCR |= DMA_CCR1_EN;
  }

private:
  DMA_Channel_TypeDef *_pChannel;
};

