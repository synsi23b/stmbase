#include "synhal.h"
using namespace syn;
#if SYN_TIMER_1_IRQ_TYPE || SYN_TIMER_2_IRQ_TYPE || SYN_TIMER_3_IRQ_TYPE || SYN_TIMER_4_IRQ_TYPE || SYN_TIMER_5_IRQ_TYPE
void Timer::enableCallback(uint16_t priority)
{
#if (SYN_TIMER_1_IRQ_TYPE != 0)
  if (_pTimer == TIM1)
  {
    Atomic a;
    _pTimer->DIER = TIM_DIER_UIE;
    _pTimer->SR = 0;
#ifdef STM32F401xC
    Core::enable_isr(TIM1_UP_TIM10_IRQn, priority);
#else
    Core::enable_isr(TIM1_UP_IRQn, priority);
#endif
    return;
  }
#endif
#if (SYN_TIMER_2_IRQ_TYPE != 0)
  if (_pTimer == TIM2)
  {
    Atomic a;
    _pTimer->DIER = TIM_DIER_UIE;
    _pTimer->SR = 0;
    Core::enable_isr(TIM2_IRQn, priority);
    return;
  }
#endif
#if (SYN_TIMER_3_IRQ_TYPE != 0)
  if (_pTimer == TIM3)
  {
    Atomic a;
    _pTimer->DIER = TIM_DIER_UIE;
    _pTimer->SR = 0;
    Core::enable_isr(TIM3_IRQn, priority);
    return;
  }
#endif
#if (SYN_TIMER_4_IRQ_TYPE != 0)
  if (_pTimer == TIM4)
  {
    Atomic a;
    _pTimer->DIER = TIM_DIER_UIE;
    _pTimer->SR = 0;
    Core::enable_isr(TIM4_IRQn, priority);
    return;
  }
#endif
#ifdef STM32F401xC
#if (SYN_TIMER_5_IRQ_TYPE != 0)
#endif
  if (_pTimer == TIM5)
  {
    Atomic a;
    _pTimer->DIER = TIM_DIER_UIE;
    _pTimer->SR = 0;
    Core::enable_isr(TIM5_IRQn, priority);
    return;
  }
#endif
}
#include "../../src/synhal_isr.h"
extern "C"
{

#ifdef STM32F401xC
// ISR_HANDLER TIM1_BRK_TIM9_IRQHandler
// ISR_HANDLER TIM1_UP_TIM10_IRQHandler
// ISR_HANDLER TIM1_TRG_COM_TIM11_IRQHandler
// ISR_HANDLER TIM1_CC_IRQHandler
#if (SYN_TIMER_1_IRQ_TYPE == 1)
  void TIM1_UP_TIM10_IRQHandler()
  {
    Core::enter_isr();
    syn_timer_1_isr();
    Core::leave_isr();
  }
#elif (SYN_TIMER_1_IRQ_TYPE == 2)
  void TIM1_UP_TIM10_IRQHandler()
  {
    syn_timer_1_isr();
  }
#endif
#else
#if (SYN_TIMER_1_IRQ_TYPE == 1)
  void TIM1_UP_IRQHandler()
  {
    Core::enter_isr();
    syn_timer_1_isr();
    Core::leave_isr();
  }
#elif (SYN_TIMER_1_IRQ_TYPE == 2)
  void TIM1_UP_IRQHandler()
  {
    syn_timer_1_isr();
  }
#endif

#endif
#if (SYN_TIMER_2_IRQ_TYPE == 1)
  void TIM2_IRQHandler()
  {
    Core::enter_isr();
    syn_timer_2_isr();
    Core::leave_isr();
  }
#elif (SYN_TIMER_2_IRQ_TYPE == 2)
  void TIM2_IRQHandler()
  {
    syn_timer_2_isr();
  }
#endif
#if (SYN_TIMER_3_IRQ_TYPE == 1)
  void TIM3_IRQHandler()
  {
    Core::enter_isr();
    syn_timer_3_isr();
    Core::leave_isr();
  }
#elif (SYN_TIMER_3_IRQ_TYPE == 2)
  void TIM3_IRQHandler()
  {
    syn_timer_3_isr();
  }
#endif
#if (SYN_TIMER_4_IRQ_TYPE == 1)
  void TIM4_IRQHandler()
  {
    Core::enter_isr();
    syn_timer_4_isr();
    Core::leave_isr();
  }
#elif (SYN_TIMER_4_IRQ_TYPE == 2)
  void TIM4_IRQHandler()
  {
    syn_timer_4_isr();
  }
#endif
#if (SYN_TIMER_5_IRQ_TYPE == 1)
  void TIM5_IRQHandler()
  {
    Core::enter_isr();
    syn_timer_5_isr();
    Core::leave_isr();
  }
#elif (SYN_TIMER_5_IRQ_TYPE == 2)
  void TIM5_IRQHandler()
  {
    syn_timer_5_isr();
  }
#endif // enable_timer_irq
} // extern c
#endif // if(SYN_ENABLE_TIMER_CALLBACK == 1)

#if(SYN_TIMER_1_RAMP_MODE != 0)
uint16_t _tim1_ramp_target = 0;
int16_t _tim1_ramp_delta = 0;
uint16_t _tim1_ramp_buffer[SYN_TIMER_1_RAMP_MODE];
Dma _tim1_dma;
#endif // #if(SYN_TIMER_1_RAMP_MODE != 0)

void Timer::ramp(uint32_t target_hz, uint16_t delta)
{
  Dma* pdma = 0;
  uint16_t* pbuf = 0;
  uint16_t bufsize = 0;
  volatile uint16_t* ptarget = 0;
#if(SYN_TIMER_1_RAMP_MODE != 0)
  if(_number == 1)
  {
    pbuf = _tim1_ramp_buffer;
    bufsize = SYN_TIMER_1_RAMP_MODE;
    pdma = &_tim1_dma;
    ptarget = &TIM1->ARR;
  }
#endif // #if(SYN_TIMER_1_RAMP_MODE != 0)
  if(pdma == 0)
    return;
  pdma->cyclicM2P(pbuf, ptarget, bufsize);
  // prepare the first buffer, after that, DMA irqs will do this until target is reached
  uint16_t* pbufend = pbuf + bufsize;
  uint32_t tclk = _tclk / 2;
  uint16_t arr = this->arr();
  uint32_t cur_hz = 0;
  if(arr > 0)
    cur_hz = tclk / arr;

  if(target_hz == cur_hz)
   return;    

  if(target_hz < cur_hz)
  {
    for(;pbuf < pbufend; pbuf++)
    {
      if(cur_hz > target_hz)
        cur_hz -= 10; // TODO do actual calculation using the target delta
      arr = tclk / cur_hz;
      *pbuf = arr;
    }
  }
  else
  {
    for(;pbuf < pbufend; pbuf++)
    {
      if(cur_hz > target_hz)
        cur_hz += 10; // TODO do actual calculation using the target delta
      arr = tclk / cur_hz;
      *pbuf = arr;
    }
  }
  // enable the DMA with the new settings again
  pdma->start();
  // generate an update event to push the values from shadow registers in real registers
  // this is necessary because if the ARR is zero, the timers are stopped and wont generate
  // the event that loads the new ARR value from shadow registers
  _pTimer->EGR |= TIM_EGR_UG;
}



void Timer::configPwm(uint16_t prescaler, uint16_t reload, uint16_t startvalue)
{
  _pTimer->PSC = prescaler; // + 1 internally
  _pTimer->ARR = reload;
  _pTimer->CCR1 = startvalue;
  _pTimer->CCR2 = startvalue;
  _pTimer->CCR3 = startvalue;
  _pTimer->CCR4 = startvalue;
  // setup the ouput compares to PWM Mode 1 with preload
  _pTimer->CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;
  _pTimer->CCMR2 = TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4PE | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3PE;
  // enable output generation of the timer
  _pTimer->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE;
  // generate an update event to push the values from shadow registers in real registers
  _pTimer->EGR |= TIM_EGR_UG;
  // Start the timer
  _pTimer->CR1 = TIM_CR1_CEN;
}

void Timer::configStepper()
{
  _pTimer->CNT = 0;
  _pTimer->PSC = 2; // 72MHz / 3 = 24MHz
  _tclk = SystemCoreClock / (_pTimer->PSC+1);
  _pTimer->ARR = 0;  // on biggest ARR (65535) is just above 366 Hz but, with toggle mode, the actual output is halfed again.
  _pTimer->CCR1 = 0;
  _pTimer->CCR2 = 0;
  _pTimer->CCR3 = 0;
  _pTimer->CCR4 = 0;
  // setup the ouput compares to Toggle Mode
  _pTimer->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_0 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;
  _pTimer->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;
  // enable output generation of the timer
  _pTimer->BDTR = TIM_BDTR_MOE | TIM_BDTR_AOE;
  // set the URS bit to not generate an interrupt when using the event gernation bit
  _pTimer->CR1 = TIM_CR1_URS;
  // generate an update event to push the values from shadow registers in real registers
  _pTimer->EGR |= TIM_EGR_UG;
  // Start the timer, does not run because ARR is 0. ARR buffering enabled to prevent timer running past ARR when adjusting
  _pTimer->CR1 = TIM_CR1_ARPE | TIM_CR1_URS | TIM_CR1_CEN;
}

void Timer::enablePwm(int8_t port, uint8_t pinnum, uint16_t channel, Gpio::Speed speed)
{
  --channel;
  OS_ASSERT(channel < 4, ERR_BAD_INDEX);

  Gpio pin(port, pinnum);
  if(_number < 3)
  {
    pin.mode(Gpio::out_alt_push_pull, speed, Gpio::Timer_1_2);
  }
  else
  {
    pin.mode(Gpio::out_alt_push_pull, speed, Gpio::Timer_3_4_5);
  }
  channel *= 4;
  _pTimer->CCER |= (0x1 << channel);
}

void Timer::enablePwm(syn::Gpio& pin, uint16_t channel, Gpio::Speed speed)
{
  --channel;
  OS_ASSERT(channel < 4, ERR_BAD_INDEX);

  if(_number < 3)
  {
    pin.mode(Gpio::out_alt_push_pull, speed, Gpio::Timer_1_2);
  }
  else
  {
    pin.mode(Gpio::out_alt_push_pull, speed, Gpio::Timer_3_4_5);
  }
  channel *= 4;
  _pTimer->CCER |= (0x1 << channel);
}

void Timer::configInputCapture(uint16_t prescaler, uint16_t reload, Timer::InputFilter filter)
{
  _pTimer->CR1 = 0;
  _pTimer->PSC = prescaler; // + 1 internally
  _pTimer->ARR = reload;
  _pTimer->CCMR1 = (filter << 12) | TIM_CCMR1_CC2S_0 | (filter << 4) | TIM_CCMR1_CC1S_0;
  _pTimer->CCMR2 = (filter << 12) | TIM_CCMR2_CC4S_0 | (filter << 4) | TIM_CCMR2_CC3S_0;
  // start the timer
  _pTimer->CR1 = TIM_CR1_CEN;
}

// configure pwm input capturing
// defaults to ch1 & ch3 capture rising, ch2 & ch4 capture falling
// if mapping is true, instead of capturing on ch1 and ch3 input signals,
// the timer will use ch2 and ch4 as it's inputs
void Timer::configPwmCapture(uint16_t prescaler, uint16_t reload, Timer::InputFilter filter, bool mapping)
{
  _pTimer->CR1 = 0;
  _pTimer->PSC = prescaler; // + 1 internally
  _pTimer->ARR = reload;
  if (mapping)
  {
    _pTimer->CCMR1 = (filter << 12) | TIM_CCMR1_CC2S_0 | (filter << 4) | TIM_CCMR1_CC1S_1;
    _pTimer->CCMR2 = (filter << 12) | TIM_CCMR2_CC4S_0 | (filter << 4) | TIM_CCMR2_CC3S_1;
  }
  else
  {
    _pTimer->CCMR1 = (filter << 12) | TIM_CCMR1_CC2S_1 | (filter << 4) | TIM_CCMR1_CC1S_0;
    _pTimer->CCMR2 = (filter << 12) | TIM_CCMR2_CC4S_1 | (filter << 4) | TIM_CCMR2_CC3S_0;
  }
  _pTimer->CCER = TIM_CCER_CC4P | TIM_CCER_CC4E | TIM_CCER_CC3E | TIM_CCER_CC2P | TIM_CCER_CC2E | TIM_CCER_CC1E;
  // start the timer
  _pTimer->CR1 = TIM_CR1_CEN;
}

// setup pin for input capture
// port shall be 'A' 'B' or 'C'
// pin is a number beteween and including 0 and 15
void Timer::enableInput(int8_t port, uint8_t pinnum, bool pulldown, bool pullup)
{
  Gpio pin(port, pinnum);
  Gpio::Alternate a;
  if(_number < 3)
    a = Gpio::Timer_1_2;
  else
    a = Gpio::Timer_3_4_5;
#ifdef STM32F103xB
  if(pulldown)
  {
    pin.mode(Gpio::in_pulldown, Gpio::Input, a);
  }
  else if(pullup)
  {
    pin.mode(Gpio::in_pullup, Gpio::Input, a);
  }
  else
  {
    pin.mode(Gpio::in_floating, Gpio::Input, a);
  }
#else
  (void)pulldown;
  (void)pullup;
  pin.mode(Gpio::out_alt_push_pull, Gpio::MHz_10, a);
#endif
}

volatile uint16_t *Timer::enableDmaUpdate(uint16_t base_reg, uint16_t burst_count)
{
  OS_ASSERT(base_reg < 19, ERR_BAD_INDEX);
  --burst_count;
  OS_ASSERT(burst_count < 18, ERR_BAD_INDEX);
  OS_ASSERT(base_reg + burst_count < 19, ERR_IMPOSSIBRU);
  _pTimer->DCR = (burst_count << 8) | base_reg;
  _pTimer->DIER |= TIM_DIER_UDE;
  return (volatile uint16_t*)&_pTimer->DMAR;
}

void Timer::stopForDebug()
{
#ifdef DEBUG
#ifdef STM32F103xB
  switch (_number)
  {
  case 1:
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM1_STOP;
    break;
  case 2:
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP;
    break;
  case 3:
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM3_STOP;
    break;
  case 4:
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM4_STOP;
    break;
  }
#endif
#ifdef STM32F401xC
  switch (_number)
  {
  case 1:
    DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP;
    break;
  case 2:
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP;
    break;
  case 3:
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP;
    break;
  case 4:
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP;
    break;
  case 5:
    DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM5_STOP;
    break;
  }
#endif
#endif
}