#include "synhal.h"
using namespace syn;
#if (SYN_ENABLE_TIMER_IRQ != 0)
void Timer::enableCallback(uint16_t priority)
{
#if (SYN_TIMER_1_IRQ_TYPE != 0)
  if (_pTimer == TIM1)
  {
    Atomic a;
    _pTimer->DIER = TIM_DIER_UIE;
    _pTimer->SR = 0;
    Core::enable_isr(TIM1_UP_IRQn, priority);
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
}
#include "../../synhal_isr.h"
extern "C"
{
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

void Timer::enablePwm(uint16_t channel, Gpio::Speed speed)
{
  --channel;
  OS_ASSERT(channel < 4, ERR_BAD_INDEX);

  int8_t port = 'A';
  uint8_t pinnum = 0;
  switch (_number)
  {
  case 1:
    port = 'A';
    pinnum = channel + 8;
    break;
  case 2:
    port = 'A';
    pinnum = channel;
    break;
  case 3:
    port = 'B';
    if (channel < 2)
      pinnum = channel + 4;
    else
      pinnum = channel - 2;
    break;
  case 4:
    port = 'B';
    pinnum = channel + 6;
    break;
  case 5:
    OS_ASSERT(true == false, ERR_DEVICE_NOT_ENABLED);
  }

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

void Timer::enableInput(uint16_t channel, bool rising_edge, bool falling_edge)
{
  --channel;
  OS_ASSERT(channel < 4, ERR_BAD_INDEX);
  int8_t port = 'A';
  uint8_t pinnum = 0;
  switch (_number)
  {
  case 1:
    port = 'A';
    pinnum = channel + 8;
    break;
  case 2:
    port = 'A';
    pinnum = channel;
    break;
  case 3:
    port = 'B';
    if (channel < 2)
      pinnum = channel + 4;
    else
      pinnum = channel - 2;
    break;
  case 4:
    port = 'B';
    pinnum = channel + 6;
    break;
  }

  Gpio pin(port, pinnum);
  pin.mode(Gpio::in_pullup_pulldown, Gpio::MHz_10);
  if (pulldown)
    pin.clear();
  else
    pin.set();

  channel *= 4;
  if (!rising_edge)
    _pTimer->CCER |= (0x1 << channel);
  else
    _pTimer->CCER |= (0x3 << channel);
}

// setup pin for input capture
// port shall be 'A' 'B' or 'C'
// pin is a number beteween and including 0 and 15
void Timer::enableInputRaw(int8_t port, uint8_t pinnum, bool pulldown)
{
  Gpio pin(port, pinnum);
  uint16_t timnum = getTimernum();
  if(pulldown)
  {
    pin.mode(Gpio::in_pullup_pulldown, Gpio::Input, Gpio::Timer_1_2);
  }
  pin.mode(Gpio::in_pullup_pulldown, Gpio::MHz_10);
  if (pulldown)
    pin.clear();
  else
    pin.set();
}

volatile uint16_t *Timer::enableDmaUpdate(uint16_t base_reg, uint16_t burst_count)
{
  OS_ASSERT(base_reg < 19, ERR_BAD_INDEX);
  --burst_count;
  OS_ASSERT(burst_count < 18, ERR_BAD_INDEX);
  OS_ASSERT(base_reg + burst_count < 19, ERR_IMPOSSIBRU);
  _pTimer->DCR = (burst_count << 8) | base_reg;
  _pTimer->DIER |= TIM_DIER_UDE;
  return &_pTimer->DMAR;
}

void Timer::stopForDebug()
{
#ifdef DEBUG
  switch (getTimernum())
  {
  case 0:
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM1_STOP;
    break;
  case 1:
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP;
    break;
  case 2:
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM3_STOP;
    break;
  case 3:
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM4_STOP;
    break;
  case 4:
    DBGMCU->CR |= DBGMCU_CR_DBG_TIM5_STOP;
    break;
  }
#endif
}