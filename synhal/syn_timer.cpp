#include "synhal.h"
using namespace syn;
#if(SYN_ENABLE_TIMER_IRQ != 0)
void Timer::enableCallback(uint32_t priority)
{
#if(SYN_TIMER_1_IRQ_TYPE != 0)
  if (_pTimer == TIM1)
  {
    Atomic a;
    _pTimer->DIER = TIM_DIER_UIE;
    _pTimer->SR = 0;
    Core::enable_isr(TIM1_UP_IRQn, priority);
    return;
  }
#endif
#if(SYN_TIMER_2_IRQ_TYPE != 0)
  if (_pTimer == TIM2)
  {
    Atomic a;
    _pTimer->DIER = TIM_DIER_UIE;
    _pTimer->SR = 0;
    Core::enable_isr(TIM2_IRQn, priority);
    return;
  }
#endif
#if(SYN_TIMER_3_IRQ_TYPE != 0)
  if (_pTimer == TIM3)
  {
    Atomic a;
    _pTimer->DIER = TIM_DIER_UIE;
    _pTimer->SR = 0;
    Core::enable_isr(TIM3_IRQn, priority);
    return;
  }
#endif
#if(SYN_TIMER_4_IRQ_TYPE != 0)
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
#if(SYN_TIMER_1_IRQ_TYPE == 1)
void TIM1_UP_IRQHandler()
{
  Core::enter_isr();
  syn_timer_1_isr();
  Core::leave_isr();
}
#elif(SYN_TIMER_1_IRQ_TYPE == 2)
void TIM1_UP_IRQHandler()
{
  syn_timer_1_isr();
}
#endif
#if(SYN_TIMER_2_IRQ_TYPE == 1)
void TIM2_IRQHandler()
{
  Core::enter_isr();
  syn_timer_2_isr();
  Core::leave_isr();
}
#elif(SYN_TIMER_2_IRQ_TYPE == 2)
void TIM2_IRQHandler()
{
  syn_timer_2_isr();
}
#endif
#if(SYN_TIMER_3_IRQ_TYPE == 1)
void TIM3_IRQHandler()
{
  Core::enter_isr();
  syn_timer_3_isr();
  Core::leave_isr();
}
#elif(SYN_TIMER_3_IRQ_TYPE == 2)
void TIM3_IRQHandler()
{
  syn_timer_3_isr();
}
#endif
#if(SYN_TIMER_4_IRQ_TYPE == 1)
void TIM4_IRQHandler()
{
  Core::enter_isr();
  syn_timer_4_isr();
  Core::leave_isr();
}
#elif(SYN_TIMER_4_IRQ_TYPE == 2)
void TIM4_IRQHandler()
{
  syn_timer_4_isr();
}
#endif // enable_timer_irq
} // extern c
#endif // if(SYN_ENABLE_TIMER_CALLBACK == 1)

uint16_t Timer::getTimernum()
{
  if (_pTimer == TIM2)
    return 1;
  if (_pTimer == TIM3)
    return 2;
  if (_pTimer == TIM4)
    return 3;
  return 0;
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

void Timer::enablePwm(uint16_t channel, Gpio::Speed speed)
{
  --channel;
  while (channel > 3)
    ;

  int8_t port = 'A';
  uint8_t pinnum = 0;
  uint16_t timernum = getTimernum();
  switch (timernum)
  {
  case 0:
    port = 'A';
    pinnum = channel + 8;
    break;
  case 1:
    port = 'A';
    pinnum = channel;
    break;
  case 2:
    port = 'B';
    if (channel < 2)
      pinnum = channel + 4;
    else
      pinnum = channel - 2;
    break;
  case 3:
    port = 'B';
    pinnum = channel + 6;
    break;
  }

  Gpio pin(port, pinnum);
  pin.mode(Gpio::out_alt_push_pull, speed);

  channel *= 4;
  _pTimer->CCER |= (0x1 << channel);
}

void Timer::configInputCapture(uint16_t prescaler, uint16_t reload, Timer::InputFilter filter)
{
  _pTimer->PSC = prescaler; // + 1 internally
  _pTimer->ARR = reload;
  _pTimer->CCMR1 = (filter << 12) | TIM_CCMR1_CC2S_0 | (filter << 4) | TIM_CCMR1_CC1S_0;
  _pTimer->CCMR2 = (filter << 12) | TIM_CCMR2_CC4S_0 | (filter << 4) | TIM_CCMR2_CC3S_0;
  // start the timer
  _pTimer->CR1 = TIM_CR1_CEN;
}

void Timer::enableInput(uint16_t channel, bool rising_edge, bool pulldown)
{
  --channel;
  OS_ASSERT(channel < 4, ERR_BAD_INDEX);
  int8_t port = 'A';
  uint8_t pinnum = 0;
  uint16_t timernum = getTimernum();
  switch (timernum)
  {
  case 0:
    port = 'A';
    pinnum = channel + 8;
    break;
  case 1:
    port = 'A';
    pinnum = channel;
    break;
  case 2:
    port = 'B';
    if (channel < 2)
      pinnum = channel + 4;
    else
      pinnum = channel - 2;
    break;
  case 3:
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
