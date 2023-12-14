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

void Timer::init(uint16_t number)
{
  _tclk = 0;
  _number = number;
  switch (number)
  {
#ifdef STM32F103xB
  case 1:
    _pTimer = TIM1;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    break;
  case 2:
    _pTimer = TIM2;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    break;
  case 3:
    _pTimer = TIM3;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    break;
  case 4:
    _pTimer = TIM4;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    break;
#endif
#ifdef STM32F401xC
  case 1:
    _pTimer = TIM1;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    break;
  case 2:
    _pTimer = TIM2;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    break;
  case 3:
    _pTimer = TIM3;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    break;
  case 4:
    _pTimer = TIM4;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    break;

  case 5:
    _pTimer = TIM5;
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    break;
#endif
  default:
    OS_ASSERT(true == false, ERR_BAD_PORT_NAME);
  }
}

uint16_t Timer::dma_channel() const
{
  uint16_t num = 0;
  switch (_number)
  {
  case 1:
    num = 5;
    break;
  case 2:
    num = 2;
    break;
  case 3:
    num = 3;
    break;
  case 4:
    num = 7;
    break;
  default:
    break;
  }
  return num;
}

void TimerRamper::init(uint16_t timer_num, uint16_t buffsize)
{
  _tim.init(timer_num);
  _tim.configStepper();
  buffsize = buffsize & 0xFFF0; // clear lower 4 bits, force multiple of 16 as buffsize
  if(buffsize == 0)
    buffsize = 16;
  _buffer = new uint16_t[buffsize];
  OS_ASSERT(_buffer != NULL, ERR_NULL_POINTER);
  if (_buffer != NULL)
  {
    _dma.init(_tim.dma_channel());
    _buffsize = buffsize;
    // enable DMA for register with offset 11 (ARR) 1 transfer per DMA request
    auto *target_reg = _tim.enableDmaUpdate(11, 1);
    // setup DMA mode to keep reading the same memory until stopped
    _dma.cyclicM2P(_buffer, target_reg, buffsize);
    // enable both DMA interrupts to rewrite half of the memory buffer in the isr
    _dma.enableIrq(Dma::IRQ_STATUS_FULL | Dma::IRQ_STATUS_HALF);
    _minspeed = _tim.hertz_to_arr(10240);
  }
}

void TimerRamper::linear(uint32_t target_hz)
{
  uint16_t target = _tim.hertz_to_arr(target_hz);
  if (_tim.arr() == target)
    return;
  
  if (_buffer == NULL)
    return;
  _dma.reset(_buffsize);

  _target = target;
  // call write dma buffer without any IRQ flag to trigger full rewrite
  _write_buffer(0);

  // enable the DMA with the new settings again
  _dma.start();
  // write the first element into ARR register to get things going in case of Stepper is at 0 Hz
  *_tim.get_arr_register() = _buffer[0];
  // trigger the timer to start the first dma transfer
  _tim.generate_update_event();
}

void TimerRamper::_write_buffer(uint16_t irq_stat)
{
  // prepare the first buffer, after that, DMA irqs will do this until target is reached
  uint16_t *pbuf = _buffer;
  uint16_t *pbufend = pbuf + _buffsize;
  uint32_t arr_start;
  if (irq_stat == Dma::IRQ_STATUS_HALF)
  {
    // on half transfer irq, rewrite the first half of the dma input
    arr_start = *(pbufend - 1);
    pbufend = pbuf + _buffsize / 2;
  }
  else if (irq_stat == Dma::IRQ_STATUS_FULL)
  {
    // on full transfer irq, rewrite the second half of the dma input
    pbuf = _buffer + _buffsize / 2;
    arr_start = *(pbuf - 1);
  }
  else
  {
    // call outside of irq to start everything
    arr_start = _tim.arr();
    if (arr_start < _minspeed)
    {
      arr_start = _minspeed;
    }
  }

  if (arr_start < _target)
  {
    for (; pbuf < pbufend;)
    {
      if (arr_start < uint32_t(_target))
      {
        arr_start += 1;
      }
      *pbuf++ = arr_start;
      *pbuf++ = arr_start;
      //*pbuf++ = arr_start;
      //*pbuf++ = arr_start;
    }
  }
  else //if (arr_start > _target)
  {
    for (; pbuf < pbufend;)
    {
      if(arr_start != _target)
      {
        arr_start -= 1;
        if(arr_start < _minspeed)
        {
          arr_start = _target;
        }
      }
      *pbuf++ = arr_start;
      *pbuf++ = arr_start;
      //*pbuf++ = arr_start;
      //*pbuf++ = arr_start;
    }
  }
}

void TimerRamper::isr(uint32_t status)
{
  if (target_reached())
  {
    _dma.stop();
  }
  else
  {
    _write_buffer(status);
  }
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
  _pTimer->PSC = 0; // 72MHz / 3 = 24MHz
  _tclk = SystemCoreClock; /// (_pTimer->PSC + 1);
  _pTimer->ARR = 0; // on biggest ARR (65535) is just above 366 Hz but, with toggle mode, the actual output is halfed again.
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
  Gpio pin(port, pinnum);
  enablePwm(pin, channel, speed);
}

void Timer::enablePwm(syn::Gpio &pin, uint16_t channel, Gpio::Speed speed)
{
  --channel;
  OS_ASSERT(channel < 4, ERR_BAD_INDEX);

  if (_number < 3)
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
  if (_number < 3)
    a = Gpio::Timer_1_2;
  else
    a = Gpio::Timer_3_4_5;
#ifdef STM32F103xB
  if (pulldown)
  {
    pin.mode(Gpio::in_pulldown, Gpio::Input, a);
  }
  else if (pullup)
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
  _pTimer->DIER |= TIM_DIER_UDE | TIM_DIER_TDE;
  return (volatile uint16_t *)&_pTimer->DMAR;
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