#include "synos.h"

using namespace syn;

Routine* Kernel::_current;
Routine Kernel::_routinelist[SYN_OS_ROUTINE_COUNT];
Routine Kernel::_idler;
uint8_t Kernel::_current_ticks_left;
uint8_t Kernel::_readycount;

#if (SYN_OS_TIMER_COUNT > 0)
static SoftTimer _timerlist[SYN_OS_TIMER_COUNT];
#endif

void Routine::_init(void* functor, void* arg, uint8_t* stack)
{
  // add the functor and argument to the stack
  // they will be retrieved on first run by synFirstTime
  *stack-- = (uint8_t)((uint16_t)functor & 0xFF);
  *stack-- = (uint8_t)(((uint16_t)functor >> 8) & 0xFF);
  *stack-- = (uint8_t)((uint16_t)arg & 0xFF);
  *stack-- = (uint8_t)(((uint16_t)arg >> 8) & 0xFF);
  /**
   * The thread restore routines will perform a RET which expects to
   * find the address of the calling routine on the stack. In this case
   * (the first time a thread is run) we "return" to the entry point for
   * the thread. That is, we store the thread entry point in the
   * place that RET will look for the return address: the stack.
   *
   * Note that we are using the thread_shell() routine to start all
   * threads, so we actually store the address of thread_shell()
   * here. Other ports may store the real thread entry point here
   * and call it directly from the thread restore routines.
   *
   * Because we are filling the stack from top to bottom, this goes
   * on the stack first (at the top).
   */
  *stack-- = (uint8_t)((uint16_t)synRunFirstTime & 0xFF);
  *stack-- = (uint8_t)(((uint16_t)synRunFirstTime >> 8) & 0xFF);
  /**
   * Because we are using a thread shell which is responsible for
   * calling the real entry point, it also passes the parameters
   * to entry point and we need not stack the entry parameter here.
   *
   * Other ports may wish to store entry_param in the appropriate
   * parameter registers when creating a thread's context,
   * particularly if that port saves those registers anyway.
   */
  /**
   * (IAR) Set up initial values for ?b8 to ?b15.
  */
  *stack-- = 0;    // ?b8
  *stack-- = 0;    // ?b9
  *stack-- = 0;    // ?b10
  *stack-- = 0;    // ?b11
  *stack-- = 0;    // ?b12
  *stack-- = 0;    // ?b13
  *stack-- = 0;    // ?b14
  *stack-- = 0;    // ?b15
  _stackptr = stack;
}

#if (SYN_OS_TIMER_COUNT > 0)
void SoftTimer::init(SoftTimer::timer_functor functor, uint8_t reload)
{
  assert(reload > 0);
  SoftTimer* ptimer = _timerlist;
  for(;;++ptimer)
  {
    assert(ptimer != &_timerlist[SYN_OS_TIMER_COUNT]);
    if(ptimer->_next_exec_time == 0)
    {
      ptimer->_next_exec_time = reload;
      ptimer->_reload = reload;
      ptimer->_functor = functor;
      break;
    }
  }
}

void SoftTimer::_checkAndExec()
{
  uint8_t millis = System::millis8();
  SoftTimer *ptim = _timerlist;
  do
  {
    if(millis == ptim->_next_exec_time)
    {
      ptim->_next_exec_time = millis + ptim->_reload;
      ptim->_functor();
    }
    ++ptim;
  } while(ptim != &_timerlist[SYN_OS_TIMER_COUNT]);
}
#endif // (SYN_OS_TIMER_COUNT > 0)

void Kernel::init()
{
  // initialize clocks and peripherals
  System::init();
  // setup the current active routine counter to be used by Routine::init
  _current = _routinelist;
}

void Kernel::spin()
{
  // allocate a local variable to find out the current stack depth
  uint8_t stack_address;
  // reset current count, got counted up by adding routines
  _current = _routinelist;
  _current_ticks_left = _current->_state_1;
  _readycount = SYN_OS_ROUTINE_COUNT; // all should be ready
  // setup the idle routine
  // substract some off stackpointer, else no return (we overwrite the return jump..)
  _idler._init((void*)&_idle, 0, &stack_address - 3);
  // but clear the event flag to prevent spurious irq
  TIM4->SR1 = 0; 
  // restore first Thread, should never return
  archFirstThreadRestore(_routinelist);
}

// transform stack point to point at the top
// add known bytes if stack check is enabled
uint8_t* Kernel::_base_stack_setup(uint8_t* stack, uint16_t size)
{
  uint8_t* stacktop = stack + (size - 1);
#ifdef SYN_OS_STACK_CHECK
  size -= 10;
  while(size != 0)
  {
    *stack++ = 0xCD;
    --size;
  }
#endif
  return stacktop;
}

void Kernel::_idle()
{
  // TODO dont just idle, run Timers
  uint16_t x;
  while(1)
    ++x;
  //wfi();
}

void Kernel::_contextSwitch()
{
  Routine *pold = _current++;
  if(_current == &_routinelist[SYN_OS_ROUTINE_COUNT])
    _current = _routinelist;
  _current_ticks_left = _current->_state_1;
  archContextSwitch(pold, _current);
}

void Kernel::_tickySwitch()
{
  --_current_ticks_left;
  if(_current_ticks_left == 0)
  {
    _contextSwitch();
  }
}

void Kernel::_funkySwitch()
{
  _contextSwitch();
}

INTERRUPT_HANDLER_TRAP(TRAP) {
  Kernel::_funkySwitch();
}

INTERRUPT_HANDLER(TIMER4_OV, 23) {
  System::_systick_isr();
#if (SYN_OS_TIMER_COUNT > 0)
  SoftTimer::_checkAndExec();
#endif
  Kernel::_tickySwitch();
}
