#include "synos.h"

extern "C" {
  extern void archContextSwitch (syn::Routine* p_old, syn::Routine* p_new);
  extern void archFirstThreadRestore(syn::Routine* p_first);
  extern void synRunFirstTime();
  __task extern void synRunOnMainStack(void* functor, uint8_t* _synos_mainstack);
}

using namespace syn;

Routine Kernel::_routinelist[SYN_OS_ROUTINE_COUNT];
uint8_t Kernel::_current_ticks_left;
uint8_t Kernel::_readycount;

static Routine* _current_routine;
static uint8_t* _mainstack;

#if (SYN_OS_TIMER_COUNT > 0)
SoftTimer SoftTimer::_timerlist[SYN_OS_TIMER_COUNT];
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

void Routine::pause()
{
  Atomic a; // disable interrupts
  _current_routine->_id_runnable &= 0xFE; // clear runnable bit
  --Kernel::_readycount; // decrease ready count
  Kernel::_contextSwitch(); // switch out the routine cooperatively
}

#if (SYN_OS_TIMER_COUNT > 0)
void SoftTimer::_init(SoftTimer::timer_functor_t functor, uint8_t reload)
{
  _next_exec_time = reload;
  _reload = reload;
  _functor = functor;
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
}

void Kernel::spin()
{
  // allocate a local variable to find out the current stack depth
  uint8_t stack_address;
  // reset current count, got counted up by adding routines
  _current_routine = _routinelist;
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
  _current_ticks_left = _current_routine->_rr_reload;
#else
  _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
  _readycount = SYN_OS_ROUTINE_COUNT; // all should be ready
  // setup the idle routine
  // substract some off stackpointer, else no return (we overwrite the return jump..)
  ((Routine*)&_mainstack)->_init((void*)&_idle, 0, &stack_address - 3);
  // clear the event flag to prevent spurious irq
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
  if(_readycount != 0)
  {
    // find the next runnable routine, is not current
    Routine *pold = _current_routine;
    // make sure we can switch out of the idle routine properly
    if(_current_routine == (Routine*)&_mainstack)
      _current_routine = _routinelist - 1;
    while(true)
    {
      // get next routine, roll over if we hit the end
      // this also works of current is the idle routine
      // we are guaranteed to break, because runnable count is not zero
      if(++_current_routine == &_routinelist[SYN_OS_ROUTINE_COUNT])
        _current_routine = _routinelist;
      if(_current_routine->_id_runnable & 0x01)
      {
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
        _current_ticks_left = _current_routine->_rr_reload;
#else
        _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
        archContextSwitch(pold, _current_routine);
        break;
      }
    }
  }
  else if(_current_routine != (Routine*)&_mainstack)
  {
    Routine *pold = _current_routine;
    _current_routine = (Routine*)&_mainstack;
    archContextSwitch(pold, _current_routine);
  }
  // else idle task already running and no runnables, do nothing
}

void Kernel::_tickySwitch()
{
  --_current_ticks_left;
  if(_current_ticks_left == 0)
  {
    if(_readycount == 1 && _current_routine->_id_runnable & 0x01)
    {
      // no switch needed, just refresh this routines tick count
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
      _current_ticks_left = _current_routine->_rr_reload;
#else
      _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
    }
    else
    {
      _contextSwitch();
    }
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
#ifdef SYN_OS_RUN_TIMER_ON_MAINSTACK
  if(_current_routine != (Routine*)&_mainstack)
    synRunOnMainStack((void*)&SoftTimer::_checkAndExec, _mainstack);
  else
    SoftTimer::_checkAndExec();
#else
  SoftTimer::_checkAndExec();
#endif
#endif
  Kernel::_tickySwitch();
}
