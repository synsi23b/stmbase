#include "synos.h"

extern "C" {
  extern void archContextSwitch (syn::Routine* p_old, syn::Routine* p_new);
  extern void synFirstThreadRestore(void(*idle_entry)(void), uint8_t** mainstack, syn::Routine* p_first);
  extern void synRunFirstTime(syn::Routine* p_first);
  extern void synRunOnMainStack(void* functor, uint8_t* mainstack);
}

using namespace syn;

Routine Kernel::_routinelist[SYN_OS_ROUTINE_COUNT];
uint8_t Kernel::_current_ticks_left;
uint8_t Kernel::_readycount;
uint8_t Kernel::_isr_reschedule_request;

static Routine* volatile _current_routine;
static uint8_t* _mainstack;

#if (SYN_OS_TICK_HOOK_COUNT > 0)
SysTickHook SysTickHook::_timerlist[SYN_OS_TICK_HOOK_COUNT];
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

void Routine::yield()
{
  Atomic a;
  yield_protected();
}

void Routine::yield_protected()
{
  Kernel::_contextYield(false);
}

bool Routine::is_runnable() const
{
  return _id_runnable & 0x01;
}

void Routine::block()
{
  _id_runnable &= 0x7E;
  --Kernel::_readycount; // decrease ready count
}

void Routine::unblock()
{
  _id_runnable |= 0x01;
  ++Kernel::_readycount; // increase ready count
}


#if (SYN_OS_TICK_HOOK_COUNT > 0)
void SysTickHook::_init(SysTickHook::timer_functor_t functor, uint8_t reload)
{
  _next_exec_time = reload;
  _reload = reload;
  _functor = functor;
}

void SysTickHook::_checkAndExec()
{
  uint8_t millis = System::millis8();
  SysTickHook *ptim = _timerlist;
  do
  {
    if(millis == ptim->_next_exec_time)
    {
      ptim->_next_exec_time = millis + ptim->_reload;
      assert(ptim->_functor != 0); // to many hooks cofigured in synhal_cfg
      ptim->_functor();
    }
    ++ptim;
  } while(ptim != &_timerlist[SYN_OS_TICK_HOOK_COUNT]);
}
#endif // (SYN_OS_TICK_HOOK_COUNT > 0)

Semaphore::Semaphore(uint8_t start)
{
  _count = start;
}

void Semaphore::give()
{
  Atomic a;
  give_isr();
}

void Semaphore::give_isr()
{
  // already critical section, dont need Atomic
  ++_count;
  assert(_count != 0); // watch for overflow
  // unconditionally unblockrecurse, Kernel will know best what to do
  Kernel::_unblockWaitlist(_waitlist);
}

void Semaphore::get()
{
  Atomic a;
  while(_count == 0)
  {
    Kernel::_enterWaitlist(_waitlist);
  }
  --_count;
}

bool Semaphore::get_isr()
{
  bool ret = false;
  if(_count > 0)
  {
    --_count;
    ret = true;
  }
  return ret;
}

bool Semaphore::try_get()
{
  Atomic a;
  return get_isr();
}

void Kernel::init()
{
  // initialize clocks and peripherals
  System::init();
}

void Kernel::spin()
{
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
  //((Routine*)&_mainstack)->_init((void*)&_idle, 0, &stack_address - 3);
  // clear the event flag to prevent spurious irq
  TIM4->SR1 = 0; 
  // setup idle stack and start first thread
  synFirstThreadRestore(&_idle, &_mainstack, _routinelist);
  while(true != false)
    ;
}

bool Kernel::is_idle()
{
  return _current_routine == (Routine*)&_mainstack;
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
  rim();
  while(1)
    syn_os_idle_hook();
}

void Kernel::exit_isr()
{
  if(_isr_reschedule_request & 0x80)
  {
    // request 0x80 is set when an interrupt enables a routine
    // but since we don't practice eager preemption, this only
    // happens when we are currently idle. and only happens once
    // _current_routine already points to the new routine, so just
    // reschedule (delayed) the whole purpose of this is to not interrupt an isr
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
    _current_ticks_left = _current_routine->_rr_reload;
#else
    _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
    archContextSwitch((Routine*)&_mainstack, _current_routine);
  }
}

void Kernel::_enterWaitlist(Routine*& listhead)
{
  if(listhead == 0)
  {
    listhead = _current_routine;
    _current_routine->_next = 0;
  }
  else
  {
    Routine *tmp = listhead;
    while(tmp->_next != 0)
    {
      tmp = tmp->_next;
    }
    tmp->_next = _current_routine;
    _current_routine->_next = 0;
  }
  _contextYield(true);
}

void Kernel::_unblockWaitlist(Routine*& listhead)
{
  if(listhead != 0)
  {
    Routine *prt = listhead;
    prt->unblock();
    listhead = prt->_next;
    _contextUnblocked(prt);
  }
}

void Kernel::_unblockWaitlistAll(Routine*& listhead)
{
  if(listhead != 0)
  {
    Routine *prt = listhead;
    do
    {
      prt->unblock();
      prt = prt->_next;
    } while (prt != 0);
    prt = listhead;
    listhead = 0;
    _contextUnblocked(prt);
  }
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

// method to use if a routine gives up control by itself
// either through yielding or by some blocking action
// paramter defines wether or not the routine can continue
void Kernel::_contextYield(bool current_is_block)
{
  if(current_is_block)
  {
    _current_routine->block();
    _contextSwitch();
  }
  else if (_readycount != 1)
  {
    // dont switch if this is the only routine runnable anyway
    _contextSwitch();
  }
}

// optimized context switch when we know some routine just got unblocked
void Kernel::_contextUnblocked(Routine* unblocked)
{
  // no aggressive preemption, we only switch the context when idle
  if(is_idle())
  {
    _current_routine = unblocked;
    if(_isr_reschedule_request == 0)
    {
      // not an isr context, so switch right away
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
      _current_ticks_left = _current_routine->_rr_reload;
#else
      _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
      archContextSwitch((Routine*)&_mainstack, _current_routine);
    }
    else
    {
      // set the isr contextswitch pending bit
      // dont care to overwrite the old value, it is forbidden to nest
      _isr_reschedule_request = 0x80;
    }
  }
}

void Kernel::_tickySwitch()
{
  if(_isr_reschedule_request & 0x80)
  {
    // for some reason without this nop the compiler thinks its a cool
    // idea to clear the reschedule_request before even reading it.
    // doesn't matter if its set to volatile or not.
    // so removed volatile and instead, added different clearing scheme
    //nop();
    _isr_reschedule_request = 0;
    // request 0x80 is set when an interrupt enables a routine
    // but since we don't practice eager preemption, this only
    // happens when we are currently idle. and only happens once
    // _current_routine already points to the new routine, so just
    // reschedule (delayed) the whole purpose of this is to not interrupt an isr
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
    _current_ticks_left = _current_routine->_rr_reload;
#else
    _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
    
    archContextSwitch((Routine*)&_mainstack, _current_routine);
  }
  else
  {
    --_isr_reschedule_request;
    --_current_ticks_left;
    if(_current_ticks_left == 0)
    {
      if(_readycount == 1 && _current_routine->is_runnable())
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
}

//INTERRUPT_HANDLER_TRAP(TRAP) {
//  Kernel::_funkySwitch();
//}

INTERRUPT_HANDLER(TIMER4_OV, 23) {
  Kernel::enter_isr();
  System::_systick_isr();
#if (SYN_OS_TICK_HOOK_COUNT > 0)
#ifdef SYN_OS_RUN_TIMER_ON_MAINSTACK
  if(Kernel::is_idle())
    SysTickHook::_checkAndExec();
  else
    synRunOnMainStack((void*)&SysTickHook::_checkAndExec, _mainstack);
#else
  SysTickHook::_checkAndExec();
#endif
#endif
  Kernel::_tickySwitch();
}
