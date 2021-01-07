#include "synos.h"

extern "C"
{
  extern void archContextSwitch(syn::Routine *p_old, syn::Routine *p_new);
  extern void synFirstThreadRestore(void (*idle_entry)(void), uint8_t **mainstack, syn::Routine *p_first);
  extern void synRunFirstTime(syn::Routine *p_first);
  extern void synRunOnMainStack(void *functor, uint8_t *mainstack);
}

using namespace syn;

Routine Kernel::_routinelist[SYN_OS_ROUTINE_COUNT];
#if (SYN_OS_ROUTINE_RR_SLICE_MS != 0)
uint8_t Kernel::_current_ticks_left;
#endif
uint8_t Kernel::_readycount;
uint8_t Kernel::_isr_reschedule_request;
uint8_t *_mainstack;

static Routine *volatile _current_routine;

#ifdef SYN_OS_MEASURE_INTERNALS
static syn::Gpio _measure_pin;
#endif

Routine *Kernel::_fake_idle_routine()
{
  return (Routine *)(((uint8_t *)&_mainstack) - 1);
}

template <typename Functor>
void Kernel::for_each_routine(Functor functor)
{
  Routine *pr = _routinelist;
  while (pr != &_routinelist[SYN_OS_ROUTINE_COUNT])
  {
    functor(pr);
    ++pr;
  }
}

void Routine::init(void *functor, uint16_t arg, uint16_t stacksize)
{
  // Routine Index out of bounds! Increase the ammount specified in the config
  //static_assert(Index < SYN_OS_ROUTINE_COUNT, "Routine Index out of bounds! Increase the ammount specified in the config");
#ifdef DEBUG
  // Routine Index out of bounds! Increase the ammount specified in the config
  while (Kernel::_readycount >= SYN_OS_ROUTINE_COUNT)
    ;
  // stack is crossing stackpointer roll over address !!!
  while (uint16_t(Kernel::_mainstack - stacksize) < 0x1FF)
    ;
#endif
  uint8_t *stack = Kernel::_base_stack_setup(stacksize);
  Routine *prout = Kernel::_routinelist + Kernel::_readycount;
  prout->_state = runnable;
  prout->_init(functor, (void *)arg, stack);
}

#ifdef SYN_OS_ENABLE_TIMEOUT_API
void Routine::sleep(uint16_t timeout)
{
  Atomic a; // have to protect the call to _setupTimeout
  _current_routine->_setupTimeout(timeout, 0);
  Kernel::_contextYield(sleeping);
#ifdef SYN_OS_MEASURE_SYSTICK
  _measure_pin.set();
#endif
}

void Routine::_setupTimeout(uint16_t timeout_ms, Routine **waitlist)
{
  if(timeout_ms < SYN_SYSTICK_FREQ)
  {
    timeout_ms = SYN_SYSTICK_FREQ;
  }
#if (SYN_SYSTICK_FREQ == 2)
  // since we test the timneout by equality for speed, we have to make sure only
  // even numbers are used, since millis is incremented by 2, not 1
  // so clear the leas significant bit
  timeout_ms = timeout_ms & 0xFFFE;
#endif
  _timeout = timeout_ms + System::millis();
  if (_timeout == 0)
  {
    // protect from roll over problem.
    // a timeout value of 0 has a different interpretition.
    // t signales to the routine that the action it did timed out.
    _timeout = SYN_SYSTICK_FREQ;
  }
  _waitlist = waitlist;
}

bool Routine::_timeout_is_expired() const
{
#ifdef SYN_OS_MEASURE_SYSTICK
  // this always gets called after a timeoutable action
  // so set the _measure pin high to make sure we can see the event on the line
  _measure_pin.set();
#endif
  return _timeout == 0;
}

// functor constructor -> store current millis.
// we execute this in the systick itself, but the system Millis is volatile.
// store a copy just to improve the compilers handling of that (maybe)
Routine::TestTimoutExpired::TestTimoutExpired()
{
  _current_millis = System::millis();
}

void Routine::TestTimoutExpired::operator()(Routine *pr)
{
  if (pr->_state & timeout)
  {
    if (pr->_timeout == _current_millis)
    {
      // unblocks the routine
      if (Kernel::_removeWaitlist(pr->_waitlist, pr))
      {
        // is expired, set to zero to notify user code
        pr->_timeout = 0;
      }
      // unblock the routines context, wether sleeping or waiting
      Kernel::_contextUnblocked(pr);
    }
  }
}

void Routine::timeouttick()
{
  TestTimoutExpired fn;
  Kernel::for_each_routine(fn);
}
#endif

#if (SYN_OS_TICK_HOOK_COUNT > 0)
SysTickHook SysTickHook::_timerlist[SYN_OS_TICK_HOOK_COUNT];
#endif

void Routine::_init(void *functor, void *arg, uint8_t *stack)
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
  *stack-- = 0; // ?b8
  *stack-- = 0; // ?b9
  *stack-- = 0; // ?b10
  *stack-- = 0; // ?b11
  *stack-- = 0; // ?b12
  *stack-- = 0; // ?b13
  *stack-- = 0; // ?b14
  *stack-- = 0; // ?b15
  _stackptr = stack;
  ++Kernel::_readycount;
}

void Routine::yield()
{
  Atomic a;
  yield_protected();
}

void Routine::yield_protected()
{
  Kernel::_contextYield(runnable);
}

bool Routine::is_runnable() const
{
  return _state == runnable;
}

void Routine::block(State blocking_reason)
{
  _state = blocking_reason;
  --Kernel::_readycount; // decrease ready count
}

void Routine::unblock()
{
  _state = runnable;
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
    if (millis == ptim->_next_exec_time)
    {
      ptim->_next_exec_time = millis + ptim->_reload;
      assert(ptim->_functor != 0); // to many hooks cofigured in synhal_cfg
      ptim->_functor();
    }
    ++ptim;
  } while (ptim != &_timerlist[SYN_OS_TICK_HOOK_COUNT]);
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
  while (_count == 0)
  {
    Kernel::_enterWaitlist(_waitlist, Routine::wait_semaphore);
  }
  --_count;
}

#ifdef SYN_OS_ENABLE_TIMEOUT_API
bool Semaphore::get(uint16_t timeout)
{
  Atomic a;
  if (_count == 0)
  {
    _current_routine->_setupTimeout(timeout, &_waitlist);
    while (_count == 0)
    {
      Kernel::_enterWaitlist(_waitlist, Routine::timeout_semaphore);
      if (_current_routine->_timeout_is_expired())
      {
        return false;
      }
    }
  }
  --_count;
  return true;
}
#endif

bool Semaphore::get_isr()
{
  bool ret = false;
  if (_count > 0)
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

void Mutex::lock()
{
  Atomic a;
  while (_owner != _current_routine)
  {
    if (_owner == 0)
    {
      _owner = _current_routine;
    }
    else
    {
      Kernel::_enterWaitlist(_waitlist, Routine::wait_mutex);
    }
  }
  ++_count;
  // watch for mutex overlock
  assert(_count != 0);
}

#ifdef SYN_OS_ENABLE_TIMEOUT_API
bool Mutex::lock(uint16_t timeout)
{
  bool ret = false;
  Atomic a;
  if (_owner == 0)
  {
    _owner = _current_routine;
  }
  if (_owner == _current_routine)
  {
    ++_count;
    // watch for mutex overlock
    assert(_count != 0);
    ret = true;
  }
  else
  {
    _current_routine->_setupTimeout(timeout, &_waitlist);
    while (_owner != _current_routine)
    {
      Kernel::_enterWaitlist(_waitlist, Routine::timeout_mutex);
      if (_current_routine->_timeout_is_expired())
      {
        break;
      }
      if (_owner == 0)
      {
        _owner = _current_routine;
        ++_count;
        ret = true;
      }
    }
  }
  return ret;
}
#endif

bool Mutex::try_lock()
{
  Atomic a;
  bool ret = false;
  if (_owner == 0)
  {
    _owner = _current_routine;
  }
  if (_owner == _current_routine)
  {
    ++_count;
    // watch for mutex overlock
    assert(_count != 0);
    ret = true;
  }
  return ret;
}

void Mutex::unlock()
{
  Atomic a;
  assert(_owner == _current_routine);
  assert(_count != 0);
  if (--_count == 0)
  {
    _owner = 0;
    Kernel::_unblockWaitlist(_waitlist);
  }
}

#ifdef SYN_OS_USE_EVENTS
// used by events, is private
void Routine::_setEventValue(uint8_t value)
{
  _eventval = value;
}
// set the regardless of its state from an isr
// value of zero is forbidden / is the same as clearing the event
void Event::set_isr(uint8_t value)
{
  _value = value;
  Kernel::_unblockEventlist(_waitlist, value);
}

// set the event regardless of its state
// value of zero is forbidden / is the same as clearing the event
void Event::set(uint8_t value)
{
  Atomic a;
  set_isr(value);
}
// set the event regardless of its state from an isr
// return true if the event was already set
// value of zero is forbidden / is the same as clearing the event
bool Event::set_check_isr(uint8_t value)
{
  bool ret = _value != 0;
  set_isr(value);
  return ret;
}
// set the event regardless of its state
// return true if the event was already set
// value of zero is forbidden / is the same as clearing the event
bool Event::set_check(uint8_t value)
{
  Atomic a;
  return set_check_isr(value);
}

// wait for the event to be set and return its value
// doesn't clear the event value
uint8_t Event::wait()
{
  Atomic a;
  if (_value == 0)
  {
    // not looping forever because if the event is set
    // any waiting routine will receive the value by the kernel
    Kernel::_enterWaitlist(_waitlist, Routine::wait_event);
  }
  else
  {
    _current_routine->_setEventValue(_value);
  }
  return _current_routine->lastEvent();
}


// wait for the event to be set and return its value
// doesn't clear the event value
// returns 0 if timeout
#ifdef SYN_OS_ENABLE_TIMEOUT_API
uint8_t Event::wait(uint16_t timeout)
{
  Atomic a;
  if (_value == 0)
  {
    _current_routine->_setupTimeout(timeout, &_waitlist);
    Kernel::_enterWaitlist(_waitlist, Routine::timeout_event);
    if (_current_routine->_timeout_is_expired())
    {
      _current_routine->_setEventValue(0);
    }
  }
  else
  {
    _current_routine->_setEventValue(_value);
  }
  return _current_routine->lastEvent();
}
#endif

void Kernel::_unblockEventlist(Routine *&listhead, uint8_t value)
{
  if (listhead != 0)
  {
    Routine *prt = listhead;
    do
    {
      prt->_setEventValue(value);
      prt->unblock();
      prt = prt->_next;
    } while (prt != 0);
    prt = listhead;
    listhead = 0;
    _contextUnblocked(prt);
  }
}
#endif

void Kernel::init()
{
  // initialize clocks and peripherals
  System::init();
#ifdef SYN_OS_MEASURE_INTERNALS
  _measure_pin.init('A', 1);
  _measure_pin.mode(true, true);
  _measure_pin.set();
#endif
  // used to select correct index in _routinelist during initialization
  _readycount = 0;
  // use this variable to initialize co routines
  _mainstack = (uint8_t *)(0x3FF - SYN_OS_MAIN_STACK_SIZE);
}

void Kernel::spin()
{
  // reset current count, got counted up by adding routines
  _current_routine = _routinelist;
#if (SYN_OS_ROUTINE_RR_SLICE_MS != 0)
  _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
#ifdef DEBUG
  while (_readycount != SYN_OS_ROUTINE_COUNT)
    ; // routinecount has to exactly match the ammount of initialized routines
#endif
  // setup the idle routine
  // substract some off stackpointer, else no return (we overwrite the return jump..)
  //((Routine*)&_mainstack)->_init((void*)&_idle, 0, &stack_address - 3);
  // clear the event flag to prevent spurious irq
  TIM4->SR1 = 0;
  // setup idle stack and start first thread
  synFirstThreadRestore(&_idle, &_mainstack, _routinelist);
  // will not return, but hint it to the compiler, too
  while (true != false)
    ;
}

bool Kernel::is_idle()
{
  return _current_routine == _fake_idle_routine();
}

// transform stack point to point at the top
// add known bytes if stack check is enabled
uint8_t *Kernel::_base_stack_setup(uint16_t size)
{
  uint8_t *stacktop = _mainstack;
  _mainstack -= size;
#ifdef SYN_OS_STACK_CHECK
  uint8_t *stackbot = _mainstack + 1;
  uint8_t initval = 0xC0 + _readycount;
  while (size != 0)
  {
    *stackbot++ = initval;
    --size;
  }
#endif
  return stacktop;
}

void Kernel::_idle()
{
  rim();
  while (1)
    syn_os_idle_hook();
}

void Kernel::exit_isr()
{
  if (_isr_reschedule_request & 0x80)
  {
    // request 0x80 is set when an interrupt enables a routine
    // but since we don't practice eager preemption, this only
    // happens when we are currently idle. and only happens once
    // _current_routine already points to the new routine, so just
    // reschedule (delayed) the whole purpose of this is to not interrupt an isr
#if (SYN_OS_ROUTINE_RR_SLICE_MS != 0)
    _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
    archContextSwitch(_fake_idle_routine(), _current_routine);
  }
}

void Kernel::_enterWaitlist(Routine *&listhead, Routine::State blocking_reason)
{
  if (listhead == 0)
  {
    listhead = _current_routine;
    _current_routine->_next = 0;
  }
  else
  {
    Routine *tmp = listhead;
    while (tmp->_next != 0)
    {
      tmp = tmp->_next;
    }
    tmp->_next = _current_routine;
    _current_routine->_next = 0;
  }
  _contextYield(blocking_reason);
}

void Kernel::_unblockWaitlist(Routine *&listhead)
{
  if (listhead != 0)
  {
    Routine *prt = listhead;
    prt->unblock();
    listhead = prt->_next;
    _contextUnblocked(prt);
  }
}

bool Kernel::_removeWaitlist(Routine **listhead, Routine *to_remove)
{
  // set the return value to true if the routine timed out during whatever it waited for
  bool ret = false;
  to_remove->unblock();
  // check if there is no list, maybe we don't have to rebuild it
  if (listhead != 0)
  {
    Routine *pcur = *listhead;
    if (pcur == to_remove)
    {
      // the element to remove is the first of the list, very easy
      *listhead = to_remove->_next;
      ret = true;
    }
    else
    {
      while (pcur != 0)
      {
        // the element is not the first, and the list is not empty
        if (pcur->_next == to_remove)
        {
          pcur->_next = to_remove->_next;
          ret = true;
          break;
        }
        pcur = pcur->_next;
      }
    }
  }
  return ret;
}

void Kernel::_contextSwitch()
{
  if (_readycount != 0)
  {
    // find the next runnable routine, is not current
    Routine *pold = _current_routine;
    // make sure we can switch out of the idle routine properly
    if (_current_routine == _fake_idle_routine())
      _current_routine = _routinelist - 1;
    while (true)
    {
      // get next routine, roll over if we hit the end
      // this also works of current is the idle routine
      // we are guaranteed to break, because runnable count is not zero
      if (++_current_routine == &_routinelist[SYN_OS_ROUTINE_COUNT])
        _current_routine = _routinelist;
      if (_current_routine->is_runnable())
      {
#if (SYN_OS_ROUTINE_RR_SLICE_MS != 0)
        _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
        archContextSwitch(pold, _current_routine);
        break;
      }
    }
  }
  else if (_current_routine != _fake_idle_routine())
  {
    Routine *pold = _current_routine;
    _current_routine = _fake_idle_routine();
    archContextSwitch(pold, _current_routine);
  }
  // else idle task already running and no runnables, do nothing
}

// method to use if a routine gives up control by itself
// either through yielding or by some blocking action
// paramter defines wether or not the routine can continue
void Kernel::_contextYield(Routine::State blocking_reason)
{
  if (blocking_reason != 0)
  {
    _current_routine->block(blocking_reason);
    _contextSwitch();
  }
  else if (_readycount != 1)
  {
    // dont switch if this is the only routine runnable anyway
    _contextSwitch();
  }
}

// optimized context switch when we know some routine just got unblocked
void Kernel::_contextUnblocked(Routine *unblocked)
{
  // no aggressive preemption, we only switch the context when idle
  if (is_idle())
  {
    _current_routine = unblocked;
    if (_isr_reschedule_request == 0)
    {
      // not an isr context, so switch right away
#if (SYN_OS_ROUTINE_RR_SLICE_MS != 0)
      _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif
      archContextSwitch(_fake_idle_routine(), _current_routine);
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
#ifdef SYN_OS_ENABLE_TIMEOUT_API
  Routine::timeouttick();
#endif
  if (_isr_reschedule_request & 0x80)
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
#if (SYN_OS_ROUTINE_RR_SLICE_MS != 0)
    _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
#endif

    archContextSwitch(_fake_idle_routine(), _current_routine);
  }
  else
  {
    --_isr_reschedule_request;
#if (SYN_OS_ROUTINE_RR_SLICE_MS != 0)
    --_current_ticks_left;
    if (_current_ticks_left == 0)
    {
      if (_readycount == 1 && _current_routine->is_runnable())
      {
        // no switch needed, just refresh this routines tick count
        _current_ticks_left = SYN_OS_ROUTINE_RR_SLICE_MS / SYN_SYSTICK_FREQ;
      }
      else
      {
        _contextSwitch();
      }
    }
#endif
  }
}

INTERRUPT_HANDLER(TIMER4_OV, 23)
{
#ifdef SYN_OS_MEASURE_SYSTICK
  _measure_pin.clear();
#endif
  Kernel::enter_isr();
  System::_systick_isr();
#if (SYN_OS_TICK_HOOK_COUNT > 0)
#ifdef SYN_OS_RUN_TIMER_ON_MAINSTACK
  if (Kernel::is_idle())
    SysTickHook::_checkAndExec();
  else
    synRunOnMainStack((void *)&SysTickHook::_checkAndExec, _mainstack);
#else
  SysTickHook::_checkAndExec();
#endif
#endif
  Kernel::_tickySwitch();
#ifdef SYN_OS_MEASURE_SYSTICK
  _measure_pin.set();
#endif
}
