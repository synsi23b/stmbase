#pragma once
#include "../synhal8/synhal.h"
#include "intrinsics.h"
//#include "assert.h"

namespace syn
{
  class Atomic
  {
  public:
    Atomic()
    {
      _state = __get_interrupt_state();
      __disable_interrupt();
    }

    ~Atomic()
    {
      __set_interrupt_state(_state);
    }
  private:
    __istate_t _state;
  };

  class Kernel; // forward declared
  class Routine
  {
    friend class Kernel;

  public:
    typedef enum State
    {
      runnable = 0x00,
      wait_semaphore = 0x01,
      wait_mutex = 0x02,
      wait_event = 0x04,
      timeout_semaphore = 0x81,
      timeout_mutex = 0x82,
      timeout_event = 0x84,
      sleeping = 0x88,
      timeout = 0x80
    } State_t;

    // the stm8s has a stack roll over limit at the address 0x01FF.
    // if pushing the stack past that limit, it will reset to 0x03FF.
    // to avoid this, the routine stacks get allocated just before the main stack.
    // care has to be taken to avoid the dangerous address.
    // for example if the main stack is 64 bytes, 512 - 64 bytes can be allocated in the range 0x1FF .. 0x3BF
    // there will be a warning when a stack crosses this line. which might be neccessary to allow more than
    // 512byte total for stack if that is needed.
    // the main stack maximum can be configured in the synos cfg file
    static void init(void (*functor)(uint16_t), uint16_t arg, uint8_t stacksize);

    // leave execution context if other routinbe is runnable
    static void yield();
    // block routine for specified time in milliseconds
    // requieres enabling of timeout api and relies on the cooperation of other routines
    // will just block this routine from running AT LEAST the specified timeout
    static void sleep(uint16_t milliseconds);
    // wether or not this routine is runnable
    bool is_runnable() const;

    // optimized yield that is only valid in a protected area. use cautious or not at all
    static void yield_protected();

    uint8_t lastEvent() const;

    // "private" functions, dont want to friend every class, shouldn't be called by application
    void _setEventValue(uint8_t value);
    void _setupTimeout(uint16_t timeout_ms, Routine **waitlist);
    bool _timeout_is_expired() const;

  private:
    //void _init(void (*functor)(uint16_t), uint16_t arg, uint8_t *stack);

    void block(State reason);
    void unblock();

    State_t _state;
    uint8_t *_stackptr;
    // linked list for waiting on Semaphores, Mutex, etc..
    Routine *_next;
#ifdef SYN_OS_USE_EVENTS
    // the value of the event when this routine got unblocked
    uint8_t _eventval;
#endif
#ifdef SYN_OS_ENABLE_TIMEOUT_API

    static void timeouttick();

    class TestTimoutExpired
    {
    public:
      TestTimoutExpired();
      void operator()(Routine *pr);

    private:
      uint16_t _current_millis;
    };

    // the system millis value to wait for
    uint16_t _timeout;
    // the timeoutable list to be removed from in case timout is reached
    // zero if the routine is just sleeping and not for example locking a mutex with timeout
    Routine **_waitlist;
#endif
  };
  
  // block current routine for specified time in milliseconds
  // requieres enabling of timeout api and relies on the cooperation of other routines
  // will just block this routine from running AT LEAST the specified timeout
  inline void sleep(uint16_t milliseconds)
  {
    syn::Routine::sleep(milliseconds);
  }

  class Rate
  {
  public:
    // Attempt to run at a secific rate by sleeping in between and compensating for
    // code exectuion time by re-evaluating the last execution time
    // since it's all integer math, usefull ranges are in 1..500
    // faster than 500 hertz results in a 1ms sleep, which could be just added manually
    Rate(uint16_t hertz);

    void sleep();

  private:
    uint16_t _last_run;
    uint16_t _sleep_time;
  };

  class SysTickHook
  {
  public:
    typedef void (*timer_functor_t)(void);

    template <uint8_t Index, uint16_t Reload_ms>
    static void init(timer_functor_t functor);

    // called from systick, dont call manually
    static void _checkAndExec();

  private:
    void _init(timer_functor_t functor, uint8_t reload);

#if (SYN_OS_TICK_HOOK_COUNT > 0)
    static SysTickHook _timerlist[SYN_OS_TICK_HOOK_COUNT];
#endif
    uint8_t _next_exec_time;
    uint8_t _reload;
    timer_functor_t _functor;
  };

  class Semaphore
  {
  public:
    Semaphore(uint8_t start = 0);

    void give();
    void give_isr();

    void get();
    bool get(uint16_t timeout);
    bool get_isr();
    bool try_get();

    uint8_t count() const;

  private:
    uint8_t _count;
    Routine *_waitlist;
  };

  class Mutex
  {
  public:
    void lock();
    bool lock(uint16_t timeout);
    bool try_lock();

    void unlock();

    uint8_t count() const;

  private:
    Routine *_owner;
    uint8_t _count;
    Routine *_waitlist;
  };

  class Event
  {
  public:
    // set the regardless of its state from an isr
    // value of zero is forbidden / is the same as clearing the event
    void set_isr(uint8_t value);
    // set the event regardless of its state
    // value of zero is forbidden / is the same as clearing the event
    void set(uint8_t value);
    // set the event regardless of its state from an isr
    // return true if the event was already set
    // value of zero is forbidden / is the same as clearing the event
    bool set_check_isr(uint8_t value);
    // set the event regardless of its state
    // return true if the event was already set
    // value of zero is forbidden / is the same as clearing the event
    bool set_check(uint8_t value);
    // set the event, if no routine is waiting, else pulse, from an isr
    void set_or_pulse_isr(uint8_t value);
    // set the event, if no routine is waiting, else pulse
    void set_or_pulse(uint8_t value);
    // wake up any waiting routine but don't store the value, from an isr
    void pulse_isr(uint8_t value);
    // wake up any waiting routine but don't store the value, from an isr
    void pulse(uint8_t value);
    // unconditionally clear the event
    void clear();
    // try to read the event, returns the event value, or 0 if not set
    // always resets the event upon exit of the method
    uint8_t get_clr_isr();
    // try to read the event, returns the event value, or 0 if not set
    uint8_t get() const;
    // try to read the event, returns the event value, or 0 if not set
    // always resets the event upon exit of the method
    uint8_t get_clr();
    // wait for the event to be set and return its value
    // doesn't clear the event value
    uint8_t wait();
    // wait for the event to be set and return its value
    // doesn't clear the event value
    // returns 0 if timeout
    uint8_t wait(uint16_t timeout);

  private:
    uint8_t _value;
    Routine *_waitlist;
  };

  class Kernel
  {
    friend class Routine;
    friend class Semaphore;
    friend class Mutex;
    friend class Event;

  public:
    static void init();
    static void spin();
    static Routine &currentRoutine();

    static bool is_idle();
    static bool is_isr();

    static void enter_isr();
    static void exit_isr();

    static void _tickySwitch();

  private:
    template <typename Functor>
    static void for_each_routine(Functor functor);
    // general purpose context switch that always scedules next runnable, or idle
    // however, next runnable could also be the same routine.
    // so its not always smart to call, but always safe
    static void _contextSwitch();
    // optimized function to call if the current routine exits and might be blocked
    static void _contextYield(Routine::State blocking_state);
    // optimized function to call when we already know something unblocked
    static void _contextUnblocked(Routine *unblocked);

    static uint8_t *_base_stack_setup(uint16_t size);
    static void _idle();
    static Routine *_fake_idle_routine();

    // put current Routine into the list, reshedule. Not allowed by ISR
    static void _enterWaitlist(Routine *&listhead, Routine::State blocking_reason);
    // unblock the head and reschedule if idle
    static void _unblockWaitlist(Routine *&listhead);
    // remove unblock the entire list and schedule the head if idle, set routine event value
    static void _unblockEventlist(Routine *&listhead, uint8_t value);
    // remove the specified routine from the waitlist, but don't reschedule
    // returns true if the routine is actually in the waitlist
    static bool _removeWaitlist(Routine **listhead, Routine *to_remove);

    static Routine _routinelist[SYN_OS_ROUTINE_COUNT];
#if (SYN_OS_ROUTINE_RR_SLICE_MS != 0)
    static uint8_t _current_ticks_left;
#endif
    static uint8_t _readycount;
    static uint8_t _isr_reschedule_request;
  };

  template <typename Mail_t, uint8_t Size>
  class MailBox
  {
  public:
    MailBox()
        : _sig_write(Size)
    {
      _pread = _pwrite = _mails;
    }

    bool push_isr(const Mail_t &mail)
    {
      bool ret = false;

      if (_sig_write.get_isr())
      {
        _write(mail);
        _sig_read.give_isr();
        ret = true;
      }
      return ret;
    }

    bool try_push(const Mail_t &mail)
    {
      Atomic a;
      return push_isr(mail);
    }

    void push(const Mail_t &mail)
    {
      _sig_write.get();
      {
        Atomic a;
        _write(mail);
        _sig_read.give_isr();
      }
    }

    bool pop_isr(Mail_t &mail)
    {
      bool ret = false;
      if (_sig_read.get_isr())
      {
        _read(mail);
        _sig_write.give_isr();
        ret = true;
      }
      return ret;
    }

    bool try_pop(Mail_t &mail)
    {
      Atomic a;
      return pop_isr(mail);
    }

    void pop(Mail_t &mail)
    {
      _sig_read.get();
      {
        Atomic a;
        _read(mail);
        _sig_write.give_isr();
      }
    }

    // alternative API for zero-copy operation
    // however, best used with just single producer & single consumer!

    bool reserve_isr(Mail_t **mail)
    {
      //assert(write_dirty == false);
      bool ret = false;
      if (!_write_dirty && _sig_write.get_isr())
      {
        *mail = _pwrite;
        ret = true;
        _write_dirty = true;
      }
      return ret;
    }

    bool try_reserve(Mail_t **mail)
    {
      Atomic a;
      return reserve_isr(mail);
    }

    void reserve(Mail_t **mail)
    {
      // not putting it behind Debug, as another routine might undirty this
      while (_write_dirty)
        ;
      //assert(_write_dirty == false);
      _sig_write.get();
      _write_dirty = true;
      *mail = _pwrite;
    }

    void release_isr()
    {
      //assert(_write_dirty == true);
#ifdef DEBUG
      while (_write_dirty == false)
        ;
#endif
      if (++_pwrite == &_mails[Size])
        _pwrite = _mails;
      _write_dirty = false;
      //assert((write_dirty = false) == false);
      _sig_read.give_isr();
    }

    void release()
    {
      Atomic a;
      release_isr();
    }

    bool peek_isr(Mail_t **mail)
    {
      //assert(read_dirty == false);
#ifdef DEBUG
      while (_read_dirty)
        ;
#endif
      bool ret = false;
      if (_sig_read.get_isr())
      {
        //assert(read_dirty = true);
#ifdef DEBUG
        _read_dirty = true;
#endif
        *mail = _pread;
        ret = true;
      }
      return ret;
    }

    bool try_peek(Mail_t **mail)
    {
      Atomic a;
      return peek_isr(mail);
    }

    void peek(Mail_t **mail)
    {
      //assert(read_dirty == false);
#ifdef DEBUG
      while (_read_dirty)
        ;
#endif
      _sig_read.get();
      //assert(read_dirty = true);
#ifdef DEBUG
      _read_dirty = true;
#endif
      *mail = _pread;
    }

    void purge_isr()
    {
      //assert(read_dirty == true);
#ifdef DEBUG
      while (!_read_dirty)
        ;
#endif
      if (++_pread == &_mails[Size])
        _pread = _mails;
        //assert((read_dirty = false) == false);
#ifdef DEBUG
      _read_dirty = false;
#endif
      _sig_write.give_isr();
    }

    void purge()
    {
      Atomic a;
      purge_isr();
    }

  private:
    void _write(const Mail_t &mail)
    {
      *_pwrite++ = mail;
      if (_pwrite == &_mails[Size])
        _pwrite = _mails;
    }

    void _read(Mail_t &mail)
    {
      mail = *_pread++;
      if (_pread == &_mails[Size])
        _pread = _mails;
    }

    Mail_t *_pread;
    Mail_t *_pwrite;
    bool _write_dirty;
    Semaphore _sig_read;
    Semaphore _sig_write;
    Mail_t _mails[Size];
#ifdef DEBUG
    bool _read_dirty;
#endif
  };

#if (SYN_OS_TICK_HOOK_COUNT > 0)
  template <uint8_t Index, uint16_t Reload_ms>
  inline void SysTickHook::init(SysTickHook::timer_functor_t functor)
  {
    static_assert(Index < SYN_OS_TICK_HOOK_COUNT, "Timer Index out of bounds!");
    static_assert(Reload_ms > 0, "Timer needs at least Reload of 1");
    static_assert((Reload_ms / SYN_SYSTICK_FREQ) < 256, "Timer reload needs to be less than 256 ticks");
    _timerlist[Index]._init(functor, Reload_ms / SYN_SYSTICK_FREQ);
  }
#endif

  inline uint8_t Semaphore::count() const
  {
    return _count;
  }

  inline uint8_t Mutex::count() const
  {
    return _count;
  }

#ifdef SYN_OS_USE_EVENTS
  // unconditionally clear the event
  inline void Event::clear()
  {
    _value = 0;
  }
  // try to read the event, returns the event value, or 0 if not set
  // always resets the event upon exit of the method
  inline uint8_t Event::get_clr_isr()
  {
    uint8_t ret = _value;
    _value = 0;
    return ret;
  }
  // try to read the event, returns the event value, or 0 if not set
  inline uint8_t Event::get() const
  {
    return _value;
  }
  // try to read the event, returns the event value, or 0 if not set
  // always resets the event upon exit of the method
  inline uint8_t Event::get_clr()
  {
    Atomic a;
    return get_clr_isr();
  }

  // set the event, if no routine is waiting, else pulse, from an isr
  inline void Event::set_or_pulse_isr(uint8_t value)
  {
    if (_waitlist == 0)
    {
      _value = value;
    }
    else
    {
      _value = 0;
      Kernel::_unblockEventlist(_waitlist, value);
    }
  }

  // set the event, if no routine is waiting, else pulse
  inline void Event::set_or_pulse(uint8_t value)
  {
    Atomic a;
    set_or_pulse_isr(value);
  }

  inline void Event::pulse_isr(uint8_t value)
  {
    Kernel::_unblockEventlist(_waitlist, value);
  }

  inline void Event::pulse(uint8_t value)
  {
    Atomic a;
    Kernel::_unblockEventlist(_waitlist, value);
  }

  inline uint8_t Routine::lastEvent() const
  {
    return _eventval;
  }
#endif

  inline void Kernel::enter_isr()
  {
#ifdef DEBUG
    //assert(_isr_reschedule_request == 0);
    while (_isr_reschedule_request)
      ;
#endif
    _isr_reschedule_request = 1;
  }

  inline bool Kernel::is_isr()
  {
    return _isr_reschedule_request != 0;
  }
} // namespace syn
