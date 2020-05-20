#pragma once
#include "../synhal8/synhal.h"
#include "intrinsics.h"
#include "assert.h"

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
    template<uint8_t Index, typename Functor_t, typename Argument_t>
    static void init(Functor_t functor, Argument_t arg, uint8_t* stack, uint16_t stacksize, uint8_t rr_millis);

    // leave execution context if other routinbe is runnable
    static void yield();

    bool is_runnable() const;
  private:
    void _init(void* functor, void* arg, uint8_t* stack);

    void block();
    void unblock();

    uint8_t* _stackptr;
    uint8_t _id_runnable;
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
    uint8_t _rr_reload;
#endif
    Routine* _next;
  };

  class SysTickHook
  {
  public:
    typedef void(*timer_functor_t)(void);
    
    template<uint8_t Index, uint8_t Reload>
    static void init(timer_functor_t functor);

    // called from systick, dont call manually
    static void _checkAndExec();
  private:
    void _init(timer_functor_t functor, uint8_t reload);

    static SysTickHook _timerlist[SYN_OS_TICK_HOOK_COUNT];

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
    bool get_isr();
    bool try_get();

    uint8_t count() const
    {
      return _count;
    }
  private:
    uint8_t _count;
    Routine* _waitlist;
  };

  class Kernel
  {
    friend class Routine;
    friend class Semaphore;
  public:
    static void init();
    static void spin();
    static Routine& currentRoutine();

    static bool is_idle();

    static void enter_isr()
    {
      assert(_isr_reschedule_request == 0);
      ++_isr_reschedule_request;
    }
    static void exit_isr();

    static void _tickySwitch();
  private:
    // general purpose context switch that always scedules next runnable, or idle
    // however, next runnable could also be the same routine.
    // so its not always smart to call, but always safe
    static void _contextSwitch();
    // optimized function to call if the current routine exits and might be blocked
    static void _contextYield(bool routine_shall_block);
    // optimized function to call when we already know something unblocked
    static void _contextUnblocked(Routine *unblocked);

    static uint8_t* _base_stack_setup(uint8_t* stack, uint16_t size);
    static void _idle();

    // put current Routine into the list, reshedule. Not allowed by ISR
    static void _enterWaitlist(Routine*& listhead);
    // unblock the head and reschedule if idle
    static void _unblockWaitlist(Routine*& listhead);
    // remove unblock the entire list and schedule the head if idle
    static void _unblockWaitlistAll(Routine*& listhead);


    static Routine _routinelist[SYN_OS_ROUTINE_COUNT];
    static uint8_t _current_ticks_left;
    static uint8_t _readycount;
    static uint8_t _isr_reschedule_request;
  };

  template<typename Mail_t, uint8_t Size>
  class MailBox
  {
  public:
    MailBox()
    {
      _pread = _pwrite = _mails;
    }

    bool push_isr(const Mail_t& mail)
    {
      bool ret = false;
      if(_signal.count() != Size)
      {
        *_pwrite++ = mail;
        if(_pwrite == &_mails[Size])
          _pwrite = _mails;
        _signal.give_isr();
        ret = true;
      }
      return ret;
    }

    bool try_push(const Mail_t& mail)
    {
      Atomic a;
      return push_isr(mail);
    }

    void push(const Mail_t& mail)
    {
      Atomic a;
      while(!push_isr(mail))
        Routine::yield();
    }

    bool pop_isr(Mail_t& mail)
    {
      bool ret = false;
      if(_signal.try_get())
      {
        mail = *_pread++;
        if(_pread == &_mails[Size])
          _pread = _mails;
        ret = true;
      }
      return ret;
    }

    bool try_pop(Mail_t& mail)
    {
      Atomic a;
      return pop_isr(mail);
    }

    void pop(Mail_t& mail)
    {
      Atomic a;
      _signal.get();
      mail = *_pread++;
      if(_pread == &_mails[Size])
        _pread = _mails;
    }
    
    // alternative API for zero-copy operation
    // however, best used with single producer single consumer!

    bool reserve_isr(Mail_t** mail)
    {
      assert(write_dirty == false);
      bool ret = false;
      if(_signal.count() != Size)
      {
        mail = _pwrite;
        ret = true;
        assert(write_dirty = true);
      }
      return ret;
    }

    bool try_reserve(Mail_t** mail)
    {
      Atomic a;
      return reserve_isr(mail);
    }

    void reserve(Mail_t** mail)
    {
      assert(write_dirty == false);
      Atomic a;
      while(_signal.count() == Size)
        Routine::yield();
      assert(write_dirty = true);
      mail = _pwrite;
    }

    void release_isr()
    {
      assert(write_dirty == true);
      _pwrite++;
      assert((write_dirty = false) == false);
      _signal.give_isr();
    }

    void release()
    {
      Atomic a;
      release_isr();
    }

    bool peek_isr(Mail_t*& mail)
    {
      assert(read_dirty == false);
      bool ret = false;
      if(_signal.count() != 0)
      {
        assert(read_dirty = true);
        mail = _pread;
        ret = true;
      }
      return ret;
    }

    bool try_peek(Mail_t*& mail)
    {
      Atomic a;
      return peek_isr(mail);
    }

    void peek(Mail_t*& mail)
    {
      while(!try_peek(mail))
        Routine::yield();
    }

    void purge_isr()
    {
      assert(read_dirty == true);
      assert(_signal.get_isr());
      if(++_pread == &_mails[Size])
        _pread = _mails;
      assert((read_dirty = false) == false);
    }

    void purge()
    {
      Atomic a;
      purge_isr();
    }
  private:
    Mail_t *_pread;
    Mail_t *_pwrite;
    Semaphore _signal;
    Mail_t _mails[Size];
#ifdef DEBUG
    bool read_dirty, write_dirty;
#endif
  };

  template<uint8_t Index, typename Functor_t, typename Argument_t>
  inline void Routine::init(Functor_t functor, Argument_t arg, uint8_t* stack, uint16_t stacksize, uint8_t rr_millis)
  {
    static_assert(Index < SYN_OS_ROUTINE_COUNT, "Routine Index out of bounds! Increase the ammount specified in the config");
    stack = Kernel::_base_stack_setup(stack, stacksize);
    assert((rr_millis / SYN_SYSTICK_FREQ) > 0);
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
    Kernel::_routinelist[Index]._rr_reload = rr_millis / SYN_SYSTICK_FREQ;
#endif
    Kernel::_routinelist[Index]._id_runnable = ((Index + 1) << 1) | 0x01;
    Kernel::_routinelist[Index]._init((void*)functor, (void*)arg, stack);
  }

  template<uint8_t Index, uint8_t Reload>
  inline void SysTickHook::init(SysTickHook::timer_functor_t functor)
  {
    static_assert(Index < SYN_OS_TICK_HOOK_COUNT, "Timer Index out of bounds!");
    static_assert(Reload > 0, "Timer needs at least Reload of 1");
    _timerlist[Index]._init(functor, Reload);
  }
} // namespace syn

