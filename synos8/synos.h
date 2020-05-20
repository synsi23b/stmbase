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
  
    static void yield()
    {
      trap();
    }

    // mark the currently running routine as not runnable
    static void pause();
  private:
    void _init(void* functor, void* arg, uint8_t* stack);
    uint8_t* _stackptr;
    uint8_t _id_runnable;
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
    uint8_t _rr_reload;
#endif
  };

  class SoftTimer
  {
  public:
    typedef void(*timer_functor_t)(void);
    
    template<uint8_t Index, uint8_t Reload>
    static void init(timer_functor_t functor);

    // called from systick, dont call manually
    static void _checkAndExec();
  private:
    void _init(timer_functor_t functor, uint8_t reload);

    static SoftTimer _timerlist[SYN_OS_TIMER_COUNT];

    uint8_t _next_exec_time;
    uint8_t _reload;
    timer_functor_t _functor;
  };

  class Kernel
  {
    friend class Routine;
  public:
    static void init();
    static void spin();
    static Routine& currentRoutine();

    static void _tickySwitch();
    static void _funkySwitch();
  private:
    static void _contextSwitch();
    static uint8_t* _base_stack_setup(uint8_t* stack, uint16_t size);
    static void _idle();

    static Routine _routinelist[SYN_OS_ROUTINE_COUNT];
    static uint8_t _current_ticks_left;
    static uint8_t _readycount;
  };

  template<uint8_t Index, typename Functor_t, typename Argument_t>
  inline void Routine::init(Functor_t functor, Argument_t arg, uint8_t* stack, uint16_t stacksize, uint8_t rr_millis)
  {
    static_assert(Index < SYN_OS_ROUTINE_COUNT, "Routine Index out of bounds!");
    stack = Kernel::_base_stack_setup(stack, stacksize);
    assert((rr_millis / SYN_SYSTICK_FREQ) > 0);
#ifndef SYN_OS_ROUTINE_RR_SLICE_MS
    Kernel::_routinelist[Index]._rr_reload = rr_millis / SYN_SYSTICK_FREQ;
#endif
    Kernel::_routinelist[Index]._id_runnable = ((Index + 1) << 1) | 0x01;
    Kernel::_routinelist[Index]._init((void*)functor, (void*)arg, stack);
  }

  template<uint8_t Index, uint8_t Reload>
    inline void SoftTimer::init(SoftTimer::timer_functor_t functor)
  {
    static_assert(Index < SYN_OS_TIMER_COUNT, "Timer Index out of bounds!");
    static_assert(Reload > 0, "Timer needs at least Reload of 1");
    _timerlist[Index]._init(functor, Reload);
  }
} // namespace syn

