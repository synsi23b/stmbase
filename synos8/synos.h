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
    template<typename Functor_t, typename Argument_t>
    static void init(Functor_t functor, Argument_t arg, uint8_t* stack, uint16_t stacksize, uint8_t rr_millis);
  
    static void yield()
    {
      trap();
    }
  private:
    void _init(void* functor, void* arg, uint8_t* stack);
    uint8_t* _stackptr;
    uint8_t _state_1;
    uint8_t _state_2;
  };

  class SoftTimer
  {
  public:
    typedef void(*timer_functor)(void);

    static void init(timer_functor, uint8_t reload);

    // called from systick, dont call manually
    static void _checkAndExec();
  private:
    uint8_t _next_exec_time;
    uint8_t _reload;
    timer_functor _functor;
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

    static Routine* _current;
    static Routine _routinelist[SYN_OS_ROUTINE_COUNT];
    static Routine _idler;

    static uint8_t _current_ticks_left;
    static uint8_t _readycount;
  };

  template<typename Functor_t, typename Argument_t>
  inline void Routine::init(Functor_t functor, Argument_t arg, uint8_t* stack, uint16_t stacksize, uint8_t rr_millis)
  {
    stack = Kernel::_base_stack_setup(stack, stacksize);
    assert(Kernel::_current != &Kernel::_routinelist[SYN_OS_ROUTINE_COUNT]);
    assert((rr_millis / SYN_SYSTICK_FREQ) > 0);
    Kernel::_current->_state_1 = rr_millis / SYN_SYSTICK_FREQ;
    Kernel::_current->_init((void*)functor, (void*)arg, stack);
    ++Kernel::_current;
  }
} // namespace syn

#ifdef __cplusplus
extern "C" {
#endif
  extern void archContextSwitch (syn::Routine* p_old, syn::Routine* p_new);
  extern void archFirstThreadRestore(syn::Routine* p_first);
  extern void synRunFirstTime();
#ifdef __cplusplus
}
#endif
