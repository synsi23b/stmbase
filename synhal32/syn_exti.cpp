#include "synhal.h"
#if (SYN_EXTI_TYPE_0 != 0 || SYN_EXTI_TYPE_1 != 0 || SYN_EXTI_TYPE_2 != 0 || SYN_EXTI_TYPE_3 != 0 || SYN_EXTI_TYPE_4 != 0 || \
     SYN_EXTI_TYPE_5 != 0 || SYN_EXTI_TYPE_6 != 0 || SYN_EXTI_TYPE_7 != 0 || SYN_EXTI_TYPE_8 != 0 || SYN_EXTI_TYPE_9 != 0 || \
     SYN_EXTI_TYPE_10 != 0 || SYN_EXTI_TYPE_11 != 0 || SYN_EXTI_TYPE_12 != 0 || SYN_EXTI_TYPE_13 != 0 || SYN_EXTI_TYPE_14 != 0 || SYN_EXTI_TYPE_15 != 0)

#include "../../src/synhal_isr.h"

using namespace syn;

extern "C"
{
#if (SYN_EXTI_TYPE_0 != 0)
  void EXTI0_IRQHandler()
  {
#if (SYN_EXTI_TYPE_0 == 1)
    Core::enter_isr();
#endif
    Exti::clear(0);
    syn_exti_0_isr();
#if (SYN_EXTI_TYPE_0 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_EXTI_TYPE_1 != 0)
  void EXTI1_IRQHandler()
  {
#if (SYN_EXTI_TYPE_1 == 1)
    Core::enter_isr();
#endif
    Exti::clear(1);
    syn_exti_1_isr();
#if (SYN_EXTI_TYPE_1 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_EXTI_TYPE_2 != 0)
  void EXTI2_IRQHandler()
  {
#if (SYN_EXTI_TYPE_2 == 1)
    Core::enter_isr();
#endif
    Exti::clear(2);
    syn_exti_2_isr();
#if (SYN_EXTI_TYPE_2 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_EXTI_TYPE_3 != 0)
  void EXTI3_IRQHandler()
  {
#if (SYN_EXTI_TYPE_3 == 1)
    Core::enter_isr();
#endif
    Exti::clear(3);
    syn_exti_3_isr();
#if (SYN_EXTI_TYPE_3 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_EXTI_TYPE_4 != 0)
  void EXTI4_IRQHandler()
  {
#if (SYN_EXTI_TYPE_4 == 1)
    Core::enter_isr();
#endif
    Exti::clear(4);
    syn_exti_4_isr();
#if (SYN_EXTI_TYPE_0 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_EXTI_TYPE_5 != 0 || SYN_EXTI_TYPE_6 != 0 || SYN_EXTI_TYPE_7 != 0 || SYN_EXTI_TYPE_8 != 0 || SYN_EXTI_TYPE_9 != 0)
  void EXTI9_5_IRQHandler()
  {
#if (SYN_EXTI_TYPE_5 == 1 || SYN_EXTI_TYPE_6 == 1 || SYN_EXTI_TYPE_7 == 1 || SYN_EXTI_TYPE_8 == 1 || SYN_EXTI_TYPE_9 == 1)
    Core::enter_isr();
#endif
#if (SYN_EXTI_TYPE_5 != 0)
    if (Exti::is_set(5))
    {
      Exti::clear(5);
      syn_exti_5_isr();
    }
#endif
#if (SYN_EXTI_TYPE_6 != 0)
    if (Exti::is_set(6))
    {
      Exti::clear(6);
      syn_exti_6_isr();
    }
#endif
#if (SYN_EXTI_TYPE_7 != 0)
    if (Exti::is_set(7))
    {
      Exti::clear(7);
      syn_exti_7_isr();
    }
#endif
#if (SYN_EXTI_TYPE_8 != 0)
    if (Exti::is_set(8))
    {
      Exti::clear(8);
      syn_exti_8_isr();
    }
#endif
#if (SYN_EXTI_TYPE_9 != 0)
    if (Exti::is_set(9))
    {
      Exti::clear(9);
      syn_exti_9_isr();
    }
#endif
#if (SYN_EXTI_TYPE_5 == 1 || SYN_EXTI_TYPE_6 == 1 || SYN_EXTI_TYPE_7 == 1 || SYN_EXTI_TYPE_8 == 1 || SYN_EXTI_TYPE_9 == 1)
    Core::leave_isr();
#endif
  }
#endif

#if (SYN_EXTI_TYPE_10 != 0 || SYN_EXTI_TYPE_11 != 0 || SYN_EXTI_TYPE_12 != 0 || SYN_EXTI_TYPE_13 != 0 || SYN_EXTI_TYPE_14 != 0 || SYN_EXTI_TYPE_15 != 0)
  void EXTI10_10_IRQHandler()
  {
#if (SYN_EXTI_TYPE_10 == 1 || SYN_EXTI_TYPE_11 == 1 || SYN_EXTI_TYPE_12 == 1 || SYN_EXTI_TYPE_13 == 1 || SYN_EXTI_TYPE_14 == 1 || SYN_EXTI_TYPE_15 == 1)
    Core::enter_isr();
#endif
#if (SYN_EXTI_TYPE_10 != 0)
    if (Exti::is_set(10))
    {
      Exti::clear(10);
      syn_exti_10_isr();
    }
#endif
#if (SYN_EXTI_TYPE_11 != 0)
    if (Exti::is_set(11))
    {
      Exti::clear(11);
      syn_exti_11_isr();
    }
#endif
#if (SYN_EXTI_TYPE_12 != 0)
    if (Exti::is_set(12))
    {
      Exti::clear(12);
      syn_exti_12_isr();
    }
#endif
#if (SYN_EXTI_TYPE_13 != 0)
    if (Exti::is_set(13))
    {
      Exti::clear(13);
      syn_exti_13_isr();
    }
#endif
#if (SYN_EXTI_TYPE_14 != 0)
    if (Exti::is_set(14))
    {
      Exti::clear(14);
      syn_exti_14_isr();
    }
#endif
#if (SYN_EXTI_TYPE_15 != 0)
    if (Exti::is_set(15))
    {
      Exti::clear(15);
      syn_exti_15_isr();
    }
#endif
#if (SYN_EXTI_TYPE_10 == 1 || SYN_EXTI_TYPE_11 == 1 || SYN_EXTI_TYPE_12 == 1 || SYN_EXTI_TYPE_13 == 1 || SYN_EXTI_TYPE_14 == 1 || SYN_EXTI_TYPE_15 == 1)
    Core::leave_isr();
#endif
  }
#endif
}
#endif
