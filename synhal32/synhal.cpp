#include "synhal.h"

using namespace syn;

#ifdef STM32F103xB
uint32_t Gpio::_swd_jtag_cfg = (uint32_t)Gpio::Remap::swj_no_nrst;
#endif

void SoftTimer::_oneshot(SoftTimer *this_timer)
{
  this_timer->execute();
}

void SoftTimer::_reload(SoftTimer *this_timer)
{
  this_timer->execute();
  this_timer->restart();
}

void Thread::runner(Thread *this_thread)
{
  this_thread->run();
  this_thread->terminate();
}

void System::init()
{
  OS_Init();
  OS_InitHW();
#if (SYN_ENABLE_USBRPC != 0)
  UsbRpc::init();
#endif
}

extern "C"
{
  // compatibility to STM HAL
  void HAL_Delay(uint32_t Delay)
  {
    syn::System::delay(Delay);
  }
}