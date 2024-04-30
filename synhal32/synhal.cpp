#include "synhal.h"
#include <stdio.h>

using namespace syn;

#ifdef STM32F103xB
uint32_t Gpio::_swd_jtag_cfg = (uint32_t)Gpio::Remap::swj_no_nrst;
#endif

void OptionBytes::set_user_bit(uint32_t bitmask, uint32_t bitvalue)
{
  // check if the bit is already set
  if((FLASH->OPTR & bitmask) != bitvalue)
  {
    // unlock flash
    while(FLASH->SR & FLASH_SR_BSY)
      ;
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    // unlock option bytes
    FLASH->OPTKEYR = 0x08192A3B;
    FLASH->OPTKEYR = 0x4C5D6E7F;
    // write the bits
    uint32_t tmp = FLASH->OPTR & ~bitmask;
    tmp |= bitvalue;
    FLASH->OPTR = tmp;
    FLASH->CR |= FLASH_CR_OPTSTRT;
    while(FLASH->SR & FLASH_SR_BSY)
      ;
    // reload option bytes
    FLASH->CR |= FLASH_CR_OBL_LAUNCH;
    // lock the bytes and flash
    FLASH->CR |= FLASH_CR_OPTLOCK;
    FLASH->CR |= FLASH_CR_LOCK;
  }
}

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


void Print::print(char c)
{
  putchar(c);
}

void Print::print(const char* str)
{
  printf(str);
}

void Print::print(float f, uint16_t decimals)
{
  uint32_t dec = decimals * 10;
  printf("%f", f*dec);
}

void Print::println()
{
  putchar('\n');
}

void Print::println(const char c)
{
  char chars[3] = {c, '\n', 0};
  printf(chars);
}

void Print::println(const char* str)
{
  printf("%s\n", str);
}

