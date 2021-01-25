#pragma once

//#define SYN_HAL_32_PIN_DEVICE
//#define SYN_HAL_I2C
//#define SYN_HAL_I2C_SLAVE
//#define SYN_HAL_UART
#define SYN_HAL_SPI
//#define SYN_HAL_SPI_SLAVE
//#define SYN_HAL_AUTO_WAKEUP_UNIT

//#define SYN_HAL_EXTI_A
//#define SYN_HAL_EXTI_B
//#define SYN_HAL_EXTI_C
//#define SYN_HAL_EXTI_D
//#define SYN_HAL_EXTI_E

// the ammount of used routines, has to match the actual ammount used
// it is detrtiement to use every routine index only once and use them all
#define SYN_OS_ROUTINE_COUNT 2
// determine wether to use fixed round robin slice or full cooporative mode (enter round robin time 0)
#define SYN_OS_ROUTINE_RR_SLICE_MS 10
// the ammount of hooks tracked by systick, has to match the actual ammount used
#define SYN_OS_TICK_HOOK_COUNT 1

#define SYN_OS_MAIN_STACK_SIZE 64
// control wether or not to use mainstack for running timer tasks
// there is a performance penality for the stack switch, but,
// if the timers use a lot of stack, it might be usefull.
#define SYN_OS_RUN_TIMER_ON_MAINSTACK
// wether to initialize the application stacks with 0xCD or not
#define SYN_OS_STACK_CHECK

// wether or not to enable event api, costs 1 byte of ram per routine
#define SYN_OS_USE_EVENTS

// wether or not to enable timeout api on blocking calls and sleep for routines
#define SYN_OS_ENABLE_TIMEOUT_API

// wether or not toggle pin A1 when enterering various functions and which
//#define SYN_OS_MEASURE_INTERNALS
//#define SYN_OS_MEASURE_SYSTICK

inline void syn_os_idle_hook()
{
  // wfi makes debugging a pain in the ass ( cant stop the target )
  //wfi();
}
 
