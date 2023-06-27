/*********************************************************************
*                     SEGGER Microcontroller GmbH                    *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 1995 - 2022 SEGGER Microcontroller GmbH                  *
*                                                                    *
*       Internet: segger.com  Support: support_embos@segger.com      *
*                                                                    *
**********************************************************************
*                                                                    *
*       embOS * Real time operating system                           *
*                                                                    *
*       Please note:                                                 *
*                                                                    *
*       Knowledge of this file may under no circumstances            *
*       be used to write a similar product or a real-time            *
*       operating system for in-house use.                           *
*                                                                    *
*       Thank you for your fairness !                                *
*                                                                    *
**********************************************************************
*                                                                    *
*       OS version: V5.18.0.0                                        *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------
Purpose : Initializes and handles the hardware for embOS
*/

#include "RTOS.h"
#include "SEGGER_SYSVIEW.h"
#include "stm32f10x.h"

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

/*********************************************************************
*
*       System tick settings
*/
#define OS_TIMER_FREQ  (SystemCoreClock)
#define OS_TICK_FREQ   (1000u)
#define OS_INT_FREQ    (OS_TICK_FREQ)

/*********************************************************************
*
*       embOSView settings
*/
#ifndef   OS_VIEW_IFSELECT
  #define OS_VIEW_IFSELECT  OS_VIEW_IF_JLINK
#endif

#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  #include "JLINKMEM.h"
#elif (OS_VIEW_IFSELECT == OS_VIEW_IF_UART)
  #include "BSP_UART.h"
  #define OS_UART      (0u)
  #define OS_BAUDRATE  (38400u)
#endif

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  const OS_U32 OS_JLINKMEM_BufferSize = 32u;  // Size of the communication buffer for JLINKMEM
#else
  const OS_U32 OS_JLINKMEM_BufferSize = 0u;   // Buffer not used
#endif

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/

#if (OS_VIEW_IFSELECT == OS_VIEW_IF_UART)
/*********************************************************************
*
*       _OS_OnRX()
*
*  Function description
*    Callback wrapper function for BSP UART module.
*/
static void _OS_OnRX(unsigned int Unit, unsigned char c) {
  OS_USE_PARA(Unit);
  OS_COM_OnRx(c);
}

/*********************************************************************
*
*       _OS_OnTX()
*
*  Function description
*    Callback wrapper function for BSP UART module.
*/
static int _OS_OnTX(unsigned int Unit) {
  OS_USE_PARA(Unit);
  return (int)OS_COM_OnTx();
}
#endif


/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/
/*********************************************************************
*
*       BSP_OS_GetCycles()
*
*  Function description
*    Calculates the current time in counter cycles since reset.
*
*  Return value
*    current time in counter cycles since reset.
*/
OS_U64 BSP_OS_GetCycles(void) {
  OS_U32 TimerCycles;
  OS_U32 CycleCntLow;
  OS_U32 CycleCntHigh;
  OS_U32 CycleCntCompare;
  OS_U64 CycleCnt64;

  do {
    CycleCnt64   = OS_TIME_GetTimestamp();      // Read current OS_Global.Time
    CycleCntLow  = (OS_U32)CycleCnt64;          // Extract lower word of OS_Global.Time
    CycleCntHigh = (OS_U32)(CycleCnt64 >> 32);  // Extract higher word of OS_Global.Time
    TimerCycles  = DWT->CYCCNT;                 // Read current hardware CycleCount
    //
    // CycleCnt64 and TimerCycles need to be retrieved "simultaneously" for the below calculation to work:
    // If the above code is interrupted after retrieving OS_Global.Time, but before reading the hardware counter,
    // it may happen that the hardware counter overflows multiple times before returning here.
    // We could then no longer accurately compare (TimerCycles < CycleCntLow) below.
    //
    // Hence, we check if the upper word of OS_Global.Time still matches the value we retrieved earlier and repeat
    // the process if it doesn't. This works because OS_Global.Time is updated once per timer interrupt, and the timer
    // interrupt must be more frequent than the hardware counter overflow, so any overflow has necessarily incremented the upper word.
    //
    CycleCntCompare = (OS_U32)(OS_TIME_GetTimestamp() >> 32);
  } while (CycleCntHigh != CycleCntCompare);
  //
  // Calculate the current time in counter cycles since reset.
  // This is done by simply copying the 32 bit hardware time stamp into the lower 32 bits of OS_Global.Time, while
  // the upper 32 bits of OS_Global.Time count the number of overflows of the hardware counter.
  // Therefore, we compare the lower 32 bits of OS_Global.Time to the newly retrieved hardware time stamp:
  //
  if (TimerCycles < CycleCntLow) {
    //
    // The hardware counter has overflown and we must increment the upper 32 bits of OS_Global.Time.
    //
    CycleCnt64 = ((OS_U64)(CycleCntHigh + 1u) << 32) + TimerCycles;
  } else {
    //
    // The hardware counter has not overflown and we do not perform any additional action.
    //
    CycleCnt64 = ((OS_U64)CycleCntHigh << 32) + TimerCycles;
  }
  return CycleCnt64;
}

/*********************************************************************
*
*       BSP_OS_StartTimer()
*
*  Function description
*    Program the hardware timer required for embOS's time base.
*    The used hardware timer must generate an interrupt before the used counter completes a full loop.
*
*  Parameters
*    Cycles: The amount of cycles after which the hardware timer shall generate an interrupt.
*/
void BSP_OS_StartTimer(OS_U32 Cycles) {
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                | SysTick_CTRL_TICKINT_Msk;  // Disable SysTick
#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  //
  // If embOSView via J-Link is selected, JLINKMEM_Process() needs to be executed periodically by the application.
  // In this exemplary BSP implementation, JLINKMEM_Process() is called from the SysTick_Handler().
  // We therefore limit the system tick to occur after a maximum of 10 milliseconds,
  // guaranteeing JLINKMEM_Process() is executed at least every 10 milliseconds.
  //
  if (Cycles > (SystemCoreClock / 100u)) {
    Cycles = SystemCoreClock / 100u;
  }
#endif
  SysTick->LOAD = Cycles;                    // Set reload register
  SysTick->VAL  = 0u;                        // Load the SysTick Counter Value
  SysTick->CTRL = SysTick_CTRL_ENABLE_Msk
                | SysTick_CTRL_CLKSOURCE_Msk
                | SysTick_CTRL_TICKINT_Msk;  // Enable SysTick
  SysTick->LOAD = 0u;                        // Make sure we receive one interrupt only by clearing the reload value
}

/*********************************************************************
*
*       SysTick_Handler()
*
*  Function description
*    This is the hardware timer exception handler.
*/
void SysTick_Handler(void) {
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                | SysTick_CTRL_TICKINT_Msk;  // Disable SysTick
  OS_INT_EnterNestable();
  OS_TICK_Handle();
#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  JLINKMEM_Process();
#endif
  OS_INT_LeaveNestable();
}

/*********************************************************************
*
*       OS_InitHW()
*
*  Function description
*    Initialize the hardware required for embOS to run.
*/
void OS_InitHW(void) {
  OS_INT_IncDI();
  //
  // Initialize NVIC vector table offset register if applicable for this device.
  // Might be necessary for RAM targets or application not running from 0x00.
  //
#if (defined(__VTOR_PRESENT) && __VTOR_PRESENT == 1)
  extern int __Vectors;
  SCB->VTOR = (OS_U32)&__Vectors;
#endif
  //
  // Enable instruction and data cache if applicable for this device
  //
#if (defined(__ICACHE_PRESENT) &&  __ICACHE_PRESENT == 1)
  SCB_EnableICache();
#endif
#if (defined(__DCACHE_PRESENT) &&  __DCACHE_PRESENT == 1)
  SCB_EnableDCache();
#endif
  //
  // Inform embOS about the frequency of the counter
  //
  {
    SystemCoreClockUpdate();
    OS_SYSTIMER_CONFIG SysTimerConfig = { SystemCoreClock };
    OS_TIME_ConfigSysTimer(&SysTimerConfig);
  }
  //
  // Start the counter (here: cycle counter)
  //
  if ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) == 0) {  // Trace not enabled?
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;            // Enable trace
  }
  if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0) {              // Cycle counter supported?
    if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0) {           // Cycle counter not enabled?
      DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                     // Enable Cycle counter
    }
  }
  //
  // Start the hardware timer (here: SysTick)
  //
  NVIC_SetPriority(SysTick_IRQn, (1u << __NVIC_PRIO_BITS) - 2u);  // Set the priority higher than the PendSV priority
  BSP_OS_StartTimer(0xFFFFFFFFu);
  //
  // Configure and initialize SEGGER SystemView
  //
  //
  // Configure and initialize SEGGER SystemView
  //
#if (OS_SUPPORT_TRACE_API != 0)
  SEGGER_SYSVIEW_Conf();
#endif
  //
  // Initialize communication for embOSView
  //
#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  JLINKMEM_SetpfOnRx(OS_COM_OnRx);
  JLINKMEM_SetpfOnTx(OS_COM_OnTx);
  JLINKMEM_SetpfGetNextChar(OS_COM_GetNextChar);
#elif (OS_VIEW_IFSELECT == OS_VIEW_IF_UART)
  BSP_UART_Init(OS_UART, OS_BAUDRATE, BSP_UART_DATA_BITS_8, BSP_UART_PARITY_NONE, BSP_UART_STOP_BITS_1);
  BSP_UART_SetReadCallback(OS_UART, _OS_OnRX);
  BSP_UART_SetWriteCallback(OS_UART, _OS_OnTX);
#endif
  OS_INT_DecRI();
}

/*********************************************************************
*
*       OS_Idle()
*
*  Function description
*    This code is executed whenever no task, software timer, or
*    interrupt is ready for execution.
*
*  Additional information
*    The idle loop does not have a stack of its own, therefore no
*    functionality should be implemented that relies on the stack
*    to be preserved.
*/
void OS_Idle(void) {  // Idle loop: No task is ready to execute
  while (1) {         // Nothing to do ... wait for interrupt
    #if ((OS_VIEW_IFSELECT != OS_VIEW_IF_JLINK) && (OS_DEBUG == 0))
      //
      // When uncommenting this line, please be aware device
      // specific issues could occur.
      // Therefore, we do not call __WFI() by default.
      //
      //__WFI();        // Switch CPU into sleep mode
    #endif
  }
}

/*********************************************************************
*
*       Optional communication with embOSView
*
**********************************************************************
*/

/*********************************************************************
*
*       OS_COM_Send1()
*
*  Function description
*    Sends one character.
*/
void OS_COM_Send1(OS_U8 c) {
#if (OS_VIEW_IFSELECT == OS_VIEW_IF_JLINK)
  JLINKMEM_SendChar(c);
#elif (OS_VIEW_IFSELECT == OS_VIEW_IF_UART)
  BSP_UART_Write1(OS_UART, c);
#elif (OS_VIEW_IFSELECT == OS_VIEW_DISABLED)
  OS_USE_PARA(c);          // Avoid compiler warning
  OS_COM_ClearTxActive();  // Let embOS know that Tx is not busy
#endif
}

/*************************** End of file ****************************/
