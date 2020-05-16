/*********************************************************************
*                     SEGGER Microcontroller GmbH                    *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*       (c) 1995 - 2020 SEGGER Microcontroller GmbH                  *
*                                                                    *
*       Internet: segger.com  Support: support_embos@segger.com      *
*                                                                    *
**********************************************************************
*                                                                    *
*       embOS * Real time operating system for microcontrollers      *
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
*       OS version: V5.8.2.0                                         *
*                                                                    *
**********************************************************************

----------------------------------------------------------------------
File    : OS_ThreadSafe.c
Purpose : Thread safe library functions
--------  END-OF-HEADER  ---------------------------------------------
*/

#include "RTOS.h"
#include "__libc.h"

/**********************************************************************
*
*       __heap_lock()
*/
void __heap_lock(void) {
  OS_HeapLock();
}

/**********************************************************************
*
*       __heap_unlock()
*/
void __heap_unlock(void) {
  OS_HeapUnlock();
}

/**********************************************************************
*
*       __printf_lock()
*/
void __printf_lock(void) {
  OS_PrintfLock();
}

/**********************************************************************
*
*       __printf_unlock()
*/
void __printf_unlock(void) {
  OS_PrintfUnlock();
}

/**********************************************************************
*
*       __scanf_lock()
*/
void __scanf_lock(void) {
  OS_ScanfLock();
}

/**********************************************************************
*
*       __scanf_unlock()
*/
void __scanf_unlock(void) {
  OS_ScanfUnlock();
}

/**********************************************************************
*
*       __debug_io_lock()
*/
void __debug_io_lock(void) {
  OS_DebugIOLock();
}

/**********************************************************************
*
*       __debug_io_unlock()
*/
void __debug_io_unlock(void) {
  OS_DebugIOUnlock();
}

/****** End Of File *************************************************/
