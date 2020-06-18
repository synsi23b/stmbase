/**
  ******************************************************************************
  * @file    stm32f4xx_hal_pcd.h
  * @author  MCD Application Team
  * @brief   Header file of PCD HAL module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32F4xx_HAL_PCD_H
#define STM32F4xx_HAL_PCD_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_ll_usb.h"

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)

typedef enum
{
  HAL_PCD_STATE_RESET   = 0x00,
  HAL_PCD_STATE_READY   = 0x01,
  HAL_PCD_STATE_ERROR   = 0x02,
  HAL_PCD_STATE_BUSY    = 0x03,
  HAL_PCD_STATE_TIMEOUT = 0x04
} PCD_StateTypeDef;

/* Device LPM suspend state */
typedef enum
{
  LPM_L0 = 0x00, /* on */
  LPM_L1 = 0x01, /* LPM L1 sleep */
  LPM_L2 = 0x02, /* suspend */
  LPM_L3 = 0x03, /* off */
} PCD_LPM_StateTypeDef;

typedef enum
{
  PCD_LPM_L0_ACTIVE = 0x00, /* on */
  PCD_LPM_L1_ACTIVE = 0x01, /* LPM L1 sleep */
} PCD_LPM_MsgTypeDef;

typedef enum
{
  PCD_BCD_ERROR                     = 0xFF,
  PCD_BCD_CONTACT_DETECTION         = 0xFE,
  PCD_BCD_STD_DOWNSTREAM_PORT       = 0xFD,
  PCD_BCD_CHARGING_DOWNSTREAM_PORT  = 0xFC,
  PCD_BCD_DEDICATED_CHARGING_PORT   = 0xFB,
  PCD_BCD_DISCOVERY_COMPLETED       = 0x00,

} PCD_BCD_MsgTypeDef;

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
typedef USB_OTG_GlobalTypeDef  PCD_TypeDef;
typedef USB_OTG_CfgTypeDef     PCD_InitTypeDef;
typedef USB_OTG_EPTypeDef      PCD_EPTypeDef;
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

/**
  * @brief  PCD Handle Structure definition
  */
#define USB_CDC_IN_EP_COUNT  3U
#define USB_CDC_OUT_EP_COUNT 2U

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
typedef struct __PCD_HandleTypeDef
#else
typedef struct
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  //PCD_TypeDef             *Instance;   /*!< Register base address             */
  //PCD_InitTypeDef         Init;        /*!< PCD required parameters           */
  __IO uint8_t            USB_Address; /*!< USB Address                       */
  PCD_EPTypeDef           IN_ep[USB_CDC_IN_EP_COUNT];   /*!< IN endpoint parameters            */
  PCD_EPTypeDef           OUT_ep[USB_CDC_OUT_EP_COUNT];  /*!< OUT endpoint parameters           */
  HAL_LockTypeDef         Lock;        /*!< PCD peripheral status             */
  __IO PCD_StateTypeDef   State;       /*!< PCD communication state           */
  //__IO  uint32_t          ErrorCode;   /*!< PCD Error code                    */
  uint32_t                Setup[12];   /*!< Setup packet buffer               */
  PCD_LPM_StateTypeDef    LPM_State;   /*!< LPM State                         */
  //uint32_t                BESL;


  uint32_t lpm_active;                 /*!< Enable or disable the Link Power Management .
                                       This parameter can be set to ENABLE or DISABLE        */

  //uint32_t battery_charging_active;    /*!< Enable or disable Battery charging.
  //                                     This parameter can be set to ENABLE or DISABLE        */
  //void                    *pData;      /*!< Pointer to upper stack Handler */

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  void (* SOFCallback)(struct __PCD_HandleTypeDef *hpcd);                              /*!< USB OTG PCD SOF callback                */
  void (* SetupStageCallback)(struct __PCD_HandleTypeDef *hpcd);                       /*!< USB OTG PCD Setup Stage callback        */
  void (* ResetCallback)(struct __PCD_HandleTypeDef *hpcd);                            /*!< USB OTG PCD Reset callback              */
  void (* SuspendCallback)(struct __PCD_HandleTypeDef *hpcd);                          /*!< USB OTG PCD Suspend callback            */
  void (* ResumeCallback)(struct __PCD_HandleTypeDef *hpcd);                           /*!< USB OTG PCD Resume callback             */
  void (* ConnectCallback)(struct __PCD_HandleTypeDef *hpcd);                          /*!< USB OTG PCD Connect callback            */
  void (* DisconnectCallback)(struct __PCD_HandleTypeDef *hpcd);                       /*!< USB OTG PCD Disconnect callback         */

  void (* DataOutStageCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);      /*!< USB OTG PCD Data OUT Stage callback     */
  void (* DataInStageCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);       /*!< USB OTG PCD Data IN Stage callback      */
  void (* ISOOUTIncompleteCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);  /*!< USB OTG PCD ISO OUT Incomplete callback */
  void (* ISOINIncompleteCallback)(struct __PCD_HandleTypeDef *hpcd, uint8_t epnum);   /*!< USB OTG PCD ISO IN Incomplete callback  */
  void (* BCDCallback)(struct __PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);      /*!< USB OTG PCD BCD callback                */
  void (* LPMCallback)(struct __PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);      /*!< USB OTG PCD LPM callback                */

  void (* MspInitCallback)(struct __PCD_HandleTypeDef *hpcd);                          /*!< USB OTG PCD Msp Init callback           */
  void (* MspDeInitCallback)(struct __PCD_HandleTypeDef *hpcd);                        /*!< USB OTG PCD Msp DeInit callback         */
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
} PCD_HandleTypeDef;

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/**
  * @}
  */

/* Include PCD HAL Extended module */
//#include "stm32f4xx_hal_pcd_ex.h"
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(uint8_t fifo, uint16_t size);
HAL_StatusTypeDef HAL_PCDEx_SetRxFiFo(uint16_t size);
void HAL_PCDEx_LPM_Callback(PCD_LPM_MsgTypeDef msg);
void HAL_PCDEx_BCD_Callback(PCD_BCD_MsgTypeDef msg);

/* Exported constants --------------------------------------------------------*/
/** @defgroup PCD_Exported_Constants PCD Exported Constants
  * @{
  */

/** @defgroup PCD_Speed PCD Speed
  * @{
  */
#define PCD_SPEED_HIGH               USBD_HS_SPEED
#define PCD_SPEED_HIGH_IN_FULL       USBD_HSINFS_SPEED
#define PCD_SPEED_FULL               USBD_FS_SPEED
/**
  * @}
  */

/** @defgroup PCD_PHY_Module PCD PHY Module
  * @{
  */
#define PCD_PHY_ULPI                 1U
#define PCD_PHY_EMBEDDED             2U
#define PCD_PHY_UTMI                 3U
/**
  * @}
  */

/** @defgroup PCD_Error_Code_definition PCD Error Code definition
  * @brief  PCD Error Code definition
  * @{
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
#define  HAL_PCD_ERROR_INVALID_CALLBACK                        (0x00000010U)    /*!< Invalid Callback error  */
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup PCD_Exported_Macros PCD Exported Macros
 *  @brief macros to handle interrupts and specific clock configurations
 * @{
 */
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
#define __HAL_PCD_ENABLE()                       (void)USB_EnableGlobalInt ()
#define __HAL_PCD_DISABLE()                      (void)USB_DisableGlobalInt ()

#define __HAL_PCD_GET_FLAG(__INTERRUPT__)      ((USB_ReadInterrupts() & (__INTERRUPT__)) == (__INTERRUPT__))
#define __HAL_PCD_CLEAR_FLAG(__INTERRUPT__)    ((USB_OTG_FS->GINTSTS) &=  (__INTERRUPT__))
#define __HAL_PCD_IS_INVALID_INTERRUPT()         (USB_ReadInterrupts() == 0U)


#define __HAL_PCD_UNGATE_PHYCLOCK()             *(__IO uint32_t *)((uint32_t)(USB_OTG_FS) + USB_OTG_PCGCCTL_BASE) &= \
                                                          ~(USB_OTG_PCGCCTL_STOPCLK)

#define __HAL_PCD_GATE_PHYCLOCK()               *(__IO uint32_t *)((uint32_t)(USB_OTG_FS) + USB_OTG_PCGCCTL_BASE) |= USB_OTG_PCGCCTL_STOPCLK

#define __HAL_PCD_IS_PHY_SUSPENDED()            ((*(__IO uint32_t *)((uint32_t)(USB_OTG_FS) + USB_OTG_PCGCCTL_BASE)) & 0x10U)

#define __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_IT()    EXTI->IMR |= (USB_OTG_HS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_HS_WAKEUP_EXTI_DISABLE_IT()   EXTI->IMR &= ~(USB_OTG_HS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_HS_WAKEUP_EXTI_GET_FLAG()     EXTI->PR & (USB_OTG_HS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_HS_WAKEUP_EXTI_CLEAR_FLAG()   EXTI->PR = (USB_OTG_HS_WAKEUP_EXTI_LINE)

#define __HAL_USB_OTG_HS_WAKEUP_EXTI_ENABLE_RISING_EDGE() \
                        do { \
                             EXTI->FTSR &= ~(USB_OTG_HS_WAKEUP_EXTI_LINE); \
                             EXTI->RTSR |= USB_OTG_HS_WAKEUP_EXTI_LINE; \
                           } while(0U)
#define __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_IT()    EXTI->IMR |= USB_OTG_FS_WAKEUP_EXTI_LINE
#define __HAL_USB_OTG_FS_WAKEUP_EXTI_DISABLE_IT()   EXTI->IMR &= ~(USB_OTG_FS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_FS_WAKEUP_EXTI_GET_FLAG()     EXTI->PR & (USB_OTG_FS_WAKEUP_EXTI_LINE)
#define __HAL_USB_OTG_FS_WAKEUP_EXTI_CLEAR_FLAG()   EXTI->PR = USB_OTG_FS_WAKEUP_EXTI_LINE

#define __HAL_USB_OTG_FS_WAKEUP_EXTI_ENABLE_RISING_EDGE() \
                        do { \
                             EXTI->FTSR &= ~(USB_OTG_FS_WAKEUP_EXTI_LINE); \
                             EXTI->RTSR |= USB_OTG_FS_WAKEUP_EXTI_LINE; \
                           } while(0U)
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */


/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup PCD_Exported_Functions PCD Exported Functions
  * @{
  */

/* Initialization/de-initialization functions  ********************************/
/** @addtogroup PCD_Exported_Functions_Group1 Initialization and de-initialization functions
  * @{
  */
HAL_StatusTypeDef HAL_PCD_Init();
HAL_StatusTypeDef HAL_PCD_DeInit();
void HAL_PCD_MspInit();
void HAL_PCD_MspDeInit();

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
/** @defgroup HAL_PCD_Callback_ID_enumeration_definition HAL USB OTG PCD Callback ID enumeration definition
  * @brief  HAL USB OTG PCD Callback ID enumeration definition
  * @{
  */
typedef enum
{
  HAL_PCD_SOF_CB_ID          = 0x01,      /*!< USB PCD SOF callback ID          */
  HAL_PCD_SETUPSTAGE_CB_ID   = 0x02,      /*!< USB PCD Setup Stage callback ID  */
  HAL_PCD_RESET_CB_ID        = 0x03,      /*!< USB PCD Reset callback ID        */
  HAL_PCD_SUSPEND_CB_ID      = 0x04,      /*!< USB PCD Suspend callback ID      */
  HAL_PCD_RESUME_CB_ID       = 0x05,      /*!< USB PCD Resume callback ID       */
  HAL_PCD_CONNECT_CB_ID      = 0x06,      /*!< USB PCD Connect callback ID      */
  HAL_PCD_DISCONNECT_CB_ID   = 0x07,      /*!< USB PCD Disconnect callback ID   */

  HAL_PCD_MSPINIT_CB_ID      = 0x08,      /*!< USB PCD MspInit callback ID      */
  HAL_PCD_MSPDEINIT_CB_ID    = 0x09       /*!< USB PCD MspDeInit callback ID    */

} HAL_PCD_CallbackIDTypeDef;
/**
  * @}
  */

/** @defgroup HAL_PCD_Callback_pointer_definition HAL USB OTG PCD Callback pointer definition
  * @brief  HAL USB OTG PCD Callback pointer definition
  * @{
  */

typedef void (*pPCD_CallbackTypeDef)(PCD_HandleTypeDef *hpcd);                                   /*!< pointer to a common USB OTG PCD callback function  */
typedef void (*pPCD_DataOutStageCallbackTypeDef)(PCD_HandleTypeDef *hpcd, uint8_t epnum);        /*!< pointer to USB OTG PCD Data OUT Stage callback     */
typedef void (*pPCD_DataInStageCallbackTypeDef)(PCD_HandleTypeDef *hpcd, uint8_t epnum);         /*!< pointer to USB OTG PCD Data IN Stage callback      */
typedef void (*pPCD_IsoOutIncpltCallbackTypeDef)(PCD_HandleTypeDef *hpcd, uint8_t epnum);        /*!< pointer to USB OTG PCD ISO OUT Incomplete callback */
typedef void (*pPCD_IsoInIncpltCallbackTypeDef)(PCD_HandleTypeDef *hpcd, uint8_t epnum);         /*!< pointer to USB OTG PCD ISO IN Incomplete callback  */
typedef void (*pPCD_LpmCallbackTypeDef)(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);        /*!< pointer to USB OTG PCD LPM callback                */
typedef void (*pPCD_BcdCallbackTypeDef)(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);        /*!< pointer to USB OTG PCD BCD callback                */

/**
  * @}
  */

HAL_StatusTypeDef HAL_PCD_RegisterCallback(PCD_HandleTypeDef *hpcd, HAL_PCD_CallbackIDTypeDef CallbackID, pPCD_CallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_PCD_UnRegisterCallback(PCD_HandleTypeDef *hpcd, HAL_PCD_CallbackIDTypeDef CallbackID);

HAL_StatusTypeDef HAL_PCD_RegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd, pPCD_DataOutStageCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_PCD_UnRegisterDataOutStageCallback(PCD_HandleTypeDef *hpcd);

HAL_StatusTypeDef HAL_PCD_RegisterDataInStageCallback(PCD_HandleTypeDef *hpcd, pPCD_DataInStageCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_PCD_UnRegisterDataInStageCallback(PCD_HandleTypeDef *hpcd);

HAL_StatusTypeDef HAL_PCD_RegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd, pPCD_IsoOutIncpltCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_PCD_UnRegisterIsoOutIncpltCallback(PCD_HandleTypeDef *hpcd);

HAL_StatusTypeDef HAL_PCD_RegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd, pPCD_IsoInIncpltCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_PCD_UnRegisterIsoInIncpltCallback(PCD_HandleTypeDef *hpcd);

HAL_StatusTypeDef HAL_PCD_RegisterBcdCallback(PCD_HandleTypeDef *hpcd, pPCD_BcdCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_PCD_UnRegisterBcdCallback(PCD_HandleTypeDef *hpcd);

HAL_StatusTypeDef HAL_PCD_RegisterLpmCallback(PCD_HandleTypeDef *hpcd, pPCD_LpmCallbackTypeDef pCallback);
HAL_StatusTypeDef HAL_PCD_UnRegisterLpmCallback(PCD_HandleTypeDef *hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
/**
  * @}
  */

/* I/O operation functions  ***************************************************/
/* Non-Blocking mode: Interrupt */
/** @addtogroup PCD_Exported_Functions_Group2 Input and Output operation functions
  * @{
  */
HAL_StatusTypeDef HAL_PCD_Start();
HAL_StatusTypeDef HAL_PCD_Stop();
void HAL_PCD_IRQHandler();

void HAL_PCD_SOFCallback();
void HAL_PCD_SetupStageCallback();
void HAL_PCD_ResetCallback();
void HAL_PCD_SuspendCallback();
void HAL_PCD_ResumeCallback();
void HAL_PCD_ConnectCallback();
void HAL_PCD_DisconnectCallback();

void HAL_PCD_DataOutStageCallback(uint8_t epnum);
void HAL_PCD_DataInStageCallback(uint8_t epnum);
void HAL_PCD_ISOOUTIncompleteCallback(uint8_t epnum);
void HAL_PCD_ISOINIncompleteCallback(uint8_t epnum);
/**
  * @}
  */

/* Peripheral Control functions  **********************************************/
/** @addtogroup PCD_Exported_Functions_Group3 Peripheral Control functions
  * @{
  */
HAL_StatusTypeDef HAL_PCD_DevConnect();
HAL_StatusTypeDef HAL_PCD_DevDisconnect();
HAL_StatusTypeDef HAL_PCD_SetAddress(uint8_t address);
HAL_StatusTypeDef HAL_PCD_EP_Open(uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
HAL_StatusTypeDef HAL_PCD_EP_Close(uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Receive(uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
HAL_StatusTypeDef HAL_PCD_EP_Transmit(uint8_t ep_addr, const uint8_t *pBuf, uint32_t len);
uint32_t          HAL_PCD_EP_GetRxCount(uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_SetStall(uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Flush(uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup();
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup();
/**
  * @}
  */

/* Peripheral State functions  ************************************************/
/** @addtogroup PCD_Exported_Functions_Group4 Peripheral State functions
  * @{
  */
PCD_StateTypeDef HAL_PCD_GetState();
/**
  * @}
  */

/**
  * @}
  */

/* Private constants ---------------------------------------------------------*/
/** @defgroup PCD_Private_Constants PCD Private Constants
  * @{
  */
/** @defgroup USB_EXTI_Line_Interrupt USB EXTI line interrupt
  * @{
  */
#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
#define USB_OTG_FS_WAKEUP_EXTI_LINE                                   (0x1U << 18)  /*!< USB FS EXTI Line WakeUp Interrupt */
#define USB_OTG_HS_WAKEUP_EXTI_LINE                                   (0x1U << 20)  /*!< USB HS EXTI Line WakeUp Interrupt */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */


/**
  * @}
  */
/**
  * @}
  */

#if defined (USB_OTG_FS) || defined (USB_OTG_HS)
#ifndef USB_OTG_DOEPINT_OTEPSPR
#define USB_OTG_DOEPINT_OTEPSPR                (0x1UL << 5)      /*!< Status Phase Received interrupt */
#endif

#ifndef USB_OTG_DOEPMSK_OTEPSPRM
#define USB_OTG_DOEPMSK_OTEPSPRM               (0x1UL << 5)      /*!< Setup Packet Received interrupt mask */
#endif

#ifndef USB_OTG_DOEPINT_NAK
#define USB_OTG_DOEPINT_NAK                    (0x1UL << 13)      /*!< NAK interrupt */
#endif

#ifndef USB_OTG_DOEPMSK_NAKM
#define USB_OTG_DOEPMSK_NAKM                   (0x1UL << 13)      /*!< OUT Packet NAK interrupt mask */
#endif

#ifndef USB_OTG_DOEPINT_STPKTRX
#define USB_OTG_DOEPINT_STPKTRX                (0x1UL << 15)      /*!< Setup Packet Received interrupt */
#endif

#ifndef USB_OTG_DOEPMSK_NYETM
#define USB_OTG_DOEPMSK_NYETM                  (0x1UL << 14)      /*!< Setup Packet Received interrupt mask */
#endif
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

/* Private macros ------------------------------------------------------------*/
/** @defgroup PCD_Private_Macros PCD Private Macros
 * @{
 */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

#ifdef __cplusplus
}
#endif

#endif /* STM32F4xx_HAL_PCD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
