/**
  ******************************************************************************
  * @file    stm32f4xx_hal_pcd.c
  * @author  MCD Application Team
  * @brief   PCD HAL module driver.
  *          This file provides firmware functions to manage the following
  *          functionalities of the USB Peripheral Controller:
  *           + Initialization and de-initialization functions
  *           + IO operation functions
  *           + Peripheral Control functions
  *           + Peripheral State functions
  *
  @verbatim
  ==============================================================================
                    ##### How to use this driver #####
  ==============================================================================
    [..]
      The PCD HAL driver can be used as follows:

     (#) Declare a PCD_HandleTypeDef handle structure, for example:
         PCD_HandleTypeDef  hpcd;

     (#) Fill parameters of Init structure in HCD handle

     (#) Call HAL_PCD_Init() API to initialize the PCD peripheral (Core, Device core, ...)

     (#) Initialize the PCD low level resources through the HAL_PCD_MspInit() API:
         (##) Enable the PCD/USB Low Level interface clock using
              (+++) __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
              (+++) __HAL_RCC_USB_OTG_HS_CLK_ENABLE(); (For High Speed Mode)

         (##) Initialize the related GPIO clocks
         (##) Configure PCD pin-out
         (##) Configure PCD NVIC interrupt

     (#)Associate the Upper USB device stack to the HAL PCD Driver:
         (##) hpcd.pData = pdev;

     (#)Enable PCD transmission and reception:
         (##) HAL_PCD_Start();

  @endverbatim
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/** @addtogroup STM32F4xx_HAL_Driver
  * @{
  */

/** @defgroup PCD PCD
  * @brief PCD HAL module driver
  * @{
  */

#ifdef HAL_PCD_MODULE_ENABLED

#if defined(USB_OTG_FS) || defined(USB_OTG_HS)

/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
//const uint32_t USBx_BASE = USB_OTG_FS_PERIPH_BASE;
#define USBx_BASE USB_OTG_FS_PERIPH_BASE
/* Private macros ------------------------------------------------------------*/
/** @defgroup PCD_Private_Macros PCD Private Macros
  * @{
  */
#define PCD_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define PCD_MAX(a, b) (((a) > (b)) ? (a) : (b))
/**
  * @}
  */

/* Private functions prototypes ----------------------------------------------*/
/** @defgroup PCD_Private_Functions PCD Private Functions
  * @{
  */
#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
static HAL_StatusTypeDef PCD_WriteEmptyTxFifo(uint32_t epnum);
static HAL_StatusTypeDef PCD_EP_OutXfrComplete_int(uint32_t epnum);
static HAL_StatusTypeDef PCD_EP_OutSetupPacket_int(uint32_t epnum);
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */
/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @defgroup PCD_Exported_Functions PCD Exported Functions
  * @{
  */

/** @defgroup PCD_Exported_Functions_Group1 Initialization and de-initialization functions
 *  @brief    Initialization and Configuration functions
 *
@verbatim
 ===============================================================================
            ##### Initialization and de-initialization functions #####
 ===============================================================================
    [..]  This section provides functions allowing to:

@endverbatim
  * @{
  */

/**
  * @brief  DeInitializes the PCD peripheral.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeInit()
{
  /* Check the PCD handle allocation */
  //if (hpcd == NULL)
  //{
  //  return HAL_ERROR;
  //}

  hpcd_USB_OTG_FS.State = HAL_PCD_STATE_BUSY;

  /* Stop Device */
  (void)HAL_PCD_Stop();

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  if (hpcd_USB_OTG_FS.MspDeInitCallback == NULL)
  {
    hpcd_USB_OTG_FS.MspDeInitCallback = HAL_PCD_MspDeInit; /* Legacy weak MspDeInit  */
  }

  /* DeInit the low level hardware */
  hpcd_USB_OTG_FS.MspDeInitCallback(hpcd);
#else
  /* DeInit the low level hardware: CLOCK, NVIC.*/
  //HAL_PCD_MspDeInit(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

  hpcd_USB_OTG_FS.State = HAL_PCD_STATE_RESET;

  return HAL_OK;
}

/**
  * @brief  Initializes the PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspInit()
{
}

/**
  * @brief  DeInitializes PCD MSP.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_MspDeInit()
{
}

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
/**
  * @brief  Register a User USB PCD Callback
  *         To be used instead of the weak predefined callback
  * @param  hpcd USB PCD handle
  * @param  CallbackID ID of the callback to be registered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_PCD_SOF_CB_ID USB PCD SOF callback ID
  *          @arg @ref HAL_PCD_SETUPSTAGE_CB_ID USB PCD Setup callback ID
  *          @arg @ref HAL_PCD_RESET_CB_ID USB PCD Reset callback ID
  *          @arg @ref HAL_PCD_SUSPEND_CB_ID USB PCD Suspend callback ID
  *          @arg @ref HAL_PCD_RESUME_CB_ID USB PCD Resume callback ID
  *          @arg @ref HAL_PCD_CONNECT_CB_ID USB PCD Connect callback ID
  *          @arg @ref HAL_PCD_DISCONNECT_CB_ID OTG PCD Disconnect callback ID
  *          @arg @ref HAL_PCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref HAL_PCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @param  pCallback pointer to the Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterCallback(HAL_PCD_CallbackIDTypeDef CallbackID, pPCD_CallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
    return HAL_ERROR;
  }
  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    switch (CallbackID)
    {
    case HAL_PCD_SOF_CB_ID:
      hpcd_USB_OTG_FS.SOFCallback = pCallback;
      break;

    case HAL_PCD_SETUPSTAGE_CB_ID:
      hpcd_USB_OTG_FS.SetupStageCallback = pCallback;
      break;

    case HAL_PCD_RESET_CB_ID:
      hpcd_USB_OTG_FS.ResetCallback = pCallback;
      break;

    case HAL_PCD_SUSPEND_CB_ID:
      hpcd_USB_OTG_FS.SuspendCallback = pCallback;
      break;

    case HAL_PCD_RESUME_CB_ID:
      hpcd_USB_OTG_FS.ResumeCallback = pCallback;
      break;

    case HAL_PCD_CONNECT_CB_ID:
      hpcd_USB_OTG_FS.ConnectCallback = pCallback;
      break;

    case HAL_PCD_DISCONNECT_CB_ID:
      hpcd_USB_OTG_FS.DisconnectCallback = pCallback;
      break;

    case HAL_PCD_MSPINIT_CB_ID:
      hpcd_USB_OTG_FS.MspInitCallback = pCallback;
      break;

    case HAL_PCD_MSPDEINIT_CB_ID:
      hpcd_USB_OTG_FS.MspDeInitCallback = pCallback;
      break;

    default:
      /* Update the error code */
      hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
      /* Return error status */
      status = HAL_ERROR;
      break;
    }
  }
  else if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_RESET)
  {
    switch (CallbackID)
    {
    case HAL_PCD_MSPINIT_CB_ID:
      hpcd_USB_OTG_FS.MspInitCallback = pCallback;
      break;

    case HAL_PCD_MSPDEINIT_CB_ID:
      hpcd_USB_OTG_FS.MspDeInitCallback = pCallback;
      break;

    default:
      /* Update the error code */
      hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
      /* Return error status */
      status = HAL_ERROR;
      break;
    }
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;
    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);
  return status;
}

/**
  * @brief  Unregister an USB PCD Callback
  *         USB PCD callabck is redirected to the weak predefined callback
  * @param  hpcd USB PCD handle
  * @param  CallbackID ID of the callback to be unregistered
  *         This parameter can be one of the following values:
  *          @arg @ref HAL_PCD_SOF_CB_ID USB PCD SOF callback ID
  *          @arg @ref HAL_PCD_SETUPSTAGE_CB_ID USB PCD Setup callback ID
  *          @arg @ref HAL_PCD_RESET_CB_ID USB PCD Reset callback ID
  *          @arg @ref HAL_PCD_SUSPEND_CB_ID USB PCD Suspend callback ID
  *          @arg @ref HAL_PCD_RESUME_CB_ID USB PCD Resume callback ID
  *          @arg @ref HAL_PCD_CONNECT_CB_ID USB PCD Connect callback ID
  *          @arg @ref HAL_PCD_DISCONNECT_CB_ID OTG PCD Disconnect callback ID
  *          @arg @ref HAL_PCD_MSPINIT_CB_ID MspDeInit callback ID
  *          @arg @ref HAL_PCD_MSPDEINIT_CB_ID MspDeInit callback ID
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterCallback(HAL_PCD_CallbackIDTypeDef CallbackID)
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  /* Setup Legacy weak Callbacks  */
  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    switch (CallbackID)
    {
    case HAL_PCD_SOF_CB_ID:
      hpcd_USB_OTG_FS.SOFCallback = HAL_PCD_SOFCallback;
      break;

    case HAL_PCD_SETUPSTAGE_CB_ID:
      hpcd_USB_OTG_FS.SetupStageCallback = HAL_PCD_SetupStageCallback;
      break;

    case HAL_PCD_RESET_CB_ID:
      hpcd_USB_OTG_FS.ResetCallback = HAL_PCD_ResetCallback;
      break;

    case HAL_PCD_SUSPEND_CB_ID:
      hpcd_USB_OTG_FS.SuspendCallback = HAL_PCD_SuspendCallback;
      break;

    case HAL_PCD_RESUME_CB_ID:
      hpcd_USB_OTG_FS.ResumeCallback = HAL_PCD_ResumeCallback;
      break;

    case HAL_PCD_CONNECT_CB_ID:
      hpcd_USB_OTG_FS.ConnectCallback = HAL_PCD_ConnectCallback;
      break;

    case HAL_PCD_DISCONNECT_CB_ID:
      hpcd_USB_OTG_FS.DisconnectCallback = HAL_PCD_DisconnectCallback;
      break;

    case HAL_PCD_MSPINIT_CB_ID:
      hpcd_USB_OTG_FS.MspInitCallback = HAL_PCD_MspInit;
      break;

    case HAL_PCD_MSPDEINIT_CB_ID:
      hpcd_USB_OTG_FS.MspDeInitCallback = HAL_PCD_MspDeInit;
      break;

    default:
      /* Update the error code */
      hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

      /* Return error status */
      status = HAL_ERROR;
      break;
    }
  }
  else if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_RESET)
  {
    switch (CallbackID)
    {
    case HAL_PCD_MSPINIT_CB_ID:
      hpcd_USB_OTG_FS.MspInitCallback = HAL_PCD_MspInit;
      break;

    case HAL_PCD_MSPDEINIT_CB_ID:
      hpcd_USB_OTG_FS.MspDeInitCallback = HAL_PCD_MspDeInit;
      break;

    default:
      /* Update the error code */
      hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

      /* Return error status */
      status = HAL_ERROR;
      break;
    }
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);
  return status;
}

/**
  * @brief  Register USB PCD Data OUT Stage Callback
  *         To be used instead of the weak HAL_PCD_DataOutStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Data OUT Stage Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterDataOutStageCallback(pPCD_DataOutStageCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.DataOutStageCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  UnRegister the USB PCD Data OUT Stage Callback
  *         USB PCD Data OUT Stage Callback is redirected to the weak HAL_PCD_DataOutStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterDataOutStageCallback()
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.DataOutStageCallback = HAL_PCD_DataOutStageCallback; /* Legacy weak DataOutStageCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  Register USB PCD Data IN Stage Callback
  *         To be used instead of the weak HAL_PCD_DataInStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Data IN Stage Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterDataInStageCallback(pPCD_DataInStageCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.DataInStageCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  UnRegister the USB PCD Data IN Stage Callback
  *         USB PCD Data OUT Stage Callback is redirected to the weak HAL_PCD_DataInStageCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterDataInStageCallback()
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.DataInStageCallback = HAL_PCD_DataInStageCallback; /* Legacy weak DataInStageCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  Register USB PCD Iso OUT incomplete Callback
  *         To be used instead of the weak HAL_PCD_ISOOUTIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Iso OUT incomplete Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterIsoOutIncpltCallback(pPCD_IsoOutIncpltCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.ISOOUTIncompleteCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  UnRegister the USB PCD Iso OUT incomplete Callback
  *         USB PCD Iso OUT incomplete Callback is redirected to the weak HAL_PCD_ISOOUTIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterIsoOutIncpltCallback()
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.ISOOUTIncompleteCallback = HAL_PCD_ISOOUTIncompleteCallback; /* Legacy weak ISOOUTIncompleteCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  Register USB PCD Iso IN incomplete Callback
  *         To be used instead of the weak HAL_PCD_ISOINIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD Iso IN incomplete Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterIsoInIncpltCallback(pPCD_IsoInIncpltCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.ISOINIncompleteCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  UnRegister the USB PCD Iso IN incomplete Callback
  *         USB PCD Iso IN incomplete Callback is redirected to the weak HAL_PCD_ISOINIncompleteCallback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterIsoInIncpltCallback()
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.ISOINIncompleteCallback = HAL_PCD_ISOINIncompleteCallback; /* Legacy weak ISOINIncompleteCallback  */
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  Register USB PCD BCD Callback
  *         To be used instead of the weak HAL_PCDEx_BCD_Callback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD BCD Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterBcdCallback(pPCD_BcdCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.BCDCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  UnRegister the USB PCD BCD Callback
  *         USB BCD Callback is redirected to the weak HAL_PCDEx_BCD_Callback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterBcdCallback()
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.BCDCallback = HAL_PCDEx_BCD_Callback; /* Legacy weak HAL_PCDEx_BCD_Callback  */
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  Register USB PCD LPM Callback
  *         To be used instead of the weak HAL_PCDEx_LPM_Callback() predefined callback
  * @param  hpcd PCD handle
  * @param  pCallback pointer to the USB PCD LPM Callback function
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_RegisterLpmCallback(pPCD_LpmCallbackTypeDef pCallback)
{
  HAL_StatusTypeDef status = HAL_OK;

  if (pCallback == NULL)
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    return HAL_ERROR;
  }

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.LPMCallback = pCallback;
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}

/**
  * @brief  UnRegister the USB PCD LPM Callback
  *         USB LPM Callback is redirected to the weak HAL_PCDEx_LPM_Callback() predefined callback
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_UnRegisterLpmCallback()
{
  HAL_StatusTypeDef status = HAL_OK;

  /* Process locked */
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if (hpcd_USB_OTG_FS.State == HAL_PCD_STATE_READY)
  {
    hpcd_USB_OTG_FS.LPMCallback = HAL_PCDEx_LPM_Callback; /* Legacy weak HAL_PCDEx_LPM_Callback  */
  }
  else
  {
    /* Update the error code */
    hpcd_USB_OTG_FS.ErrorCode |= HAL_PCD_ERROR_INVALID_CALLBACK;

    /* Return error status */
    status = HAL_ERROR;
  }

  /* Release Lock */
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return status;
}
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group2 Input and Output operation functions
 *  @brief   Data transfers functions
 *
@verbatim
 ===============================================================================
                      ##### IO operation functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to manage the PCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Start the USB device
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Start()
{
#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

  __HAL_LOCK(&hpcd_USB_OTG_FS);
  (void)USB_DevConnect();
  __HAL_PCD_ENABLE();
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);
  return HAL_OK;
}

/**
  * @brief  Stop the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_Stop()
{
  __HAL_LOCK(&hpcd_USB_OTG_FS);
  __HAL_PCD_DISABLE();

  if (USB_StopDevice() != HAL_OK)
  {
    __HAL_UNLOCK(&hpcd_USB_OTG_FS);
    return HAL_ERROR;
  }

  (void)USB_DevDisconnect();
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return HAL_OK;
}
#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
/**
  * @brief  Handles PCD interrupt request.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
void HAL_PCD_IRQHandler()
{
  uint32_t i, ep_intr, epint, epnum;
  uint32_t fifoemptymsk, temp;
  USB_OTG_EPTypeDef *ep;

  /* ensure that we are in device mode */
  if (USB_GetMode() == USB_OTG_MODE_DEVICE)
  {
    /* avoid spurious interrupt */
    if (__HAL_PCD_IS_INVALID_INTERRUPT())
    {
      return;
    }

    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_MMIS))
    {
      /* incorrect mode, acknowledge the interrupt */
      __HAL_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_MMIS);
    }

    /* Handle RxQLevel Interrupt */
    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_RXFLVL))
    {
      USB_MASK_INTERRUPT(USB_OTG_GINTSTS_RXFLVL);

      temp = USB_OTG_FS->GRXSTSP;

      ep = &hpcd_USB_OTG_FS.OUT_ep[temp & USB_OTG_GRXSTSP_EPNUM];

      if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_DATA_UPDT)
      {
        if ((temp & USB_OTG_GRXSTSP_BCNT) != 0U)
        {
          (void)USB_ReadPacket(ep->xfer_buff,
                               (uint16_t)((temp & USB_OTG_GRXSTSP_BCNT) >> 4));

          ep->xfer_buff += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
          ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
        }
      }
      else if (((temp & USB_OTG_GRXSTSP_PKTSTS) >> 17) == STS_SETUP_UPDT)
      {
        (void)USB_ReadPacket((uint8_t *)hpcd_USB_OTG_FS.Setup, 8U);
        ep->xfer_count += (temp & USB_OTG_GRXSTSP_BCNT) >> 4;
      }
      else
      {
        /* ... */
      }
      USB_UNMASK_INTERRUPT(USB_OTG_GINTSTS_RXFLVL);
    }

    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_OEPINT))
    {
      epnum = 0U;

      /* Read in the device interrupt bits */
      ep_intr = USB_ReadDevAllOutEpInterrupt();

      while (ep_intr != 0U)
      {
        if ((ep_intr & 0x1U) != 0U)
        {
          epint = USB_ReadDevOutEPInterrupt((uint8_t)epnum);

          if ((epint & USB_OTG_DOEPINT_XFRC) == USB_OTG_DOEPINT_XFRC)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_XFRC);
            (void)PCD_EP_OutXfrComplete_int(epnum);
          }

          if ((epint & USB_OTG_DOEPINT_STUP) == USB_OTG_DOEPINT_STUP)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STUP);
            /* Class B setup phase done for previous decoded setup */
            (void)PCD_EP_OutSetupPacket_int(epnum);
          }

          if ((epint & USB_OTG_DOEPINT_OTEPDIS) == USB_OTG_DOEPINT_OTEPDIS)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPDIS);
          }

          /* Clear Status Phase Received interrupt */
          if ((epint & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
          }

          /* Clear OUT NAK interrupt */
          if ((epint & USB_OTG_DOEPINT_NAK) == USB_OTG_DOEPINT_NAK)
          {
            CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_NAK);
          }
        }
        epnum++;
        ep_intr >>= 1U;
      }
    }

    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_IEPINT))
    {
      /* Read in the device interrupt bits */
      ep_intr = USB_ReadDevAllInEpInterrupt();

      epnum = 0U;

      while (ep_intr != 0U)
      {
        if ((ep_intr & 0x1U) != 0U) /* In ITR */
        {
          epint = USB_ReadDevInEPInterrupt((uint8_t)epnum);

          if ((epint & USB_OTG_DIEPINT_XFRC) == USB_OTG_DIEPINT_XFRC)
          {
            fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
            USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;

            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_XFRC);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
            hpcd_USB_OTG_FS.DataInStageCallback(hpcd, (uint8_t)epnum);
#else
            HAL_PCD_DataInStageCallback((uint8_t)epnum);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
          }
          if ((epint & USB_OTG_DIEPINT_TOC) == USB_OTG_DIEPINT_TOC)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_TOC);
          }
          if ((epint & USB_OTG_DIEPINT_ITTXFE) == USB_OTG_DIEPINT_ITTXFE)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_ITTXFE);
          }
          if ((epint & USB_OTG_DIEPINT_INEPNE) == USB_OTG_DIEPINT_INEPNE)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_INEPNE);
          }
          if ((epint & USB_OTG_DIEPINT_EPDISD) == USB_OTG_DIEPINT_EPDISD)
          {
            CLEAR_IN_EP_INTR(epnum, USB_OTG_DIEPINT_EPDISD);
          }
          if ((epint & USB_OTG_DIEPINT_TXFE) == USB_OTG_DIEPINT_TXFE)
          {
            (void)PCD_WriteEmptyTxFifo(epnum);
          }
        }
        epnum++;
        ep_intr >>= 1U;
      }
    }

    /* Handle Resume Interrupt */
    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_WKUINT))
    {
      /* Clear the Remote Wake-up Signaling */
      USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;

      if (hpcd_USB_OTG_FS.LPM_State == LPM_L1)
      {
        hpcd_USB_OTG_FS.LPM_State = LPM_L0;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd_USB_OTG_FS.LPMCallback(hpcd, PCD_LPM_L0_ACTIVE);
#else
        HAL_PCDEx_LPM_Callback(PCD_LPM_L0_ACTIVE);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
      else
      {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd_USB_OTG_FS.ResumeCallback(hpcd);
#else
        HAL_PCD_ResumeCallback();
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }

      __HAL_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_WKUINT);
    }

    /* Handle Suspend Interrupt */
    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_USBSUSP))
    {
      if ((USBx_DEVICE->DSTS & USB_OTG_DSTS_SUSPSTS) == USB_OTG_DSTS_SUSPSTS)
      {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd_USB_OTG_FS.SuspendCallback(hpcd);
#else
        HAL_PCD_SuspendCallback();
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
      __HAL_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_USBSUSP);
    }
#if defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
    /* Handle LPM Interrupt */
    if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_LPMINT))
    {
      __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_LPMINT);

      if (hpcd_USB_OTG_FS.LPM_State == LPM_L0)
      {
        hpcd_USB_OTG_FS.LPM_State = LPM_L1;
        hpcd_USB_OTG_FS.BESL = (USB_OTG_FS->GLPMCFG & USB_OTG_GLPMCFG_BESL) >> 2U;

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd_USB_OTG_FS.LPMCallback(hpcd, PCD_LPM_L1_ACTIVE);
#else
        HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L1_ACTIVE);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
      else
      {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd_USB_OTG_FS.SuspendCallback(hpcd);
#else
        HAL_PCD_SuspendCallback(hpcd);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
    }
#endif /* defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx) */
    /* Handle Reset Interrupt */
    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_USBRST))
    {
      USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_RWUSIG;
      (void)USB_FlushTxFifo(0x10U);

      for (i = 0U; i < 4; i++)
      {
        USBx_INEP(i)->DIEPINT = 0xFB7FU;
        USBx_INEP(i)->DIEPCTL &= ~USB_OTG_DIEPCTL_STALL;
        USBx_INEP(i)->DIEPCTL |= USB_OTG_DIEPCTL_SNAK;
        USBx_OUTEP(i)->DOEPINT = 0xFB7FU;
        USBx_OUTEP(i)->DOEPCTL &= ~USB_OTG_DOEPCTL_STALL;
        USBx_OUTEP(i)->DOEPCTL |= USB_OTG_DOEPCTL_SNAK;
      }
      USBx_DEVICE->DAINTMSK |= 0x10001U;

        USBx_DEVICE->DOEPMSK |= USB_OTG_DOEPMSK_STUPM |
                                USB_OTG_DOEPMSK_XFRCM |
                                USB_OTG_DOEPMSK_EPDM |
                                USB_OTG_DOEPMSK_OTEPSPRM |
                                USB_OTG_DOEPMSK_NAKM;

        USBx_DEVICE->DIEPMSK |= USB_OTG_DIEPMSK_TOM |
                                USB_OTG_DIEPMSK_XFRCM |
                                USB_OTG_DIEPMSK_EPDM;

      /* Set Default Address to 0 */
      USBx_DEVICE->DCFG &= ~USB_OTG_DCFG_DAD;

      /* setup EP0 to receive SETUP packets */
      (void)USB_EP0_OutStart();

      __HAL_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_USBRST);
    }

    /* Handle Enumeration done Interrupt */
    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_ENUMDNE))
    {
      (void)USB_ActivateSetup();
      //hpcd_USB_OTG_FS.Init.speed = USB_GetDevSpeed();

      /* Set USB Turnaround time */
      (void)USB_SetTurnaroundTime(SystemCoreClock,
                                  USB_OTG_SPEED_FULL);

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd_USB_OTG_FS.ResetCallback(hpcd);
#else
      HAL_PCD_ResetCallback();
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      __HAL_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_ENUMDNE);
    }

    /* Handle SOF Interrupt */
    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_SOF))
    {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd_USB_OTG_FS.SOFCallback(hpcd);
#else
      // not supported
      // HAL_PCD_SOFCallback();
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      __HAL_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_SOF);
    }

    /* Handle Incomplete ISO IN Interrupt */
    //     if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_IISOIXFR))
    //     {
    //       /* Keep application checking the corresponding Iso IN endpoint
    //       causing the incomplete Interrupt */
    //       epnum = 0U;

    // #if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    //       hpcd_USB_OTG_FS.ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
    // #else
    //       HAL_PCD_ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
    // #endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    //       __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_IISOIXFR);
    //     }

    //     /* Handle Incomplete ISO OUT Interrupt */
    //     if (__HAL_PCD_GET_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT))
    //     {
    //       /* Keep application checking the corresponding Iso OUT endpoint
    //       causing the incomplete Interrupt */
    //       epnum = 0U;

    // #if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    //       hpcd_USB_OTG_FS.ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
    // #else
    //       HAL_PCD_ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
    // #endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

    //       __HAL_PCD_CLEAR_FLAG(hpcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
    //     }

    /* Handle Connection event Interrupt */
    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_SRQINT))
    {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd_USB_OTG_FS.ConnectCallback(hpcd);
#else
      HAL_PCD_ConnectCallback();
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

      __HAL_PCD_CLEAR_FLAG(USB_OTG_GINTSTS_SRQINT);
    }

    /* Handle Disconnection event Interrupt */
    if (__HAL_PCD_GET_FLAG(USB_OTG_GINTSTS_OTGINT))
    {
      temp = USB_OTG_FS->GOTGINT;

      if ((temp & USB_OTG_GOTGINT_SEDET) == USB_OTG_GOTGINT_SEDET)
      {
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
        hpcd_USB_OTG_FS.DisconnectCallback(hpcd);
#else
        HAL_PCD_DisconnectCallback();
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
      }
      USB_OTG_FS->GOTGINT |= temp;
    }
  }
}
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

/**
  * @brief  Data OUT stage callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_DataOutStageCallback(uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */

  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_DataOutStageCallback could be implemented in the user file
   */
}

/**
  * @brief  Data IN stage callback
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_DataInStageCallback(uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */

  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_DataInStageCallback could be implemented in the user file
   */
}
/**
  * @brief  Setup stage callback
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SetupStageCallback()
{
  /* Prevent unused argument(s) compilation warning */

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_SetupStageCallback could be implemented in the user file
   */
}

/**
  * @brief  USB Start Of Frame callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SOFCallback()
{
  /* Prevent unused argument(s) compilation warning */

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_SOFCallback could be implemented in the user file
   */
}

/**
  * @brief  USB Reset callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ResetCallback()
{
  /* Prevent unused argument(s) compilation warning */

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ResetCallback could be implemented in the user file
   */
}

/**
  * @brief  Suspend event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_SuspendCallback()
{
  /* Prevent unused argument(s) compilation warning */

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_SuspendCallback could be implemented in the user file
   */
}

/**
  * @brief  Resume event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ResumeCallback()
{
  /* Prevent unused argument(s) compilation warning */

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ResumeCallback could be implemented in the user file
   */
}

/**
  * @brief  Incomplete ISO OUT callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_ISOOUTIncompleteCallback(uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */

  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ISOOUTIncompleteCallback could be implemented in the user file
   */
}

/**
  * @brief  Incomplete ISO IN callback.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval None
  */
__weak void HAL_PCD_ISOINIncompleteCallback(uint8_t epnum)
{
  /* Prevent unused argument(s) compilation warning */

  UNUSED(epnum);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ISOINIncompleteCallback could be implemented in the user file
   */
}

/**
  * @brief  Connection event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_ConnectCallback()
{
  /* Prevent unused argument(s) compilation warning */

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_ConnectCallback could be implemented in the user file
   */
}

/**
  * @brief  Disconnection event callback.
  * @param  hpcd PCD handle
  * @retval None
  */
__weak void HAL_PCD_DisconnectCallback()
{
  /* Prevent unused argument(s) compilation warning */

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCD_DisconnectCallback could be implemented in the user file
   */
}

/**
  * @}
  */

/** @defgroup PCD_Exported_Functions_Group3 Peripheral Control functions
 *  @brief   management functions
 *
@verbatim
 ===============================================================================
                      ##### Peripheral Control functions #####
 ===============================================================================
    [..]
    This subsection provides a set of functions allowing to control the PCD data
    transfers.

@endverbatim
  * @{
  */

/**
  * @brief  Connect the USB device
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DevConnect()
{
#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

  __HAL_LOCK(&hpcd_USB_OTG_FS);
  (void)USB_DevConnect();
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);
  return HAL_OK;
}

/**
  * @brief  Disconnect the USB device.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DevDisconnect()
{
  __HAL_LOCK(&hpcd_USB_OTG_FS);
  (void)USB_DevDisconnect();
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);
  return HAL_OK;
}

/**
  * @brief  Set the USB Device address.
  * @param  hpcd PCD handle
  * @param  address new device address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_SetAddress(uint8_t address)
{
  __HAL_LOCK(&hpcd_USB_OTG_FS);
  hpcd_USB_OTG_FS.USB_Address = address;
  (void)USB_SetDevAddress(address);
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);
  return HAL_OK;
}
/**
  * @brief  Open and configure an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  ep_mps endpoint max packet size
  * @param  ep_type endpoint type
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Open(uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type)
{
  HAL_StatusTypeDef ret = HAL_OK;
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd_USB_OTG_FS.IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd_USB_OTG_FS.OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->num = ep_addr & EP_ADDR_MSK;
  ep->maxpacket = ep_mps;
  ep->type = ep_type;

  if (ep->is_in != 0U)
  {
    /* Assign a Tx FIFO */
    ep->tx_fifo_num = ep->num;
  }
  /* Set initial data PID. */
  if (ep_type == EP_TYPE_BULK)
  {
    ep->data_pid_start = 0U;
  }

  __HAL_LOCK(&hpcd_USB_OTG_FS);
  (void)USB_ActivateEndpoint(ep);
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return ret;
}

/**
  * @brief  Deactivate an endpoint.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Close(uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if ((ep_addr & 0x80U) == 0x80U)
  {
    ep = &hpcd_USB_OTG_FS.IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd_USB_OTG_FS.OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(&hpcd_USB_OTG_FS);
  (void)USB_DeactivateEndpoint(ep);
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);
  return HAL_OK;
}

/**
  * @brief  Receive an amount of data.
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the reception buffer
  * @param  len amount of data to be received
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Receive(uint8_t ep_addr, uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd_USB_OTG_FS.OUT_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0StartXfer(ep);
  }
  else
  {
    (void)USB_EPStartXfer(ep);
  }

  return HAL_OK;
}

/**
  * @brief  Get Received Data Size
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval Data Size
  */
uint32_t HAL_PCD_EP_GetRxCount(uint8_t ep_addr)
{
  return hpcd_USB_OTG_FS.OUT_ep[ep_addr & EP_ADDR_MSK].xfer_count;
}
/**
  * @brief  Send an amount of data
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @param  pBuf pointer to the transmission buffer
  * @param  len amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Transmit(uint8_t ep_addr, const uint8_t *pBuf, uint32_t len)
{
  PCD_EPTypeDef *ep;

  ep = &hpcd_USB_OTG_FS.IN_ep[ep_addr & EP_ADDR_MSK];

  /*setup and start the Xfer */
  ep->xfer_buff = (uint8_t *)pBuf;
  ep->xfer_len = len;
  ep->xfer_count = 0U;
  ep->is_in = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0StartXfer(ep);
  }
  else
  {
    return USB_EPStartXfer(ep);
  }

  return HAL_OK;
}

/**
  * @brief  Set a STALL condition over an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_SetStall(uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd_USB_OTG_FS.IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd_USB_OTG_FS.OUT_ep[ep_addr];
    ep->is_in = 0U;
  }

  ep->is_stall = 1U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(&hpcd_USB_OTG_FS);

  (void)USB_EPSetStall(ep);
  if ((ep_addr & EP_ADDR_MSK) == 0U)
  {
    (void)USB_EP0_OutStart((uint8_t *)hpcd_USB_OTG_FS.Setup);
  }
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return HAL_OK;
}

/**
  * @brief  Clear a STALL condition over in an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(uint8_t ep_addr)
{
  PCD_EPTypeDef *ep;

  if ((0x80U & ep_addr) == 0x80U)
  {
    ep = &hpcd_USB_OTG_FS.IN_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 1U;
  }
  else
  {
    ep = &hpcd_USB_OTG_FS.OUT_ep[ep_addr & EP_ADDR_MSK];
    ep->is_in = 0U;
  }

  ep->is_stall = 0U;
  ep->num = ep_addr & EP_ADDR_MSK;

  __HAL_LOCK(&hpcd_USB_OTG_FS);
  (void)USB_EPClearStall(ep);
  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return HAL_OK;
}

/**
  * @brief  Flush an endpoint
  * @param  hpcd PCD handle
  * @param  ep_addr endpoint address
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_EP_Flush(uint8_t ep_addr)
{
  __HAL_LOCK(&hpcd_USB_OTG_FS);

  if ((ep_addr & 0x80U) == 0x80U)
  {
    (void)USB_FlushTxFifo((uint32_t)ep_addr & EP_ADDR_MSK);
  }
  else
  {
    (void)USB_FlushRxFifo();
  }

  __HAL_UNLOCK(&hpcd_USB_OTG_FS);

  return HAL_OK;
}

/**
  * @brief  Activate remote wakeup signalling
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup()
{
  return (USB_ActivateRemoteWakeup());
}

/**
  * @brief  De-activate remote wakeup signalling.
  * @param  hpcd PCD handle
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup()
{
  return (USB_DeActivateRemoteWakeup());
}

/**
  * @brief  Return the PCD handle state.
  * @param  hpcd PCD handle
  * @retval HAL state
  */
PCD_StateTypeDef HAL_PCD_GetState()
{
  return hpcd_USB_OTG_FS.State;
}

/* Private functions ---------------------------------------------------------*/
/** @addtogroup PCD_Private_Functions
  * @{
  */
#if defined(USB_OTG_FS) || defined(USB_OTG_HS)
/**
  * @brief  Check FIFO for the next packet to be loaded.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_WriteEmptyTxFifo(uint32_t epnum)
{
  USB_OTG_EPTypeDef *ep;
  uint32_t len;
  uint32_t len32b;
  uint32_t fifoemptymsk;

  ep = &hpcd_USB_OTG_FS.IN_ep[epnum];

  len = ep->xfer_len - ep->xfer_count;

  if (len > ep->maxpacket)
  {
    len = ep->maxpacket;
  }

  len32b = (len + 3U) / 4U;

  while (((USBx_INEP(epnum)->DTXFSTS & USB_OTG_DTXFSTS_INEPTFSAV) >= len32b) &&
         (ep->xfer_count < ep->xfer_len) && (ep->xfer_len != 0U))
  {
    /* Write the FIFO */
    len = ep->xfer_len - ep->xfer_count;

    if (len > ep->maxpacket)
    {
      len = ep->maxpacket;
    }
    len32b = (len + 3U) / 4U;

    (void)USB_WritePacket(ep->xfer_buff, (uint8_t)epnum, (uint16_t)len);

    ep->xfer_buff += len;
    ep->xfer_count += len;
  }

  if (ep->xfer_len <= ep->xfer_count)
  {
    fifoemptymsk = (uint32_t)(0x1UL << (epnum & EP_ADDR_MSK));
    USBx_DEVICE->DIEPEMPMSK &= ~fifoemptymsk;
  }

  return HAL_OK;
}

/**
  * @brief  process EP OUT transfer complete interrupt.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP_OutXfrComplete_int(uint32_t epnum)
{
  uint32_t gSNPSiD = *(__IO uint32_t *)(&USB_OTG_FS->CID + 0x1U);
  uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;

  if (gSNPSiD == USB_OTG_CORE_ID_310A)
  {
    /* StupPktRcvd = 1 this is a setup packet */
    if ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX)
    {
      CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
    }
    else
    {
      if ((DoepintReg & USB_OTG_DOEPINT_OTEPSPR) == USB_OTG_DOEPINT_OTEPSPR)
      {
        CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_OTEPSPR);
      }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
      hpcd_USB_OTG_FS.DataOutStageCallback(hpcd, (uint8_t)epnum);
#else
      HAL_PCD_DataOutStageCallback((uint8_t)epnum);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
    }
  }
  else
  {
    if ((epnum == 0U) && (hpcd_USB_OTG_FS.OUT_ep[epnum].xfer_len == 0U))
    {
      /* this is ZLP, so prepare EP0 for next setup */
      (void)USB_EP0_OutStart((uint8_t *)hpcd_USB_OTG_FS.Setup);
    }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
    hpcd_USB_OTG_FS.DataOutStageCallback(hpcd, (uint8_t)epnum);
#else
    HAL_PCD_DataOutStageCallback((uint8_t)epnum);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
  }

  return HAL_OK;
}

/**
  * @brief  process EP OUT setup packet received interrupt.
  * @param  hpcd PCD handle
  * @param  epnum endpoint number
  * @retval HAL status
  */
static HAL_StatusTypeDef PCD_EP_OutSetupPacket_int(uint32_t epnum)
{
  uint32_t gSNPSiD = *(__IO uint32_t *)(&USB_OTG_FS->CID + 0x1U);
  uint32_t DoepintReg = USBx_OUTEP(epnum)->DOEPINT;

  if ((gSNPSiD > USB_OTG_CORE_ID_300A) &&
      ((DoepintReg & USB_OTG_DOEPINT_STPKTRX) == USB_OTG_DOEPINT_STPKTRX))
  {
    CLEAR_OUT_EP_INTR(epnum, USB_OTG_DOEPINT_STPKTRX);
  }

  /* Inform the upper layer that a setup packet is available */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  hpcd_USB_OTG_FS.SetupStageCallback(hpcd);
#else
  HAL_PCD_SetupStageCallback();
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */

  // if ((gSNPSiD > USB_OTG_CORE_ID_300A) && (hpcd_USB_OTG_FS.Init.dma_enable == 1U))
  // {
  //   (void)USB_EP0_OutStart( 1U, (uint8_t *)hpcd_USB_OTG_FS.Setup);
  // }

  return HAL_OK;
}

// ***************
// ***************
// HAL _ PCD _ EXT
// ***************
// ***************

/**
  * @brief  Set Tx FIFO
  * @param  hpcd PCD handle
  * @param  fifo The number of Tx fifo
  * @param  size Fifo size
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(uint8_t fifo, uint16_t size)
{
  uint8_t i;
  uint32_t Tx_Offset;

  /*  TXn min size = 16 words. (n  : Transmit FIFO index)
      When a TxFIFO is not used, the Configuration should be as follows:
          case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txm can use the space allocated for Txn.
         case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txn should be configured with the minimum space of 16 words
     The FIFO is used optimally when used TxFIFOs are allocated in the top
         of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
     When DMA is used 3n * FIFO locations should be reserved for internal DMA registers */

  Tx_Offset = USB_OTG_FS->GRXFSIZ;

  if (fifo == 0U)
  {
    USB_OTG_FS->DIEPTXF0_HNPTXFSIZ = ((uint32_t)size << 16) | Tx_Offset;
  }
  else
  {
    Tx_Offset += (USB_OTG_FS->DIEPTXF0_HNPTXFSIZ) >> 16;
    for (i = 0U; i < (fifo - 1U); i++)
    {
      Tx_Offset += (USB_OTG_FS->DIEPTXF[i] >> 16);
    }

    /* Multiply Tx_Size by 2 to get higher performance */
    USB_OTG_FS->DIEPTXF[fifo - 1U] = ((uint32_t)size << 16) | Tx_Offset;
  }

  return HAL_OK;
}

/**
  * @brief  Set Rx FIFO
  * @param  hpcd PCD handle
  * @param  size Size of Rx fifo
  * @retval HAL status
  */
HAL_StatusTypeDef HAL_PCDEx_SetRxFiFo(uint16_t size)
{
  USB_OTG_FS->GRXFSIZ = size;

  return HAL_OK;
}

/**
  * @brief  Send LPM message to user layer callback.
  * @param  hpcd PCD handle
  * @param  msg LPM message
  * @retval HAL status
  */
__weak void HAL_PCDEx_LPM_Callback(PCD_LPM_MsgTypeDef msg)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(msg);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCDEx_LPM_Callback could be implemented in the user file
   */
}

/**
  * @brief  Send BatteryCharging message to user layer callback.
  * @param  hpcd PCD handle
  * @param  msg LPM message
  * @retval HAL status
  */
__weak void HAL_PCDEx_BCD_Callback(PCD_BCD_MsgTypeDef msg)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(msg);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_PCDEx_BCD_Callback could be implemented in the user file
   */
}

#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */

/**
  * @}
  */
#endif /* defined (USB_OTG_FS) || defined (USB_OTG_HS) */
#endif /* HAL_PCD_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
