/**
  ******************************************************************************
  * @file    usbd_core.c
  * @author  MCD Application Team
  * @brief   This file provides all the USBD core functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_core.h"


extern uint8_t USBD_CDC_Init();
extern uint8_t USBD_CDC_DeInit();
extern uint8_t USBD_CDC_Setup(USBD_SetupReqTypedef *req);
extern uint8_t USBD_CDC_DataIn();
extern void USBD_CDC_DataOut();
extern uint8_t USBD_CDC_EP0_RxReady();
extern uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length);

/**
* @brief  USBD_Init
*         Initializes the device stack and load the class driver
* @param  pdev: device instance
* @param  pdesc: Descriptor structure address
* @param  id: Low level core index
* @retval None
*/
USBD_StatusTypeDef USBD_Init(USBD_DescriptorsTypeDef *pdesc)
{
  USBD_StatusTypeDef ret;

  /* Assign USBD Descriptors */
  if (pdesc != NULL)
  {
    hUsbDeviceFS.pDesc = pdesc;
  }

  /* Set Device initial State */
  hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;

  /* Initialize low level driver */
  ret = USBD_LL_Init();

  return ret;
}

/**
* @brief  USBD_DeInit
*         Re-Initialize th device library
* @param  pdev: device instance
* @retval status: status
*/
USBD_StatusTypeDef USBD_DeInit()
{
  USBD_StatusTypeDef ret;

  /* Set Default State */
  hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;

  USBD_CDC_DeInit();
 
  /* Stop the low level driver  */
  ret = USBD_LL_Stop();

  if (ret != USBD_OK)
  {
    return ret;
  }

  /* Initialize low level driver */
  ret = USBD_LL_DeInit();

  return ret;
}


/**
  * @brief  USBD_Stop
  *         Stop the USB Device Core.
  * @param  pdev: Device Handle
  * @retval USBD Status
  */
USBD_StatusTypeDef USBD_Stop()
{
  USBD_StatusTypeDef ret;


  USBD_CDC_DeInit();


  /* Stop the low level driver */
  ret = USBD_LL_Stop();

  return ret;
}



/**
* @brief  USBD_SetClassConfig
*        Configure device and start the interface
* @param  pdev: device instance
* @param  cfgidx: configuration index
* @retval status
*/

USBD_StatusTypeDef USBD_SetClassConfig(uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_StatusTypeDef ret = USBD_FAIL;

  ret = (USBD_StatusTypeDef)USBD_CDC_Init();

  return ret;
}

/**
* @brief  USBD_ClrClassConfig
*         Clear current configuration
* @param  pdev: device instance
* @param  cfgidx: configuration index
* @retval status: USBD_StatusTypeDef
*/
USBD_StatusTypeDef USBD_ClrClassConfig(uint8_t cfgidx)
{
  (void)cfgidx;
  USBD_CDC_DeInit();

  return USBD_OK;
}


/**
* @brief  USBD_SetupStage
*         Handle the setup stage
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_SetupStage(uint8_t *psetup)
{
  USBD_StatusTypeDef ret;

  USBD_ParseSetupRequest(&hUsbDeviceFS.request, psetup);

  hUsbDeviceFS.ep0_state = USBD_EP0_SETUP;

  hUsbDeviceFS.ep0_data_len = hUsbDeviceFS.request.wLength;

  switch (hUsbDeviceFS.request.bmRequest & 0x1FU)
  {
    case USB_REQ_RECIPIENT_DEVICE:
      ret = USBD_StdDevReq(&hUsbDeviceFS.request);
      break;

    case USB_REQ_RECIPIENT_INTERFACE:
      ret = USBD_StdItfReq(&hUsbDeviceFS.request);
      break;

    case USB_REQ_RECIPIENT_ENDPOINT:
      ret = USBD_StdEPReq(&hUsbDeviceFS.request);
      break;

    default:
      ret = HAL_PCD_EP_SetStall((hUsbDeviceFS.request.bmRequest & 0x80U));
      break;
  }

  return ret;
}

/**
* @brief  USBD_DataOutStage
*         Handle data OUT stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
USBD_StatusTypeDef USBD_LL_DataOutStage(uint8_t epnum, uint8_t *pdata)
{
  USBD_EndpointTypeDef *pep;
  //USBD_StatusTypeDef ret;

  if (epnum == 0U)
  {
    pep = &hUsbDeviceFS.ep_out[0];

    if (hUsbDeviceFS.ep0_state == USBD_EP0_DATA_OUT)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        USBD_CtlContinueRx(pdata, MIN(pep->rem_length, pep->maxpacket));
      }
      else
      {
        if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
        {
          USBD_CDC_EP0_RxReady();
        }
        USBD_CtlSendStatus();
      }
    }
    else
    {
#if 0
      if (hUsbDeviceFS.ep0_state == USBD_EP0_STATUS_OUT)
      {
        /*
         * STATUS PHASE completed, update ep0_state to idle
         */
        hUsbDeviceFS.ep0_state = USBD_EP0_IDLE;
        USBD_LL_StallEP(0U);
      }
#endif
    }
  }
  else if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
  {
    USBD_CDC_DataOut();
  }
  else
  {
    /* should never be in this condition */
    return USBD_FAIL;
  }

  return USBD_OK;
}

/**
* @brief  USBD_DataInStage
*         Handle data in stage
* @param  pdev: device instance
* @param  epnum: endpoint index
* @retval status
*/
USBD_StatusTypeDef USBD_LL_DataInStage(uint8_t epnum, uint8_t *pdata)
{
  USBD_EndpointTypeDef *pep;

  if (epnum == 0U)
  {
    pep = &hUsbDeviceFS.ep_in[0];

    if (hUsbDeviceFS.ep0_state == USBD_EP0_DATA_IN)
    {
      if (pep->rem_length > pep->maxpacket)
      {
        pep->rem_length -= pep->maxpacket;

        USBD_CtlContinueSendData(pdata, pep->rem_length);

        /* Prepare endpoint for premature end of transfer */
       USBD_LL_PrepareReceive(0U, NULL, 0U);
      }
      else
      {
        /* last packet is MPS multiple, so send ZLP packet */
        if ((pep->maxpacket == pep->rem_length) &&
            (pep->total_length >= pep->maxpacket) &&
            (pep->total_length < hUsbDeviceFS.ep0_data_len))
        {
          USBD_CtlContinueSendData(NULL, 0U);
          hUsbDeviceFS.ep0_data_len = 0U;

          /* Prepare endpoint for premature end of transfer */
          USBD_LL_PrepareReceive(0U, NULL, 0U);
        }
        else
        {
          // if ((hUsbDeviceFS.pClass->EP0_TxSent != NULL) &&
          //     (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED))
          // {
          //   hUsbDeviceFS.pClass->EP0_TxSent();
          // }
          HAL_PCD_EP_SetStall(0x80U);
          USBD_CtlReceiveStatus();
        }
      }
    }
    else
    {
#if 0
      if ((hUsbDeviceFS.ep0_state == USBD_EP0_STATUS_IN) ||
          (hUsbDeviceFS.ep0_state == USBD_EP0_IDLE))
      {
        USBD_LL_StallEP(0x80U);
      }
#endif
    }

  }
  else if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
  {
    USBD_CDC_DataIn();
  }
  else
  {
    /* should never be in this condition */
    return USBD_FAIL;
  }

  return USBD_OK;
}

/**
* @brief  USBD_LL_Reset
*         Handle Reset event
* @param  pdev: device instance
* @retval status
*/

USBD_StatusTypeDef USBD_LL_Reset()
{
  /* Upon Reset call user call back */
  hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;
  hUsbDeviceFS.ep0_state = USBD_EP0_IDLE;
  hUsbDeviceFS.dev_config = 0U;
  hUsbDeviceFS.dev_remote_wakeup = 0U;

  USBD_CDC_DeInit();

    /* Open EP0 OUT */
  USBD_LL_OpenEP(0x00U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  hUsbDeviceFS.ep_out[0x00U & 0xFU].is_used = 1U;

  hUsbDeviceFS.ep_out[0].maxpacket = USB_MAX_EP0_SIZE;

  /* Open EP0 IN */
  USBD_LL_OpenEP(0x80U, USBD_EP_TYPE_CTRL, USB_MAX_EP0_SIZE);
  hUsbDeviceFS.ep_in[0x80U & 0xFU].is_used = 1U;

  hUsbDeviceFS.ep_in[0].maxpacket = USB_MAX_EP0_SIZE;

  return USBD_OK;
}


/**
* @brief  USBD_Suspend
*         Handle Suspend event
* @param  pdev: device instance
* @retval status
*/

USBD_StatusTypeDef USBD_LL_Suspend()
{
  hUsbDeviceFS.dev_old_state = hUsbDeviceFS.dev_state;
  hUsbDeviceFS.dev_state = USBD_STATE_SUSPENDED;

  return USBD_OK;
}

/**
* @brief  USBD_Resume
*         Handle Resume event
* @param  pdev: device instance
* @retval status
*/

USBD_StatusTypeDef USBD_LL_Resume()
{
  if (hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED)
  {
    hUsbDeviceFS.dev_state = hUsbDeviceFS.dev_old_state;
  }

  return USBD_OK;
}

/**
* @brief  USBD_SOF
*         Handle SOF event
* @param  pdev: device instance
* @retval status
*/

USBD_StatusTypeDef USBD_LL_SOF()
{
  // if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
  // {
  //   if (hUsbDeviceFS.pClass->SOF != NULL)
  //   {
  //     hUsbDeviceFS.pClass->SOF();
  //   }
  // }

  return USBD_OK;
}

/**
* @brief  USBD_IsoINIncomplete
*         Handle iso in incomplete event
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_IsoINIncomplete(uint8_t epnum)
{
  /* Prevent unused arguments compilation warning */
  UNUSED(epnum);

  return USBD_OK;
}

/**
* @brief  USBD_IsoOUTIncomplete
*         Handle iso out incomplete event
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_IsoOUTIncomplete(uint8_t epnum)
{
  /* Prevent unused arguments compilation warning */
  UNUSED(epnum);

  return USBD_OK;
}

/**
* @brief  USBD_DevConnected
*         Handle device connection event
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_DevConnected()
{
  /* Prevent unused argument compilation warning */

  return USBD_OK;
}

/**
* @brief  USBD_DevDisconnected
*         Handle device disconnection event
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_LL_DevDisconnected()
{
  /* Free Class Resources */
  hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;

  USBD_CDC_DeInit();

  return USBD_OK;
}
/**
* @}
*/


/**
* @}
*/


/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

