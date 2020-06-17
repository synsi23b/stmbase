/**
  ******************************************************************************
  * @file    usbd_req.c
  * @author  MCD Application Team
  * @brief   This file provides the standard USB requests following chapter 9.
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
#include "usbd_ctlreq.h"
#include "usbd_core.h"

/** @addtogroup STM32_USBD_STATE_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_REQ
  * @brief USB standard requests module
  * @{
  */

/** @defgroup USBD_REQ_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Defines
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_Variables
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_REQ_Private_FunctionPrototypes
  * @{
  */
static void USBD_GetDescriptor(USBD_SetupReqTypedef *req);
static void USBD_SetAddress(USBD_SetupReqTypedef *req);
static USBD_StatusTypeDef USBD_SetConfig(USBD_SetupReqTypedef *req);
static void USBD_GetConfig(USBD_SetupReqTypedef *req);
static void USBD_GetStatus(USBD_SetupReqTypedef *req);
static void USBD_SetFeature(USBD_SetupReqTypedef *req);
static void USBD_ClrFeature(USBD_SetupReqTypedef *req);
static uint8_t USBD_GetLen(uint8_t *buf);

/**
  * @}
  */
extern uint8_t USBD_CDC_Init(uint8_t cfgidx);
extern uint8_t USBD_CDC_DeInit(uint8_t cfgidx);
extern uint8_t USBD_CDC_Setup(USBD_SetupReqTypedef *req);
extern uint8_t USBD_CDC_DataIn(uint8_t epnum);
extern uint8_t USBD_CDC_DataOut(uint8_t epnum);
extern uint8_t USBD_CDC_EP0_RxReady();
extern uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length);


/** @defgroup USBD_REQ_Private_Functions
  * @{
  */


/**
* @brief  USBD_StdDevReq
*         Handle standard usb device requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
USBD_StatusTypeDef USBD_StdDevReq(USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS:
  case USB_REQ_TYPE_VENDOR:
    ret = (USBD_StatusTypeDef)USBD_CDC_Setup(req);
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR:
      USBD_GetDescriptor(req);
      break;

    case USB_REQ_SET_ADDRESS:
      USBD_SetAddress(req);
      break;

    case USB_REQ_SET_CONFIGURATION:
      ret = USBD_SetConfig(req);
      break;

    case USB_REQ_GET_CONFIGURATION:
      USBD_GetConfig(req);
      break;

    case USB_REQ_GET_STATUS:
      USBD_GetStatus(req);
      break;

    case USB_REQ_SET_FEATURE:
      USBD_SetFeature(req);
      break;

    case USB_REQ_CLEAR_FEATURE:
      USBD_ClrFeature(req);
      break;

    default:
      USBD_CtlError(req);
      break;
    }
    break;

  default:
    USBD_CtlError(req);
    break;
  }

  return ret;
}

/**
* @brief  USBD_StdItfReq
*         Handle standard usb interface requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
USBD_StatusTypeDef USBD_StdItfReq(USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS:
  case USB_REQ_TYPE_VENDOR:
  case USB_REQ_TYPE_STANDARD:
    switch (hUsbDeviceFS.dev_state)
    {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:

      if (LOBYTE(req->wIndex) <= USBD_MAX_NUM_INTERFACES)
      {
        ret = (USBD_StatusTypeDef)USBD_CDC_Setup(req);

        if ((req->wLength == 0U) && (ret == USBD_OK))
        {
          USBD_CtlSendStatus();
        }
      }
      else
      {
        USBD_CtlError(req);
      }
      break;

    default:
      USBD_CtlError(req);
      break;
    }
    break;

  default:
    USBD_CtlError(req);
    break;
  }

  return ret;
}

/**
* @brief  USBD_StdEPReq
*         Handle standard usb endpoint requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
USBD_StatusTypeDef USBD_StdEPReq(USBD_SetupReqTypedef *req)
{
  USBD_EndpointTypeDef *pep;
  uint8_t ep_addr;
  USBD_StatusTypeDef ret = USBD_OK;
  ep_addr = LOBYTE(req->wIndex);

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_CLASS:
  case USB_REQ_TYPE_VENDOR:
    ret = (USBD_StatusTypeDef)USBD_CDC_Setup(req);
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_SET_FEATURE:
      switch (hUsbDeviceFS.dev_state)
      {
      case USBD_STATE_ADDRESSED:
        if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
        {
          HAL_PCD_EP_SetStall(ep_addr);
          HAL_PCD_EP_SetStall(0x80U);
        }
        else
        {
          USBD_CtlError(req);
        }
        break;

      case USBD_STATE_CONFIGURED:
        if (req->wValue == USB_FEATURE_EP_HALT)
        {
          if ((ep_addr != 0x00U) && (ep_addr != 0x80U) && (req->wLength == 0x00U))
          {
            HAL_PCD_EP_SetStall(ep_addr);
          }
        }
        USBD_CtlSendStatus();

        break;

      default:
        USBD_CtlError(req);
        break;
      }
      break;

    case USB_REQ_CLEAR_FEATURE:

      switch (hUsbDeviceFS.dev_state)
      {
      case USBD_STATE_ADDRESSED:
        if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
        {
          HAL_PCD_EP_SetStall(ep_addr);
          HAL_PCD_EP_SetStall(0x80U);
        }
        else
        {
          USBD_CtlError(req);
        }
        break;

      case USBD_STATE_CONFIGURED:
        if (req->wValue == USB_FEATURE_EP_HALT)
        {
          if ((ep_addr & 0x7FU) != 0x00U)
          {
            HAL_PCD_EP_ClrStall(ep_addr);
          }
          USBD_CtlSendStatus();
          (USBD_StatusTypeDef)USBD_CDC_Setup(req);
        }
        break;

      default:
        USBD_CtlError(req);
        break;
      }
      break;

    case USB_REQ_GET_STATUS:
      switch (hUsbDeviceFS.dev_state)
      {
      case USBD_STATE_ADDRESSED:
        if ((ep_addr != 0x00U) && (ep_addr != 0x80U))
        {
          USBD_CtlError(req);
          break;
        }
        pep = ((ep_addr & 0x80U) == 0x80U) ? &hUsbDeviceFS.ep_in[ep_addr & 0x7FU] : \
              &hUsbDeviceFS.ep_out[ep_addr & 0x7FU];

        pep->status = 0x0000U;

        USBD_CtlSendData((uint8_t *)&pep->status, 2U);
        break;

      case USBD_STATE_CONFIGURED:
        if ((ep_addr & 0x80U) == 0x80U)
        {
          if (hUsbDeviceFS.ep_in[ep_addr & 0xFU].is_used == 0U)
          {
            USBD_CtlError(req);
            break;
          }
        }
        else
        {
          if (hUsbDeviceFS.ep_out[ep_addr & 0xFU].is_used == 0U)
          {
            USBD_CtlError(req);
            break;
          }
        }

        pep = ((ep_addr & 0x80U) == 0x80U) ? &hUsbDeviceFS.ep_in[ep_addr & 0x7FU] : \
              &hUsbDeviceFS.ep_out[ep_addr & 0x7FU];

          if ((ep_addr == 0x00U) || (ep_addr == 0x80U))
          {
            pep->status = 0x0000U;
          }
          else if (USBD_LL_IsStallEP(ep_addr) != 0U)
          {
            pep->status = 0x0001U;
          }
          else
          {
            pep->status = 0x0000U;
          }

          USBD_CtlSendData((uint8_t *)&pep->status, 2U);
          break;

      default:
        USBD_CtlError(req);
        break;
      }
      break;

    default:
      USBD_CtlError(req);
      break;
    }
    break;

  default:
    USBD_CtlError(req);
    break;
  }

  return ret;
}


/**
* @brief  USBD_GetDescriptor
*         Handle Get Descriptor requests
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetDescriptor(USBD_SetupReqTypedef *req)
{
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint8_t err = 0U;

  switch (req->wValue >> 8)
  {
#if ((USBD_LPM_ENABLED == 1U) || (USBD_CLASS_BOS_ENABLED == 1U))
  case USB_DESC_TYPE_BOS:
    if (hUsbDeviceFS.pDesc->GetBOSDescriptor != NULL)
    {
      pbuf = hUsbDeviceFS.pDesc->GetBOSDescriptor(&len);
    }
    else
    {
      USBD_CtlError(req);
      err++;
    }
    break;
#endif
  case USB_DESC_TYPE_DEVICE:
    pbuf = hUsbDeviceFS.pDesc->GetDeviceDescriptor(&len);
    break;

  case USB_DESC_TYPE_CONFIGURATION:
    // if (hUsbDeviceFS.dev_speed == USBD_SPEED_HIGH)
    // {
    //   pbuf = hUsbDeviceFS.pClass->GetHSConfigDescriptor(&len);
    //   pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
    // }
    // else
    {
      pbuf = USBD_CDC_GetFSCfgDesc(&len);
      pbuf[1] = USB_DESC_TYPE_CONFIGURATION;
    }
    break;

  case USB_DESC_TYPE_STRING:
    switch ((uint8_t)(req->wValue))
    {
    case USBD_IDX_LANGID_STR:
      if (hUsbDeviceFS.pDesc->GetLangIDStrDescriptor != NULL)
      {
        pbuf = hUsbDeviceFS.pDesc->GetLangIDStrDescriptor(&len);
      }
      else
      {
        USBD_CtlError(req);
        err++;
      }
      break;

    case USBD_IDX_MFC_STR:
      if (hUsbDeviceFS.pDesc->GetManufacturerStrDescriptor != NULL)
      {
        pbuf = hUsbDeviceFS.pDesc->GetManufacturerStrDescriptor(&len);
      }
      else
      {
        USBD_CtlError(req);
        err++;
      }
      break;

    case USBD_IDX_PRODUCT_STR:
      if (hUsbDeviceFS.pDesc->GetProductStrDescriptor != NULL)
      {
        pbuf = hUsbDeviceFS.pDesc->GetProductStrDescriptor(&len);
      }
      else
      {
        USBD_CtlError(req);
        err++;
      }
      break;

    case USBD_IDX_SERIAL_STR:
      if (hUsbDeviceFS.pDesc->GetSerialStrDescriptor != NULL)
      {
        pbuf = hUsbDeviceFS.pDesc->GetSerialStrDescriptor(&len);
      }
      else
      {
        USBD_CtlError(req);
        err++;
      }
      break;

    case USBD_IDX_CONFIG_STR:
      if (hUsbDeviceFS.pDesc->GetConfigurationStrDescriptor != NULL)
      {
        pbuf = hUsbDeviceFS.pDesc->GetConfigurationStrDescriptor(&len);
      }
      else
      {
        USBD_CtlError(req);
        err++;
      }
      break;

    case USBD_IDX_INTERFACE_STR:
      if (hUsbDeviceFS.pDesc->GetInterfaceStrDescriptor != NULL)
      {
        pbuf = hUsbDeviceFS.pDesc->GetInterfaceStrDescriptor(&len);
      }
      else
      {
        USBD_CtlError(req);
        err++;
      }
      break;

    default:
#if (USBD_SUPPORT_USER_STRING_DESC == 1U)
      // if (hUsbDeviceFS.pClass->GetUsrStrDescriptor != NULL)
      // {
      //   pbuf = hUsbDeviceFS.pClass->GetUsrStrDescriptor((req->wValue), &len);
      // }
      // else
      {
        USBD_CtlError(req);
        err++;
      }
#elif (USBD_CLASS_USER_STRING_DESC == 1U)
      // if (hUsbDeviceFS.pDesc->GetUserStrDescriptor != NULL)
      // {
      //   pbuf = hUsbDeviceFS.pDesc->GetUserStrDescriptor((req->wValue), &len);
      // }
      // else
      {
        USBD_CtlError(req);
        err++;
      }
#else
      USBD_CtlError(req);
      err++;
#endif
      break;
    }
    break;

  case USB_DESC_TYPE_DEVICE_QUALIFIER:
    // if (hUsbDeviceFS.dev_speed == USBD_SPEED_HIGH)
    // {
    //   pbuf = USBD_CDC_GetDeviceQualifierDescriptor(&len);
    // }
    // else
    {
      USBD_CtlError(req);
      err++;
    }
    break;

  case USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION:
    // if (hUsbDeviceFS.dev_speed == USBD_SPEED_HIGH)
    // {
    //   pbuf = hUsbDeviceFS.pClass->GetOtherSpeedConfigDescriptor(&len);
    //   pbuf[1] = USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION;
    // }
    // else
    {
      USBD_CtlError(req);
      err++;
    }
    break;

  default:
    USBD_CtlError(req);
    err++;
    break;
  }

  if (err != 0U)
  {
    return;
  }
  else
  {
    if (req->wLength != 0U)
    {
      if (len != 0U)
      {
        len = MIN(len, req->wLength);
        USBD_CtlSendData(pbuf, len);
      }
      else
      {
        USBD_CtlError(req);
      }
    }
    else
    {
      USBD_CtlSendStatus();
    }
  }
}

/**
* @brief  USBD_SetAddress
*         Set device address
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetAddress(USBD_SetupReqTypedef *req)
{
  uint8_t  dev_addr;

  if ((req->wIndex == 0U) && (req->wLength == 0U) && (req->wValue < 128U))
  {
    dev_addr = (uint8_t)(req->wValue) & 0x7FU;

    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
      USBD_CtlError(req);
    }
    else
    {
      hUsbDeviceFS.dev_address = dev_addr;
      USBD_LL_SetUSBAddress(dev_addr);
      USBD_CtlSendStatus();

      if (dev_addr != 0U)
      {
        hUsbDeviceFS.dev_state = USBD_STATE_ADDRESSED;
      }
      else
      {
        hUsbDeviceFS.dev_state = USBD_STATE_DEFAULT;
      }
    }
  }
  else
  {
    USBD_CtlError(req);
  }
}

/**
* @brief  USBD_SetConfig
*         Handle Set device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static USBD_StatusTypeDef USBD_SetConfig(USBD_SetupReqTypedef *req)
{
  USBD_StatusTypeDef ret = USBD_OK;
  static uint8_t cfgidx;

  cfgidx = (uint8_t)(req->wValue);

  if (cfgidx > USBD_MAX_NUM_CONFIGURATION)
  {
    USBD_CtlError(req);
    return USBD_FAIL;
  }

  switch (hUsbDeviceFS.dev_state)
  {
  case USBD_STATE_ADDRESSED:
    if (cfgidx != 0U)
    {
      hUsbDeviceFS.dev_config = cfgidx;

      ret = USBD_SetClassConfig(cfgidx);

      if (ret != USBD_OK)
      {
        USBD_CtlError(req);
      }
      else
      {
        USBD_CtlSendStatus();
        hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
      }
    }
    else
    {
      USBD_CtlSendStatus();
    }
    break;

  case USBD_STATE_CONFIGURED:
    if (cfgidx == 0U)
    {
      hUsbDeviceFS.dev_state = USBD_STATE_ADDRESSED;
      hUsbDeviceFS.dev_config = cfgidx;
      USBD_ClrClassConfig(cfgidx);
      USBD_CtlSendStatus();
    }
    else if (cfgidx != hUsbDeviceFS.dev_config)
    {
      /* Clear old configuration */
      USBD_ClrClassConfig((uint8_t)hUsbDeviceFS.dev_config);

      /* set new configuration */
      hUsbDeviceFS.dev_config = cfgidx;

      ret = USBD_SetClassConfig(cfgidx);

      if (ret != USBD_OK)
      {
        USBD_CtlError(req);
        USBD_ClrClassConfig((uint8_t)hUsbDeviceFS.dev_config);
        hUsbDeviceFS.dev_state = USBD_STATE_ADDRESSED;
      }
      else
      {
        USBD_CtlSendStatus();
      }
    }
    else
    {
      USBD_CtlSendStatus();
    }
    break;

  default:
    USBD_CtlError(req);
    USBD_ClrClassConfig(cfgidx);
    ret = USBD_FAIL;
    break;
  }

  return ret;
}

/**
* @brief  USBD_GetConfig
*         Handle Get device configuration request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetConfig(USBD_SetupReqTypedef *req)
{
  if (req->wLength != 1U)
  {
    USBD_CtlError(req);
  }
  else
  {
    switch (hUsbDeviceFS.dev_state)
    {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
      hUsbDeviceFS.dev_default_config = 0U;
      USBD_CtlSendData((uint8_t *)&hUsbDeviceFS.dev_default_config, 1U);
      break;

    case USBD_STATE_CONFIGURED:
      USBD_CtlSendData((uint8_t *)&hUsbDeviceFS.dev_config, 1U);
      break;

    default:
      USBD_CtlError(req);
      break;
    }
  }
}

/**
* @brief  USBD_GetStatus
*         Handle Get Status request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_GetStatus(USBD_SetupReqTypedef *req)
{
  switch (hUsbDeviceFS.dev_state)
  {
  case USBD_STATE_DEFAULT:
  case USBD_STATE_ADDRESSED:
  case USBD_STATE_CONFIGURED:
    if (req->wLength != 0x2U)
    {
      USBD_CtlError(req);
      break;
    }

#if (USBD_SELF_POWERED == 1U)
    hUsbDeviceFS.dev_config_status = USB_CONFIG_SELF_POWERED;
#else
    hUsbDeviceFS.dev_config_status = 0U;
#endif

    if (hUsbDeviceFS.dev_remote_wakeup != 0U)
    {
      hUsbDeviceFS.dev_config_status |= USB_CONFIG_REMOTE_WAKEUP;
    }

    USBD_CtlSendData((uint8_t *)&hUsbDeviceFS.dev_config_status, 2U);
    break;

  default:
    USBD_CtlError(req);
    break;
  }
}


/**
* @brief  USBD_SetFeature
*         Handle Set device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_SetFeature(USBD_SetupReqTypedef *req)
{
  if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
  {
    hUsbDeviceFS.dev_remote_wakeup = 1U;
    USBD_CtlSendStatus();
  }
}


/**
* @brief  USBD_ClrFeature
*         Handle clear device feature request
* @param  pdev: device instance
* @param  req: usb request
* @retval status
*/
static void USBD_ClrFeature(USBD_SetupReqTypedef *req)
{
  switch (hUsbDeviceFS.dev_state)
  {
    case USBD_STATE_DEFAULT:
    case USBD_STATE_ADDRESSED:
    case USBD_STATE_CONFIGURED:
      if (req->wValue == USB_FEATURE_REMOTE_WAKEUP)
      {
        hUsbDeviceFS.dev_remote_wakeup = 0U;
        USBD_CtlSendStatus();
      }
      break;

    default:
      USBD_CtlError(req);
      break;
  }
}

/**
* @brief  USBD_ParseSetupRequest
*         Copy buffer into setup structure
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/

void USBD_ParseSetupRequest(USBD_SetupReqTypedef *req, uint8_t *pdata)
{
  uint8_t *pbuff = pdata;

  req->bmRequest = *(uint8_t *)(pbuff);

  pbuff++;
  req->bRequest = *(uint8_t *)(pbuff);

  pbuff++;
  req->wValue = SWAPBYTE(pbuff);

  pbuff++;
  pbuff++;
  req->wIndex = SWAPBYTE(pbuff);

  pbuff++;
  pbuff++;
  req->wLength = SWAPBYTE(pbuff);
}

/**
* @brief  USBD_CtlError
*         Handle USB low level Error
* @param  pdev: device instance
* @param  req: usb request
* @retval None
*/

void USBD_CtlError(USBD_SetupReqTypedef *req)
{
  UNUSED(req);

  HAL_PCD_EP_SetStall(0x80U);
  HAL_PCD_EP_SetStall(0U);
}


/**
  * @brief  USBD_GetString
  *         Convert Ascii string into unicode one
  * @param  desc : descriptor buffer
  * @param  unicode : Formatted string buffer (unicode)
  * @param  len : descriptor length
  * @retval None
  */
void USBD_GetString(uint8_t *desc, uint8_t *unicode, uint16_t *len)
{
  uint8_t idx = 0U;
  uint8_t *pdesc;

  if (desc == NULL)
  {
    return;
  }

  pdesc = desc;
  *len = ((uint16_t)USBD_GetLen(pdesc) * 2U) + 2U;

  unicode[idx] = *(uint8_t *)len;
  idx++;
  unicode[idx] = USB_DESC_TYPE_STRING;
  idx++;

  while (*pdesc != (uint8_t)'\0')
  {
    unicode[idx] = *pdesc;
    pdesc++;
    idx++;

    unicode[idx] = 0U;
    idx++;
  }
}

/**
  * @brief  USBD_GetLen
  *         return the string length
   * @param  buf : pointer to the ascii string buffer
  * @retval string length
  */
static uint8_t USBD_GetLen(uint8_t *buf)
{
  uint8_t  len = 0U;
  uint8_t *pbuff = buf;

  while (*pbuff != (uint8_t)'\0')
  {
    len++;
    pbuff++;
  }

  return len;
}

/**
* @brief  USBD_CtlSendData
*         send data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be sent
* @retval status
*/
USBD_StatusTypeDef USBD_CtlSendData(uint8_t *pbuf, uint32_t len)
{
  /* Set EP0 State */
  hUsbDeviceFS.ep0_state = USBD_EP0_DATA_IN;
  hUsbDeviceFS.ep_in[0].total_length = len;
  hUsbDeviceFS.ep_in[0].rem_length = len;

  /* Start the transfer */
  USBD_LL_Transmit(0x00U, pbuf, len);

  return USBD_OK;
}

/**
* @brief  USBD_CtlContinueSendData
*         continue sending data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be sent
* @retval status
*/
USBD_StatusTypeDef USBD_CtlContinueSendData(uint8_t *pbuf, uint32_t len)
{
  /* Start the next transfer */
  USBD_LL_Transmit(0x00U, pbuf, len);

  return USBD_OK;
}

/**
* @brief  USBD_CtlPrepareRx
*         receive data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be received
* @retval status
*/
USBD_StatusTypeDef USBD_CtlPrepareRx(uint8_t *pbuf, uint32_t len)
{
  /* Set EP0 State */
  hUsbDeviceFS.ep0_state = USBD_EP0_DATA_OUT;
  hUsbDeviceFS.ep_out[0].total_length = len;
  hUsbDeviceFS.ep_out[0].rem_length = len;

  /* Start the transfer */
  USBD_LL_PrepareReceive(0U, pbuf, len);

  return USBD_OK;
}

/**
* @brief  USBD_CtlContinueRx
*         continue receive data on the ctl pipe
* @param  pdev: device instance
* @param  buff: pointer to data buffer
* @param  len: length of data to be received
* @retval status
*/
USBD_StatusTypeDef USBD_CtlContinueRx(uint8_t *pbuf, uint32_t len)
{
  USBD_LL_PrepareReceive(0U, pbuf, len);

  return USBD_OK;
}

/**
* @brief  USBD_CtlSendStatus
*         send zero lzngth packet on the ctl pipe
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_CtlSendStatus()
{
  /* Set EP0 State */
  hUsbDeviceFS.ep0_state = USBD_EP0_STATUS_IN;

  /* Start the transfer */
  USBD_LL_Transmit(0x00U, NULL, 0U);

  return USBD_OK;
}

/**
* @brief  USBD_CtlReceiveStatus
*         receive zero lzngth packet on the ctl pipe
* @param  pdev: device instance
* @retval status
*/
USBD_StatusTypeDef USBD_CtlReceiveStatus()
{
  /* Set EP0 State */
  hUsbDeviceFS.ep0_state = USBD_EP0_STATUS_OUT;

  /* Start the transfer */
  USBD_LL_PrepareReceive(0U, NULL, 0U);

  return USBD_OK;
}

/**
* @brief  USBD_GetRxCount
*         returns the received data length
* @param  pdev: device instance
* @param  ep_addr: endpoint address
* @retval Rx Data blength
*/
uint32_t USBD_GetRxCount(uint8_t ep_addr)
{
  return USBD_LL_GetRxDataSize(ep_addr);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
