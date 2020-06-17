/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_conf.c
  * @version        : v1.0_Cube
  * @brief          : This file implements the board support package for the USB device library
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

#include "usbd_def.h"
#include "usbd_core.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

PCD_HandleTypeDef hpcd_USB_OTG_FS;
void Error_Handler(void);

/* External functions --------------------------------------------------------*/
//void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
//USBD_StatusTypeDef USBD_Get_USB_Status(HAL_StatusTypeDef hal_status);

/* USER CODE END PFP */

/* Private functions ---------------------------------------------------------*/

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/*******************************************************************************
                       LL Driver Callbacks (PCD -> USB Device Library)
*******************************************************************************/
/* MSP Init */

/**
  * @brief  Initializes the PCD according to the specified
  *         parameters in the PCD_InitTypeDef and initialize the associated handle.
  * @param  hpcd PCD handle
  * @retval HAL status
  */

HAL_StatusTypeDef HAL_PCD_Init()
{
  //USB_OTG_GlobalTypeDef *USBx;
  uint16_t i;

  /* Check the parameters */
  //assert_param(IS_PCD_ALL_INSTANCE(hpcd->Instance));

  //USBx = hpcd->Instance;
  hpcd_USB_OTG_FS.State = HAL_PCD_STATE_BUSY;
  hpcd_USB_OTG_FS.Init.dma_enable = 0U;
  

  /* Disable the Interrupts */
  __HAL_PCD_DISABLE();

  /*Init the Core (common init.) */
  if (USB_CoreInit(hpcd_USB_OTG_FS.Init) != HAL_OK)
  {
    hpcd_USB_OTG_FS.State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  /* Force Device Mode*/
  (void)USB_SetCurrentMode(USB_DEVICE_MODE);

  /* Init endpoints structures */
  for (i = 0U; i < 4; i++)
  {
    /* Init ep structure */
    hpcd_USB_OTG_FS.IN_ep[i].is_in = 1U;
    hpcd_USB_OTG_FS.IN_ep[i].num = i;
    hpcd_USB_OTG_FS.IN_ep[i].tx_fifo_num = i;
    /* Control until ep is activated */
    hpcd_USB_OTG_FS.IN_ep[i].type = EP_TYPE_CTRL;
    hpcd_USB_OTG_FS.IN_ep[i].maxpacket = 0U;
    hpcd_USB_OTG_FS.IN_ep[i].xfer_buff = 0U;
    hpcd_USB_OTG_FS.IN_ep[i].xfer_len = 0U;
  }

  for (i = 0U; i < 4; i++)
  {
    hpcd_USB_OTG_FS.OUT_ep[i].is_in = 0U;
    hpcd_USB_OTG_FS.OUT_ep[i].num = i;
    /* Control until ep is activated */
    hpcd_USB_OTG_FS.OUT_ep[i].type = EP_TYPE_CTRL;
    hpcd_USB_OTG_FS.OUT_ep[i].maxpacket = 0U;
    hpcd_USB_OTG_FS.OUT_ep[i].xfer_buff = 0U;
    hpcd_USB_OTG_FS.OUT_ep[i].xfer_len = 0U;
  }

  /* Init Device */
  if (USB_DevInit(hpcd_USB_OTG_FS.Init) != HAL_OK)
  {
    hpcd_USB_OTG_FS.State = HAL_PCD_STATE_ERROR;
    return HAL_ERROR;
  }

  hpcd_USB_OTG_FS.USB_Address = 0U;
  hpcd_USB_OTG_FS.State = HAL_PCD_STATE_READY;
  #if defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx)
  /* Activate LPM */
  if (hpcd->Init.lpm_enable == 1U)
  {
    (void)HAL_PCDEx_ActivateLPM(hpcd);
  }
  #endif /* defined(STM32F446xx) || defined(STM32F469xx) || defined(STM32F479xx) || defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx) || defined(STM32F413xx) || defined(STM32F423xx) */
  (void)USB_DevDisconnect(hpcd_USB_OTG_FS.Instance);

  return HAL_OK;
}

/**
  * @brief  Setup stage callback
  * @param  hpcd: PCD handle
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_SetupStageCallback()
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  USBD_LL_SetupStage((uint8_t *)hpcd_USB_OTG_FS.Setup);
}

/**
  * @brief  Data Out stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint number
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void HAL_PCD_DataOutStageCallback(uint8_t epnum)
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  USBD_LL_DataOutStage(epnum, hpcd_USB_OTG_FS.OUT_ep[epnum].xfer_buff);
}

/**
  * @brief  Data In stage callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint number
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void HAL_PCD_DataInStageCallback(uint8_t epnum)
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  USBD_LL_DataInStage(epnum, hpcd_USB_OTG_FS.IN_ep[epnum].xfer_buff);
}

/**
  * @brief  SOF callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SOFCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_SOFCallback()
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  //USBD_LL_SOF();
}

/**
  * @brief  Reset callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ResetCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_ResetCallback()
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{ 
    /* Set Speed. */
  USBD_LL_SetSpeed(USBD_SPEED_FULL);

  /* Reset Device. */
  USBD_LL_Reset();
}

/**
  * @brief  Suspend callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_SuspendCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_SuspendCallback()
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  /* Inform USB library that core enters in suspend Mode. */
  USBD_LL_Suspend();
  __HAL_PCD_GATE_PHYCLOCK();
  /* Enter in STOP mode. */
  /* USER CODE BEGIN 2 */
  if (hpcd_USB_OTG_FS.Init.low_power_enable)
  {
    /* Set SLEEPDEEP bit and SleepOnExit of Cortex System Control Register. */
    SCB->SCR |= (uint32_t)((uint32_t)(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk));
  }
  /* USER CODE END 2 */
}

/**
  * @brief  Resume callback.
  * When Low power mode is enabled the debug cannot be used (IAR, Keil doesn't support it)
  * @param  hpcd: PCD handle
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ResumeCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_ResumeCallback()
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
  USBD_LL_Resume();
}

/**
  * @brief  ISOOUTIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint number
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void HAL_PCD_ISOOUTIncompleteCallback(uint8_t epnum)
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  USBD_LL_IsoOUTIncomplete(epnum);
}

/**
  * @brief  ISOINIncomplete callback.
  * @param  hpcd: PCD handle
  * @param  epnum: Endpoint number
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum)
#else
void HAL_PCD_ISOINIncompleteCallback(uint8_t epnum)
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  USBD_LL_IsoINIncomplete(epnum);
}

/**
  * @brief  Connect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_ConnectCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_ConnectCallback()
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  USBD_LL_DevConnected();
}

/**
  * @brief  Disconnect callback.
  * @param  hpcd: PCD handle
  * @retval None
  */
#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
static void PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd)
#else
void HAL_PCD_DisconnectCallback()
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
{
  USBD_LL_DevDisconnected();
}

/*******************************************************************************
                       LL Driver Interface (USB Device Library --> PCD)
*******************************************************************************/

/**
  * @brief  Initializes the low level portion of the device driver.
  * @param  pdev: Device handle
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_Init()
{
  /* Init USB Ip. */
  /* Link the driver to the stack. */
  hpcd_USB_OTG_FS.pData = &hUsbDeviceFS;
  hUsbDeviceFS.pData = &hpcd_USB_OTG_FS;
  
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init() != HAL_OK)
  {
    Error_Handler( );
  }

#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
  /* Register USB PCD CallBacks */
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_SOF_CB_ID, PCD_SOFCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_SETUPSTAGE_CB_ID, PCD_SetupStageCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_RESET_CB_ID, PCD_ResetCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_SUSPEND_CB_ID, PCD_SuspendCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_RESUME_CB_ID, PCD_ResumeCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_CONNECT_CB_ID, PCD_ConnectCallback);
  HAL_PCD_RegisterCallback(&hpcd_USB_OTG_FS, HAL_PCD_DISCONNECT_CB_ID, PCD_DisconnectCallback);

  HAL_PCD_RegisterDataOutStageCallback(&hpcd_USB_OTG_FS, PCD_DataOutStageCallback);
  HAL_PCD_RegisterDataInStageCallback(&hpcd_USB_OTG_FS, PCD_DataInStageCallback);
  HAL_PCD_RegisterIsoOutIncpltCallback(&hpcd_USB_OTG_FS, PCD_ISOOUTIncompleteCallback);
  HAL_PCD_RegisterIsoInIncpltCallback(&hpcd_USB_OTG_FS, PCD_ISOINIncompleteCallback);
#endif /* USE_HAL_PCD_REGISTER_CALLBACKS */
  HAL_PCDEx_SetRxFiFo(0x80); // 512 bytes
  HAL_PCDEx_SetTxFiFo(0, 0x40); // 256 bytes
  HAL_PCDEx_SetTxFiFo(1, 0x80); // 512 bytes
  return USBD_OK;
}

/**
  * @brief  De-Initializes the low level portion of the device driver.
  * @param  pdev: Device handle
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_DeInit()
{
  HAL_PCD_DeInit();
  return USBD_OK; 
}

/**
  * @brief  Stops the low level portion of the device driver.
  * @param  pdev: Device handle
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_Stop()
{
  // returns 1 or 0 --  1 = error
  return (USBD_StatusTypeDef)HAL_PCD_Stop();
}

/**
  * @brief  Opens an endpoint of the low level driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint number
  * @param  ep_type: Endpoint type
  * @param  ep_mps: Endpoint max packet size
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_OpenEP(uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps)
{
  HAL_PCD_EP_Open(ep_addr, ep_mps, ep_type);
  return USBD_OK;
}

/**
  * @brief  Closes an endpoint of the low level driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint number
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_CloseEP(uint8_t ep_addr)
{
  HAL_PCD_EP_Close(ep_addr);
  return USBD_OK;  
}

/**
  * @brief  Flushes an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint number
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_FlushEP(uint8_t ep_addr)
{
  HAL_PCD_EP_Flush(ep_addr);
  return USBD_OK;  
}

/**
  * @brief  Sets a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint number
  * @retval USBD status
  */
// USBD_StatusTypeDef USBD_LL_StallEP(uint8_t ep_addr)
// {
//   HAL_PCD_EP_SetStall(ep_addr);
//   return USBD_OK;  
// }

/**
  * @brief  Clears a Stall condition on an endpoint of the Low Level Driver.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint number
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_ClearStallEP(uint8_t ep_addr)
{
  HAL_PCD_EP_ClrStall(ep_addr);
  return USBD_OK; 
}

/**
  * @brief  Returns Stall condition.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint number
  * @retval Stall (1: Yes, 0: No)
  */
uint8_t USBD_LL_IsStallEP(uint8_t ep_addr)
{
  if((ep_addr & 0x80) == 0x80)
  {
    return hpcd_USB_OTG_FS.IN_ep[ep_addr & 0x7F].is_stall; 
  }
  else
  {
    return hpcd_USB_OTG_FS.OUT_ep[ep_addr & 0x7F].is_stall; 
  }
}

/**
  * @brief  Assigns a USB address to the device.
  * @param  pdev: Device handle
  * @param  dev_addr: Device address
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_SetUSBAddress(uint8_t dev_addr)
{
  HAL_PCD_SetAddress(dev_addr);
  return USBD_OK;  
}

/**
  * @brief  Transmits data over an endpoint.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint number
  * @param  pbuf: Pointer to data to be sent
  * @param  size: Data size    
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_Transmit(uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Transmit(ep_addr, pbuf, size);
  return USBD_OK;    
}

/**
  * @brief  Prepares an endpoint for reception.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint number
  * @param  pbuf: Pointer to data to be received
  * @param  size: Data size
  * @retval USBD status
  */
USBD_StatusTypeDef USBD_LL_PrepareReceive(uint8_t ep_addr, uint8_t *pbuf, uint32_t size)
{
  HAL_PCD_EP_Receive(ep_addr, pbuf, size);
  return USBD_OK; 
}

/**
  * @brief  Returns the last transfered packet size.
  * @param  pdev: Device handle
  * @param  ep_addr: Endpoint number
  * @retval Recived Data Size
  */
uint32_t USBD_LL_GetRxDataSize(uint8_t ep_addr)
{
  return HAL_PCD_EP_GetRxCount(ep_addr);
}

/**
  * @brief  Delays routine for the USB Device Library.
  * @param  Delay: Delay in ms
  * @retval None
  */
void USBD_LL_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
