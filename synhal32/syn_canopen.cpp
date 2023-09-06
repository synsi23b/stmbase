#include "synhal.h"
#include "../CANopenNode/CANopen.h"
#include "../CANopenNode/301/CO_driver.h"

#include <CO_storageBlank.h>
#include <OD.h>
#include "CO_driver_target.h"

#include <stdio.h>

#include <stm32f1xx_hal_can.h>

/* default values for CO_CANopenInit() */
#define NMT_CONTROL ((CO_NMT_control_t)uint16_t(CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION))
#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL


CAN_HandleTypeDef hcan;

#define CAN_BTR_TS1_Pos (16U)
#define CAN_BTR_TS1_0 (0x1UL << CAN_BTR_TS1_Pos)                                 /*!< 0x00010000 */
#define CAN_BTR_TS1_1 (0x2UL << CAN_BTR_TS1_Pos)                                 /*!< 0x00020000 */
#define CAN_BTR_TS1_2 (0x4UL << CAN_BTR_TS1_Pos)                                 /*!< 0x00040000 */
#define CAN_BTR_TS1_3 (0x8UL << CAN_BTR_TS1_Pos)                                 /*!< 0x00080000 */
#define CAN_BS1_15TQ ((uint32_t)(CAN_BTR_TS1_3 | CAN_BTR_TS1_2 | CAN_BTR_TS1_1)) /*!< 15 time quantum */

#define CAN_BTR_TS2_Pos (20U)
#define CAN_BTR_TS2_0 (0x1UL << CAN_BTR_TS2_Pos) /*!< 0x00100000 */
#define CAN_BTR_TS2_1 (0x2UL << CAN_BTR_TS2_Pos) /*!< 0x00200000 */
#define CAN_BTR_TS2_2 (0x4UL << CAN_BTR_TS2_Pos) /*!< 0x00400000 */
#define CAN_BS2_2TQ ((uint32_t)CAN_BTR_TS2_0)

#define CAN_TX_MAILBOX0             (0x00000001U)  /*!< Tx Mailbox 0  */
#define CAN_TX_MAILBOX1             (0x00000002U)  /*!< Tx Mailbox 1  */
#define CAN_TX_MAILBOX2             (0x00000004U)  /*!< Tx Mailbox 2  */

static void MX_CAN_Init(void)
{

    /* USER CODE BEGIN CAN_Init 0 */

    /* USER CODE END CAN_Init 0 */

    /* USER CODE BEGIN CAN_Init 1 */

    /* USER CODE END CAN_Init 1 */
    hcan.Instance = CAN1;
    hcan.Init.Prescaler = 4;
    hcan.Init.Mode = CAN_MODE_NORMAL;
    hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
    hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan.Init.TimeTriggeredMode = DISABLE;
    hcan.Init.AutoBusOff = ENABLE;
    hcan.Init.AutoWakeUp = DISABLE;
    hcan.Init.AutoRetransmission = DISABLE;
    hcan.Init.ReceiveFifoLocked = DISABLE;
    hcan.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan) != HAL_OK)
    {
        while (true)
            ;
    }
    /* USER CODE BEGIN CAN_Init 2 */

    /* USER CODE END CAN_Init 2 */
}

/* Global variables and objects */
CO_t *CO = NULL; /* CANopen object */
static uint32_t process_time_point = 1000;
static uint16_t baudrate;
static uint8_t desiredNodeID;
static uint8_t activeNodeID; /* Assigned Node ID */

CO_ReturnError_t err;
syn::CANopenNode::CANopenSYNC syn::CANopenNode::_synctimer;


/* Local CAN module object */
static CO_CANmodule_t *CANModule_local = NULL; /* Local instance of global CAN module */

/* CAN masks for identifiers */
#define CANID_MASK 0x07FF /*!< CAN standard ID mask */
#define FLAG_RTR 0x8000   /*!< RTR flag, part of identifier */

/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr)
{
    /* Put CAN module in configuration mode */
        HAL_CAN_Stop(&hcan);
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    /* Put CAN module in normal mode */
        if (HAL_CAN_Start(&hcan) == HAL_OK)
        {
            CANmodule->CANnormal = true;
        }
}

/******************************************************************************/
CO_ReturnError_t
CO_CANmodule_init(CO_CANmodule_t *CANmodule, void *CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                  uint16_t txSize, uint16_t CANbitRate)
{

    /* verify arguments */
    if (CANmodule == NULL || rxArray == NULL || txArray == NULL)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Hold CANModule variable */
    CANmodule->CANptr = CANptr;

    /* Keep a local copy of CANModule */
    CANModule_local = CANmodule;

    /* Configure object variables */
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false; /* Do not use HW filters */
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;

    /* Reset all variables */
    for (uint16_t i = 0U; i < rxSize; i++)
    {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }
    for (uint16_t i = 0U; i < txSize; i++)
    {
        txArray[i].bufferFull = false;
    }

    /***************************************/
    /* STM32 related configuration */
    /***************************************/
    MX_CAN_Init();

    /*
     * Configure global filter that is used as last check if message did not pass any of other filters:
     *
     * We do not rely on hardware filters in this example
     * and are performing software filters instead
     *
     * Accept non-matching standard ID messages
     * Reject non-matching extended ID messages
     */


    CAN_FilterTypeDef FilterConfig;
#if defined(CAN)
        FilterConfig.FilterBank = 0;
#else
        if (hcan.Instance == CAN1)
        {
            FilterConfig.FilterBank = 0;
        }
        else
        {
            FilterConfig.FilterBank = 14;
        }
#endif
    FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    FilterConfig.FilterIdHigh = 0x0;
    FilterConfig.FilterIdLow = 0x0;
    FilterConfig.FilterMaskIdHigh = 0x0;
    FilterConfig.FilterMaskIdLow = 0x0;
    FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

    FilterConfig.FilterActivation = ENABLE;
    FilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &FilterConfig) != HAL_OK)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Enable notifications */
    /* Activate the CAN notification interrupts */
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
    {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }
    return CO_ERROR_NO;
}

/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    HAL_CAN_Stop(&hcan);
}

/******************************************************************************/
CO_ReturnError_t
CO_CANrxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident, uint16_t mask, bool_t rtr, void *object,
                   void (*CANrx_callback)(void *object, void *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if (CANmodule != NULL && object != NULL && CANrx_callback != NULL && index < CANmodule->rxSize)
    {
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->CANrx_callback = CANrx_callback;

        /*
         * Configure global identifier, including RTR bit
         *
         * This is later used for RX operation match case
         */
        buffer->ident = (ident & CANID_MASK) | (rtr ? FLAG_RTR : 0x00);
        buffer->mask = (mask & CANID_MASK) | FLAG_RTR;

        /* Set CAN hardware module filter and mask. */
        if (CANmodule->useCANrxFilters)
        {
            __NOP();
        }
    }
    else
    {
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}

/******************************************************************************/
CO_CANtx_t *
CO_CANtxBufferInit(CO_CANmodule_t *CANmodule, uint16_t index, uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                   bool_t syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if (CANmodule != NULL && index < CANmodule->txSize)
    {
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr, bit aligned with CAN module transmit buffer */
        buffer->ident = ((uint32_t)ident & CANID_MASK) | ((uint32_t)(rtr ? FLAG_RTR : 0x00));
        buffer->DLC = noOfBytes;
        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }
    return buffer;
}

/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       CANmodule: CAN module instance
 * \param[in]       buffer: Pointer to buffer to transmit
 */
static uint8_t
prv_send_can_message(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{

    uint8_t success = 0;

    /* Check if TX FIFO is ready to accept more messages */
    static CAN_TxHeaderTypeDef tx_hdr;
    /* Check if TX FIFO is ready to accept more messages */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0)
    {
        /*
         * RTR flag is part of identifier value
         * hence it needs to be properly decoded
         */
        tx_hdr.ExtId = 0u;
        tx_hdr.IDE = CAN_ID_STD;
        tx_hdr.DLC = buffer->DLC;
        tx_hdr.StdId = buffer->ident & CANID_MASK;
        tx_hdr.RTR = (buffer->ident & FLAG_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;

        uint32_t TxMailboxNum; // Transmission MailBox number

        /* Now add message to FIFO. Should not fail */
        success = HAL_CAN_AddTxMessage(&hcan, &tx_hdr, buffer->data,
                                       &TxMailboxNum) == HAL_OK;
    }
    return success;
}

/******************************************************************************/
CO_ReturnError_t
CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull)
    {
        if (!CANmodule->firstCANtxMessage)
        {
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    /*
     * Send message to CAN network
     *
     * Lock interrupts for atomic operation
     */
    CO_LOCK_CAN_SEND(CANmodule);
    if (prv_send_can_message(CANmodule, buffer))
    {
        CANmodule->bufferInhibitFlag = buffer->syncFlag;
    }
    else
    {
        buffer->bufferFull = true;
        CANmodule->CANtxCount += 1;
    }
    CO_UNLOCK_CAN_SEND(CANmodule);

    return err;
}

/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND(CANmodule);
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if (/*messageIsOnCanBuffer && */ CANmodule->bufferInhibitFlag)
    {
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if (CANmodule->CANtxCount > 0)
    {
        for (uint16_t i = CANmodule->txSize; i > 0U; --i)
        {
            if (CANmodule->txArray[i].bufferFull)
            {
                if (CANmodule->txArray[i].syncFlag)
                {
                    CANmodule->txArray[i].bufferFull = false;
                    CANmodule->CANtxCount -= 1;
                    tpdoDeleted = 2U;
                }
            }
        }
    }
    CO_UNLOCK_CAN_SEND(CANmodule);
    if (tpdoDeleted)
    {
        CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
    }
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */
static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    uint32_t err = 0;

    // CANOpen just care about Bus_off, Warning, Passive and Overflow
    // I didn't find overflow error register in STM32, if you find it please let me know

    err = hcan.Instance->ESR & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF);

    //    uint32_t esrVal = ((CAN_HandleTypeDef*)((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle)->Instance->ESR; Debug purpose
    if (CANmodule->errOld != err)
    {

        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (err & CAN_ESR_BOFF)
        {
            status |= CO_CAN_ERRTX_BUS_OFF;
            // In this driver, we assume that auto bus recovery is activated ! so this error will eventually handled automatically.
        }
        else
        {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING | CO_CAN_ERRTX_PASSIVE);

            if (err & CAN_ESR_EWGF)
            {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
            }

            if (err & CAN_ESR_EPVF)
            {
                status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
            }
        }

        CANmodule->CANerrorStatus = status;
    }
}

/**
 * \brief           Read message from RX FIFO
 * \param           hfdcan: pointer to an FDCAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified FDCAN.
 * \param[in]       fifo: Fifo number to use for read
 * \param[in]       fifo_isrs: List of interrupts for respected FIFO
 */
static void
prv_read_can_received_msg(CAN_HandleTypeDef *hcan, uint32_t fifo, uint32_t fifo_isrs)
{

    CO_CANrxMsg_t rcvMsg;
    CO_CANrx_t *buffer = NULL; /* receive message buffer from CO_CANmodule_t object. */
    uint16_t index;            /* index of received message */
    uint32_t rcvMsgIdent;      /* identifier of the received message */
    uint8_t messageFound = 0;

    static CAN_RxHeaderTypeDef rx_hdr;
    /* Read received message from FIFO */
    if (HAL_CAN_GetRxMessage(hcan, fifo, &rx_hdr, rcvMsg.data) != HAL_OK)
    {
        return;
    }
    /* Setup identifier (with RTR) and length */
    rcvMsg.ident = rx_hdr.StdId | (rx_hdr.RTR == CAN_RTR_REMOTE ? FLAG_RTR : 0x00);
    rcvMsg.dlc = rx_hdr.DLC;
    rcvMsgIdent = rcvMsg.ident;

    /*
     * Hardware filters are not used for the moment
     * \todo: Implement hardware filters...
     */
    if (CANModule_local->useCANrxFilters)
    {
        __BKPT(0);
    }
    else
    {
        /*
         * We are not using hardware filters, hence it is necessary
         * to manually match received message ID with all buffers
         */
        buffer = CANModule_local->rxArray;
        for (index = CANModule_local->rxSize; index > 0U; --index, ++buffer)
        {
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
            {
                messageFound = 1;
                break;
            }
        }
    }

    /* Call specific function, which will process the message */
    if (messageFound && buffer != NULL && buffer->CANrx_callback != NULL)
    {
        buffer->CANrx_callback(buffer->object, (void *)&rcvMsg);
    }
}


/**
 * \brief           Rx FIFO 0 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    prv_read_can_received_msg(hcan, CAN_RX_FIFO0, 0);
}

/**
 * \brief           Rx FIFO 1 callback.
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    prv_read_can_received_msg(hcan, CAN_RX_FIFO1, 0);
}

/**
 * \brief           TX buffer has been well transmitted callback
 * \param[in]       hcan: pointer to an CAN_HandleTypeDef structure that contains
 *                      the configuration information for the specified CAN.
 * \param[in]       MailboxNumber: the mailbox number that has been transmitted
 */
void CO_CANinterrupt_TX(CO_CANmodule_t *CANmodule, uint32_t MailboxNumber)
{

    CANmodule->firstCANtxMessage = false; /* First CAN message (bootup) was sent successfully */
    CANmodule->bufferInhibitFlag = false; /* Clear flag from previous message */
    if (CANmodule->CANtxCount > 0U)
    {                                                /* Are there any new messages waiting to be send */
        CO_CANtx_t *buffer = &CANmodule->txArray[0]; /* Start with first buffer handle */
        uint16_t i;

        /*
         * Try to send more buffers, process all empty ones
         *
         * This function is always called from interrupt,
         * however to make sure no preemption can happen, interrupts are anyway locked
         * (unless you can guarantee no higher priority interrupt will try to access to CAN instance and send data,
         *  then no need to lock interrupts..)
         */
        CO_LOCK_CAN_SEND(CANmodule);
        for (i = CANmodule->txSize; i > 0U; --i, ++buffer)
        {
            /* Try to send message */
            if (buffer->bufferFull)
            {
                if (prv_send_can_message(CANmodule, buffer))
                {
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount -= 1;
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                }
            }
        }
        /* Clear counter if no more messages */
        if (i == 0U)
        {
            CANmodule->CANtxCount = 0U;
        }
        CO_UNLOCK_CAN_SEND(CANmodule);
    }
}

using namespace syn;

int32_t CANopenNode::init(uint8_t desired_id, uint16_t baudrate_k)
{
    _synctimer.init(1, true);

    desiredNodeID = desired_id;
    baudrate = baudrate_k;

    uint32_t timeslices;
    switch (baudrate_k)
    {
    case 500:
        timeslices = 0x123;
        break;
    default:
        while (true)
        {
            ;
        }
    }
    // enable clock for hw
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    // initialize GPIO
    // TODO remap pins according to afio register?
    {
        Gpio rx('a', 11);
        rx.mode(Gpio::in_floating, Gpio::Input);
    }
    {
        Gpio tx('a', 12);
        tx.mode(Gpio::out_alt_push_pull, Gpio::MHz_50, Gpio::Alternate::CAN);
    }

    // enable CAN interrupts
    Core::enable_isr(USB_HP_CAN1_TX_IRQn, 5);
    Core::enable_isr(USB_LP_CAN1_RX0_IRQn, 5);
    Core::enable_isr(CAN1_RX1_IRQn, 5);
    Core::enable_isr(CAN1_SCE_IRQn, 5);

    // CAN1->IER |= CAN_IER_TMEIE;

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    CO_storage_t storage;
    CO_storage_entry_t storageEntries[] = {{.addr = &OD_PERSIST_COMM,
                                            .len = sizeof(OD_PERSIST_COMM),
                                            .subIndexOD = 2,
                                            .attr = CO_storage_cmd | CO_storage_restore,
                                            .addrNV = NULL}};
    uint8_t storageEntriesCount = sizeof(storageEntries) / sizeof(storageEntries[0]);
    uint32_t storageInitError = 0;
#endif

    /* Allocate memory */
    CO_config_t *config_ptr = NULL;
#ifdef CO_MULTIPLE_OD
    /* example usage of CO_MULTIPLE_OD (but still single OD here) */
    CO_config_t co_config = {0};
    OD_INIT_CONFIG(co_config); /* helper macro from OD.h */
    co_config.CNT_LEDS = 1;
    co_config.CNT_LSS_SLV = 1;
    config_ptr = &co_config;
#endif /* CO_MULTIPLE_OD */

    uint32_t heapMemoryUsed;
    CO = CO_new(config_ptr, &heapMemoryUsed);
#ifndef CO_USE_GLOBALS
    if (CO == NULL)
    {
        printf("Error: Can't allocate memory\n");
        return 1;
    }
    else
    {
        printf("Allocated %u bytes for CANopen objects\n", heapMemoryUsed);
    }
#endif
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage, CO->CANmodule, OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT)
    {
        printf("Error: Storage %d\n", storageInitError);
        return 2;
    }
#endif
    reset_com();
    return 0;
}

void CANopenNode::process(syn::Gpio &led_green, syn::Gpio &led_red)
{
    /* loop for normal program execution ******************************************/
    /* get time difference since last function call */
    uint32_t time_current = System::microseconds();
    uint32_t time_delta = process_time_point - time_current;

    if (static_cast<int32_t>(time_delta) <= 0)
    {   // Make sure more than 1ms elapsed
        uint32_t timeDifference_us = (-static_cast<int32_t>(time_delta)) + 1000;
        // setup next check for expired at least 1ms in the future;
        process_time_point = time_current + 1000;
        /* CANopen process */
        CO_NMT_reset_cmd_t reset_status;
        reset_status = CO_process(CO, false, timeDifference_us, NULL);

        // set LED status lights
        led_green.set(CO_LED_GREEN(CO->LEDs, CO_LED_CANopen));
        led_red.set(CO_LED_RED(CO->LEDs, CO_LED_CANopen));

        if (reset_status == CO_RESET_COMM)
        {
            /* delete objects from memory */
            CO_CANsetConfigurationMode(NULL);
            CO_delete(CO);
            printf("CANopenNode Reset Communication request\n");
            reset_com(); // Reset Communication routine
        }
        else if (reset_status == CO_RESET_APP)
        {
            printf("CANopenNode Device Reset\n");
            __NVIC_SystemReset(); // Reset the STM32 Microcontroller
        }
    }
}

int32_t CANopenNode::reset_com()
{
    /* CANopen communication reset - initialize CANopen objects *******************/
    printf("CANopenNode - Reset communication...\n");

    /* Wait rt_thread. */
    CO->CANmodule->CANnormal = false;

    /* Enter CAN configuration. */
    CO_CANsetConfigurationMode(NULL);
    CO_CANmodule_disable(CO->CANmodule);

    /* initialize CANopen */
    err = CO_CANinit(CO, 0, 0); // Bitrate for STM32 microcontroller is being set in MXCube Settings
    if (err != CO_ERROR_NO)
    {
        printf("Error: CAN initialization failed: %d\n", err);
        return 1;
    }

    CO_LSS_address_t lssAddress = {.identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                                .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                                .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                                .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};
    err = CO_LSSinit(CO, &lssAddress, &desiredNodeID, &baudrate);
    if (err != CO_ERROR_NO)
    {
        printf("Error: LSS slave initialization failed: %d\n", err);
        return 2;
    }

    activeNodeID = desiredNodeID;
    uint32_t errInfo = 0;

    err = CO_CANopenInit(CO,                   /* CANopen object */
                         NULL,                 /* alternate NMT */
                         NULL,                 /* alternate em */
                         OD,                   /* Object dictionary */
                         OD_STATUS_BITS,       /* Optional OD_statusBits */
                         NMT_CONTROL,          /* CO_NMT_control_t */
                         FIRST_HB_TIME,        /* firstHBTime_ms */
                         SDO_SRV_TIMEOUT_TIME, /* SDOserverTimeoutTime_ms */
                         SDO_CLI_TIMEOUT_TIME, /* SDOclientTimeoutTime_ms */
                         SDO_CLI_BLOCK,        /* SDOclientBlockTransfer */
                         activeNodeID, &errInfo);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS)
    {
        if (err == CO_ERROR_OD_PARAMETERS)
        {
            printf("Error: Object Dictionary entry 0x%X\n", errInfo);
        }
        else
        {
            printf("Error: CANopen initialization failed: %d\n", err);
        }
        return 3;
    }

    err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeID, &errInfo);
    if (err != CO_ERROR_NO)
    {
        if (err == CO_ERROR_OD_PARAMETERS)
        {
            printf("Error: Object Dictionary entry 0x%X\n", errInfo);
        }
        else
        {
            printf("Error: PDO initialization failed: %d\n", err);
        }
        return 4;
    }

    /* Restart Timer interrupt function for execution every 1 millisecond */
    _synctimer.restart();
    /* Configure CAN transmit and receive interrupt */

    /* Configure CANopen callbacks, etc */
    if (!CO->nodeIdUnconfigured)
    {

#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
        if (storageInitError != 0)
        {
            CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
        }
#endif
    }
    else
    {
        printf("CANopenNode - Node-id not initialized\n");
    }

    /* start CAN */
    CO_CANsetNormalMode(CO->CANmodule);

    printf("CANopenNode - Running...\n");
    fflush(stdout);
    process_time_point = System::microseconds() + 1000;
    return 0;
}

void CANopenNode::CANopenSYNC::execute()
{
    CO_LOCK_OD(CO->CANmodule);
    if (!CO->nodeIdUnconfigured && CO->CANmodule->CANnormal)
    {
        bool_t syncWas = false;
        /* get time difference since last function call */
        uint32_t timeDifference_us = 1000; // 1ms second

#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
        syncWas = CO_process_SYNC(CO, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
        CO_process_RPDO(CO, syncWas, timeDifference_us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
        CO_process_TPDO(CO, syncWas, timeDifference_us, NULL);
#endif

        /* Further I/O or nonblocking application code may go here. */
    }
    CO_UNLOCK_OD(CO->CANmodule);
}

extern "C"
{
    // void USB_HP_CAN1_TX_IRQHandler(void)
    // {
    //     /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 0 */

    //     /* USER CODE END USB_HP_CAN1_TX_IRQn 0 */
    //     HAL_CAN_IRQHandler(&hcan);
    //     /* USER CODE BEGIN USB_HP_CAN1_TX_IRQn 1 */

    //     /* USER CODE END USB_HP_CAN1_TX_IRQn 1 */
    // }

    /**
     * @brief This function handles USB low priority or CAN RX0 interrupts.
     */
    void USB_LP_CAN1_RX0_IRQHandler(void)
    {
        /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

        /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
        HAL_CAN_IRQHandler(&hcan);
        /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

        /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
    }

    /**
     * @brief This function handles CAN RX1 interrupt.
     */
    void CAN1_RX1_IRQHandler(void)
    {
        /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

        /* USER CODE END CAN1_RX1_IRQn 0 */
        HAL_CAN_IRQHandler(&hcan);
        /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

        /* USER CODE END CAN1_RX1_IRQn 1 */
    }

        void USB_HP_CAN1_TX_IRQHandler(void)
        {
        uint32_t errorcode = HAL_CAN_ERROR_NONE;
            uint32_t tsrflags = CAN1->TSR;
            /* Transmit Mailbox 0 management *****************************************/
            if ((tsrflags & CAN_TSR_RQCP0) != 0U)
            {
                /* Clear the Transmission Complete flag (and TXOK0,ALST0,TERR0 bits) */
                CAN1->TSR |= CAN_TSR_RQCP0;

                if ((tsrflags & CAN_TSR_TXOK0) != 0U)
                {
                    CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX0);
                }
                else
                {
                    if ((tsrflags & CAN_TSR_ALST0) != 0U)
                    {
                        /* Update error code */
                        errorcode |= HAL_CAN_ERROR_TX_ALST0;
                    }
                    else if ((tsrflags & CAN_TSR_TERR0) != 0U)
                    {
                        /* Update error code */
                        errorcode |= HAL_CAN_ERROR_TX_TERR0;
                    }
                    else
                    {
                        /* Transmission Mailbox 0 abort callback */
    #if USE_HAL_CAN_REGISTER_CALLBACKS == 1
                        /* Call registered callback*/
                        hcan->TxMailbox0AbortCallback(hcan);
    #else
                        /* Call weak (surcharged) callback */
                        //HAL_CAN_TxMailbox0AbortCallback(hcan);
    #endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
                    }
                }
            }

            /* Transmit Mailbox 1 management *****************************************/
            if ((tsrflags & CAN_TSR_RQCP1) != 0U)
            {
                /* Clear the Transmission Complete flag (and TXOK1,ALST1,TERR1 bits) */
                CAN1->TSR |= CAN_TSR_RQCP1;

                if ((tsrflags & CAN_TSR_TXOK1) != 0U)
                {
                    /* Transmission Mailbox 1 complete callback */
                    CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX1);
                }
                else
                {
                    if ((tsrflags & CAN_TSR_ALST1) != 0U)
                    {
                        /* Update error code */
                        errorcode |= HAL_CAN_ERROR_TX_ALST1;
                    }
                    else if ((tsrflags & CAN_TSR_TERR1) != 0U)
                    {
                        /* Update error code */
                        errorcode |= HAL_CAN_ERROR_TX_TERR1;
                    }
                    else
                    {
                        /* Transmission Mailbox 1 abort callback */
    #if USE_HAL_CAN_REGISTER_CALLBACKS == 1
                        /* Call registered callback*/
                        hcan->TxMailbox1AbortCallback(hcan);
    #else
                        /* Call weak (surcharged) callback */
                        //HAL_CAN_TxMailbox1AbortCallback(hcan);
    #endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
                    }
                }
            }

            /* Transmit Mailbox 2 management *****************************************/
            if ((tsrflags & CAN_TSR_RQCP2) != 0U)
            {
                /* Clear the Transmission Complete flag (and TXOK2,ALST2,TERR2 bits) */
                CAN1->TSR |= CAN_TSR_RQCP2;

                if ((tsrflags & CAN_TSR_TXOK2) != 0U)
                {
                    CO_CANinterrupt_TX(CANModule_local, CAN_TX_MAILBOX2);
                }
                else
                {
                    if ((tsrflags & CAN_TSR_ALST2) != 0U)
                    {
                        /* Update error code */
                        errorcode |= HAL_CAN_ERROR_TX_ALST2;
                    }
                    else if ((tsrflags & CAN_TSR_TERR2) != 0U)
                    {
                        /* Update error code */
                        errorcode |= HAL_CAN_ERROR_TX_TERR2;
                    }
                    else
                    {
                        /* Transmission Mailbox 2 abort callback */
    #if USE_HAL_CAN_REGISTER_CALLBACKS == 1
                        /* Call registered callback*/
                        hcan->TxMailbox2AbortCallback(hcan);
    #else
                        /* Call weak (surcharged) callback */
                       // HAL_CAN_TxMailbox2AbortCallback(hcan);
    #endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
                    }
                }
            }
              /* Call the Error call Back in case of Errors */
  if (errorcode != HAL_CAN_ERROR_NONE)
  {
    /* Update error code in handle */
    hcan.ErrorCode |= errorcode;

    /* Call Error callback function */
#if USE_HAL_CAN_REGISTER_CALLBACKS == 1
    /* Call registered callback*/
    hcan->ErrorCallback(hcan);
#else
    /* Call weak (surcharged) callback */
    //HAL_CAN_ErrorCallback(hcan);
#endif /* USE_HAL_CAN_REGISTER_CALLBACKS */
  }
        }

    //     void USB_LP_CAN1_RX0_IRQHandler(void)
    //     {
    //     }

    //     void CAN1_RX1_IRQHandler(void)
    //     {
    //     }

    //     void CAN1_SCE_IRQHandler(void)
    //     {
    //     }
}