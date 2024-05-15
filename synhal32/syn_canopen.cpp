#include "synhal.h"
#include "../CANopenNode/CANopen.h"
#include "../CANopenNode/301/CO_driver.h"
#include "../CANopenNode/301/CO_ODinterface.h"
#include "../CANopenNode/305/CO_LSSslave.h"

#include <CO_storage_target.h>
#include <OD.h>
#include "CO_driver_target.h"

#include <stdio.h>

#ifdef NDEBUG
#define log_printf(macropar_message, ...) \
    (void)(macropar_message)
#else
#define log_printf(macropar_message, ...) \
    printf(macropar_message, ##__VA_ARGS__)
#endif


#define HAL_CAN_ERROR_NONE (0x00000000U)            /*!< No error                                             */
#define HAL_CAN_ERROR_EWG (0x00000001U)             /*!< Protocol Error Warning                               */
#define HAL_CAN_ERROR_EPV (0x00000002U)             /*!< Error Passive                                        */
#define HAL_CAN_ERROR_BOF (0x00000004U)             /*!< Bus-off error                                        */
#define HAL_CAN_ERROR_STF (0x00000008U)             /*!< Stuff error                                          */
#define HAL_CAN_ERROR_FOR (0x00000010U)             /*!< Form error                                           */
#define HAL_CAN_ERROR_ACK (0x00000020U)             /*!< Acknowledgment error                                 */
#define HAL_CAN_ERROR_BR (0x00000040U)              /*!< Bit recessive error                                  */
#define HAL_CAN_ERROR_BD (0x00000080U)              /*!< Bit dominant error                                   */
#define HAL_CAN_ERROR_CRC (0x00000100U)             /*!< CRC error                                            */
#define HAL_CAN_ERROR_RX_FOV0 (0x00000200U)         /*!< Rx FIFO0 overrun error                               */
#define HAL_CAN_ERROR_RX_FOV1 (0x00000400U)         /*!< Rx FIFO1 overrun error                               */
#define HAL_CAN_ERROR_TX_ALST0 (0x00000800U)        /*!< TxMailbox 0 transmit failure due to arbitration lost */
#define HAL_CAN_ERROR_TX_TERR0 (0x00001000U)        /*!< TxMailbox 0 transmit failure due to transmit error    */
#define HAL_CAN_ERROR_TX_ALST1 (0x00002000U)        /*!< TxMailbox 1 transmit failure due to arbitration lost */
#define HAL_CAN_ERROR_TX_TERR1 (0x00004000U)        /*!< TxMailbox 1 transmit failure due to transmit error    */
#define HAL_CAN_ERROR_TX_ALST2 (0x00008000U)        /*!< TxMailbox 2 transmit failure due to arbitration lost */
#define HAL_CAN_ERROR_TX_TERR2 (0x00010000U)        /*!< TxMailbox 2 transmit failure due to transmit error    */
#define HAL_CAN_ERROR_TIMEOUT (0x00020000U)         /*!< Timeout error                                        */
#define HAL_CAN_ERROR_NOT_INITIALIZED (0x00040000U) /*!< Peripheral not initialized                           */
#define HAL_CAN_ERROR_NOT_READY (0x00080000U)       /*!< Peripheral not ready                                 */
#define HAL_CAN_ERROR_NOT_STARTED (0x00100000U)     /*!< Peripheral not started                               */
#define HAL_CAN_ERROR_PARAM (0x00200000U)           /*!< Parameter error */

/** @defgroup HAL_FDCAN_Error_Code HAL FDCAN Error Code
  * @{
  */
#define HAL_FDCAN_ERROR_NONE            ((uint32_t)0x00000000U) /*!< No error                                                               */
#define HAL_FDCAN_ERROR_TIMEOUT         ((uint32_t)0x00000001U) /*!< Timeout error                                                          */
#define HAL_FDCAN_ERROR_NOT_INITIALIZED ((uint32_t)0x00000002U) /*!< Peripheral not initialized                                             */
#define HAL_FDCAN_ERROR_NOT_READY       ((uint32_t)0x00000004U) /*!< Peripheral not ready                                                   */
#define HAL_FDCAN_ERROR_NOT_STARTED     ((uint32_t)0x00000008U) /*!< Peripheral not started                                                 */
#define HAL_FDCAN_ERROR_NOT_SUPPORTED   ((uint32_t)0x00000010U) /*!< Mode not supported                                                     */
#define HAL_FDCAN_ERROR_PARAM           ((uint32_t)0x00000020U) /*!< Parameter error                                                        */
#define HAL_FDCAN_ERROR_PENDING         ((uint32_t)0x00000040U) /*!< Pending operation                                                      */
#define HAL_FDCAN_ERROR_RAM_ACCESS      ((uint32_t)0x00000080U) /*!< Message RAM Access Failure                                             */
#define HAL_FDCAN_ERROR_FIFO_EMPTY      ((uint32_t)0x00000100U) /*!< Put element in full FIFO                                               */
#define HAL_FDCAN_ERROR_FIFO_FULL       ((uint32_t)0x00000200U) /*!< Get element from empty FIFO                                            */
#define HAL_FDCAN_ERROR_LOG_OVERFLOW    FDCAN_IR_ELO            /*!< Overflow of CAN Error Logging Counter                                  */
#define HAL_FDCAN_ERROR_RAM_WDG         FDCAN_IR_WDI            /*!< Message RAM Watchdog event occurred                                    */
#define HAL_FDCAN_ERROR_PROTOCOL_ARBT   FDCAN_IR_PEA            /*!< Protocol Error in Arbitration Phase (Nominal Bit Time is used)         */
#define HAL_FDCAN_ERROR_PROTOCOL_DATA   FDCAN_IR_PED            /*!< Protocol Error in Data Phase (Data Bit Time is used)                   */
#define HAL_FDCAN_ERROR_RESERVED_AREA   FDCAN_IR_ARA


#define FDCAN_ELEMENT_MASK_STDID ((uint32_t)0x1FFC0000U) /* Standard Identifier         */
#define FDCAN_ELEMENT_MASK_EXTID ((uint32_t)0x1FFFFFFFU) /* Extended Identifier         */
#define FDCAN_ELEMENT_MASK_RTR   ((uint32_t)0x20000000U) /* Remote Transmission Request */
#define FDCAN_ELEMENT_MASK_XTD   ((uint32_t)0x40000000U) /* Extended Identifier         */
#define FDCAN_ELEMENT_MASK_ESI   ((uint32_t)0x80000000U) /* Error State Indicator       */
#define FDCAN_ELEMENT_MASK_TS    ((uint32_t)0x0000FFFFU) /* Timestamp                   */
#define FDCAN_ELEMENT_MASK_DLC   ((uint32_t)0x000F0000U) /* Data Length Code            */
#define FDCAN_ELEMENT_MASK_BRS   ((uint32_t)0x00100000U) /* Bit Rate Switch             */
#define FDCAN_ELEMENT_MASK_FDF   ((uint32_t)0x00200000U) /* FD Format                   */
#define FDCAN_ELEMENT_MASK_EFC   ((uint32_t)0x00800000U) /* Event FIFO Control          */
#define FDCAN_ELEMENT_MASK_MM    ((uint32_t)0xFF000000U) /* Message Marker              */
#define FDCAN_ELEMENT_MASK_FIDX  ((uint32_t)0x7F000000U) /* Filter Index                */
#define FDCAN_ELEMENT_MASK_ANMF  ((uint32_t)0x80000000U) /* Accepted Non-matching Frame */
#define FDCAN_ELEMENT_MASK_ET    ((uint32_t)0x00C00000U) /* Event type                  */
#define FDCAN_CLASSIC_CAN ((uint32_t)0x00000000U) /*!< Frame transmitted/received in Classic CAN format */
#define FDCAN_FD_CAN      ((uint32_t)0x00200000U) /*!< Frame transmitted/received in FDCAN format       */
#define FDCAN_ESI_ACTIVE  ((uint32_t)0x00000000U) /*!< Transmitting node is error active  */
#define FDCAN_ESI_PASSIVE ((uint32_t)0x80000000U) /*!< Transmitting node is error passive */
#define FDCAN_BRS_OFF ((uint32_t)0x00000000U) /*!< FDCAN frames transmitted/received without bit rate switching */
#define FDCAN_BRS_ON  ((uint32_t)0x00100000U) /*!< FDCAN frames transmitted/received with bit rate switching    */
#define FDCAN_NO_TX_EVENTS    ((uint32_t)0x00000000U) /*!< Do not store Tx events */
#define FDCAN_STORE_TX_EVENTS ((uint32_t)0x00800000U) /*!< Store Tx events        */
#define FDCAN_TX_FIFO_OPERATION  ((uint32_t)0x00000000U)     /*!< FIFO mode  */
#define FDCAN_TX_QUEUE_OPERATION ((uint32_t)FDCAN_TXBC_TFQM) /*!< Queue mode */
#define FDCAN_RX_FIFO_BLOCKING  ((uint32_t)0x00000000U) /*!< Rx FIFO blocking mode  */
#define FDCAN_RX_FIFO_OVERWRITE ((uint32_t)0x00000001U) /*!< Rx FIFO overwrite mode */
#define FDCAN_STANDARD_ID ((uint32_t)0x00000000U) /*!< Standard ID element */
#define FDCAN_EXTENDED_ID ((uint32_t)0x40000000U) /*!< Extended ID element */
#define FDCAN_DATA_FRAME   ((uint32_t)0x00000000U)  /*!< Data frame   */
#define FDCAN_REMOTE_FRAME ((uint32_t)0x20000000U)  /*!< Remote frame */
#define FDCAN_DLC_BYTES_0  ((uint32_t)0x00000000U) /*!< 0 bytes data field  */
#define FDCAN_DLC_BYTES_1  ((uint32_t)0x00000001U) /*!< 1 bytes data field  */
#define FDCAN_DLC_BYTES_2  ((uint32_t)0x00000002U) /*!< 2 bytes data field  */
#define FDCAN_DLC_BYTES_3  ((uint32_t)0x00000003U) /*!< 3 bytes data field  */
#define FDCAN_DLC_BYTES_4  ((uint32_t)0x00000004U) /*!< 4 bytes data field  */
#define FDCAN_DLC_BYTES_5  ((uint32_t)0x00000005U) /*!< 5 bytes data field  */
#define FDCAN_DLC_BYTES_6  ((uint32_t)0x00000006U) /*!< 6 bytes data field  */
#define FDCAN_DLC_BYTES_7  ((uint32_t)0x00000007U) /*!< 7 bytes data field  */
#define FDCAN_DLC_BYTES_8  ((uint32_t)0x00000008U) /*!< 8 bytes data field  */
#define FDCAN_TIMEOUT_VALUE 10U
#define FDCAN_TX_EVENT_FIFO_MASK (FDCAN_IR_TEFL | FDCAN_IR_TEFF | FDCAN_IR_TEFN)
#define FDCAN_RX_FIFO0_MASK (FDCAN_IR_RF0L | FDCAN_IR_RF0F | FDCAN_IR_RF0N)
#define FDCAN_RX_FIFO1_MASK (FDCAN_IR_RF1L | FDCAN_IR_RF1F | FDCAN_IR_RF1N)
#define FDCAN_FRAME_CLASSIC   ((uint32_t)0x00000000U)                         /*!< Classic mode                      */
#define FDCAN_FRAME_FD_NO_BRS ((uint32_t)FDCAN_CCCR_FDOE)                     /*!< FD mode without BitRate Switching */
#define FDCAN_FRAME_FD_BRS    ((uint32_t)(FDCAN_CCCR_FDOE | FDCAN_CCCR_BRSE)) /*!< FD mode with BitRate Switching    */
#define FDCAN_ACCEPT_IN_RX_FIFO0 ((uint32_t)0x00000000U) /*!< Accept in Rx FIFO 0 */
#define FDCAN_ACCEPT_IN_RX_FIFO1 ((uint32_t)0x00000001U) /*!< Accept in Rx FIFO 1 */
#define FDCAN_REJECT             ((uint32_t)0x00000002U) /*!< Reject              */
#define FDCAN_FILTER_REMOTE ((uint32_t)0x00000000U) /*!< Filter remote frames */
#define FDCAN_REJECT_REMOTE ((uint32_t)0x00000001U) /*!< Reject all remote frames */
#define FDCAN_IT_TX_COMPLETE           FDCAN_IE_TCE   /*!< Transmission Completed                                */
#define FDCAN_IT_TX_ABORT_COMPLETE     FDCAN_IE_TCFE  /*!< Transmission Cancellation Finished                    */
#define FDCAN_IT_TX_FIFO_EMPTY         FDCAN_IE_TFEE  /*!< Tx FIFO Empty                                         */
#define FDCAN_IT_TIMESTAMP_WRAPAROUND  FDCAN_IE_TSWE  /*!< Timestamp counter wrapped around                      */
#define FDCAN_IT_TIMEOUT_OCCURRED      FDCAN_IE_TOOE  /*!< Timeout reached                                       */
#define FDCAN_IT_TX_EVT_FIFO_ELT_LOST  FDCAN_IE_TEFLE /*!< Tx Event FIFO element lost                 */
#define FDCAN_IT_TX_EVT_FIFO_FULL      FDCAN_IE_TEFFE /*!< Tx Event FIFO full                         */
#define FDCAN_IT_TX_EVT_FIFO_NEW_DATA  FDCAN_IE_TEFNE /*!< Tx Handler wrote Tx Event FIFO element     */
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST FDCAN_IE_RF0LE /*!< Rx FIFO 0 message lost                 */
#define FDCAN_IT_RX_FIFO0_FULL         FDCAN_IE_RF0FE /*!< Rx FIFO 0 full                         */
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE  FDCAN_IE_RF0NE /*!< New message written to Rx FIFO 0       */
#define FDCAN_IT_RX_FIFO1_MESSAGE_LOST FDCAN_IE_RF1LE /*!< Rx FIFO 1 message lost                 */
#define FDCAN_IT_RX_FIFO1_FULL         FDCAN_IE_RF1FE /*!< Rx FIFO 1 full                         */
#define FDCAN_IT_RX_FIFO1_NEW_MESSAGE  FDCAN_IE_RF1NE /*!< New message written to Rx FIFO 1       */
#define FDCAN_IT_RAM_ACCESS_FAILURE      FDCAN_IE_MRAFE /*!< Message RAM access failure occurred              */
#define FDCAN_IT_ERROR_LOGGING_OVERFLOW  FDCAN_IE_ELOE  /*!< Overflow of FDCAN Error Logging Counter occurred */
#define FDCAN_IT_RAM_WATCHDOG            FDCAN_IE_WDIE  /*!< Message RAM Watchdog event due to missing READY  */
#define FDCAN_IT_ARB_PROTOCOL_ERROR      FDCAN_IE_PEAE  /*!< Protocol error in arbitration phase detected     */
#define FDCAN_IT_DATA_PROTOCOL_ERROR     FDCAN_IE_PEDE  /*!< Protocol error in data phase detected            */
#define FDCAN_IT_RESERVED_ADDRESS_ACCESS FDCAN_IE_ARAE  /*!< Access to reserved address occurred              */
#define FDCAN_IT_ERROR_PASSIVE           FDCAN_IE_EPE   /*!< Error_Passive status changed      */
#define FDCAN_IT_ERROR_WARNING           FDCAN_IE_EWE   /*!< Error_Warning status changed      */
#define FDCAN_IT_BUS_OFF                 FDCAN_IE_BOE   /*!< Bus_Off status changed            */


/* CAN masks for identifiers */
#define CANID_MASK 0x07FF            /*!< CAN standard ID mask */
#define FLAG_RTR 0x8000              /*!< RTR flag, part of identifier */
#define CAN_RTR_DATA (0x00000000U)   /*!< Data frame   */
#define CAN_RTR_REMOTE (0x00000002U) /*!< Remote frame */
/* CAN register bits and masks*/
#define CAN_TI0R_STID_Pos (21U)


/* default values for CO_CANopenInit() */
#define NMT_CONTROL ((CO_NMT_control_t)uint16_t(CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION))
#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL


typedef enum
{
    HAL_CAN_STATE_RESET = 0x00U,         /*!< CAN not yet initialized or disabled */
    HAL_CAN_STATE_READY = 0x01U,         /*!< CAN initialized and ready for use   */
    HAL_CAN_STATE_LISTENING = 0x02U,     /*!< CAN receive process is ongoing      */
    HAL_CAN_STATE_SLEEP_PENDING = 0x03U, /*!< CAN sleep request is pending        */
    HAL_CAN_STATE_SLEEP_ACTIVE = 0x04U,  /*!< CAN sleep mode is active            */
    HAL_CAN_STATE_ERROR = 0x05U          /*!< CAN error state                     */

} HAL_CAN_StateTypeDef;

/**
  * @brief HAL State structures definition
  */
typedef enum
{
  HAL_FDCAN_STATE_RESET      = 0x00U, /*!< FDCAN not yet initialized or disabled */
  HAL_FDCAN_STATE_READY      = 0x01U, /*!< FDCAN initialized and ready for use   */
  HAL_FDCAN_STATE_BUSY       = 0x02U, /*!< FDCAN process is ongoing              */
  HAL_FDCAN_STATE_ERROR      = 0x03U  /*!< FDCAN error state                     */
} HAL_FDCAN_StateTypeDef;

/* Global variables and objects */
CO_t *CO = NULL; /* CANopen object */
static uint32_t process_time_point = 1000;
static uint16_t baudrate;
static uint8_t desiredNodeID;
static uint8_t activeNodeID; /* Assigned Node ID */
static volatile uint32_t can_error = 0;
static volatile HAL_CAN_StateTypeDef can_state = HAL_CAN_STATE_RESET;
static volatile HAL_FDCAN_StateTypeDef fdcan_state = HAL_FDCAN_STATE_RESET;
// set by init_lss_store before calling CAN init, will configure and load node cfg from perm storage
static syn::EepromBase* peeprom = 0;
static uint16_t lss_store_address;

/* Local CAN module object */
static CO_CANmodule_t *CANModule_local = NULL; /* Local instance of global CAN module */

CO_ReturnError_t err;
#if SYN_CAN_USE_TIMER == 0
syn::CANopenNode::CANopenTick syn::CANopenNode::_cantick;
#endif


/**
 * @brief  CAN Rx message header structure definition
 */
typedef struct
{
    uint16_t StdId; /*!< Specifies the standard identifier.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF. */

    uint8_t RTR; /*!< Specifies the type of frame for the message that will be transmitted.
                       This parameter can be a value of @ref CAN_remote_transmission_request */

    uint8_t DLC; /*!< Specifies the length of the frame that will be transmitted.
                       This parameter must be a number between Min_Data = 0 and Max_Data = 8. */
} CAN_RxHeaderTypeDef;


#ifdef STM32G431xx
#define SRAMCAN_FLS_NBR                  (28U)         /* Max. Filter List Standard Number      */
#define SRAMCAN_FLE_NBR                  ( 8U)         /* Max. Filter List Extended Number      */
#define SRAMCAN_RF0_NBR                  ( 3U)         /* RX FIFO 0 Elements Number             */
#define SRAMCAN_RF1_NBR                  ( 3U)         /* RX FIFO 1 Elements Number             */
#define SRAMCAN_TEF_NBR                  ( 3U)         /* TX Event FIFO Elements Number         */
#define SRAMCAN_TFQ_NBR                  ( 3U)         /* TX FIFO/Queue Elements Number         */

#define SRAMCAN_FLS_SIZE            ( 1U * 4U)         /* Filter Standard Element Size in bytes */
#define SRAMCAN_FLE_SIZE            ( 2U * 4U)         /* Filter Extended Element Size in bytes */
#define SRAMCAN_RF0_SIZE            (18U * 4U)         /* RX FIFO 0 Elements Size in bytes      */
#define SRAMCAN_RF1_SIZE            (18U * 4U)         /* RX FIFO 1 Elements Size in bytes      */
#define SRAMCAN_TEF_SIZE            ( 2U * 4U)         /* TX Event FIFO Elements Size in bytes  */
#define SRAMCAN_TFQ_SIZE            (18U * 4U)         /* TX FIFO/Queue Elements Size in bytes  */

#define SRAMCAN_FLSSA ((uint32_t)0)                                                      /* Filter List Standard Start
                                                                                            Address                  */
#define SRAMCAN_FLESA ((uint32_t)(SRAMCAN_FLSSA + (SRAMCAN_FLS_NBR * SRAMCAN_FLS_SIZE))) /* Filter List Extended Start
                                                                                            Address                  */
#define SRAMCAN_RF0SA ((uint32_t)(SRAMCAN_FLESA + (SRAMCAN_FLE_NBR * SRAMCAN_FLE_SIZE))) /* Rx FIFO 0 Start Address  */
#define SRAMCAN_RF1SA ((uint32_t)(SRAMCAN_RF0SA + (SRAMCAN_RF0_NBR * SRAMCAN_RF0_SIZE))) /* Rx FIFO 1 Start Address  */
#define SRAMCAN_TEFSA ((uint32_t)(SRAMCAN_RF1SA + (SRAMCAN_RF1_NBR * SRAMCAN_RF1_SIZE))) /* Tx Event FIFO Start
                                                                                            Address */
#define SRAMCAN_TFQSA ((uint32_t)(SRAMCAN_TEFSA + (SRAMCAN_TEF_NBR * SRAMCAN_TEF_SIZE))) /* Tx FIFO/Queue Start
                                                                                            Address                  */
#define SRAMCAN_SIZE  ((uint32_t)(SRAMCAN_TFQSA + (SRAMCAN_TFQ_NBR * SRAMCAN_TFQ_SIZE))) /* Message RAM size         */
#define msgRam_StandardFilterSA (SRAMCAN_BASE + SRAMCAN_FLSSA)
#define msgRam_ExtendedFilterSA (SRAMCAN_BASE + SRAMCAN_FLESA)
#define msgRam_RxFIFO0SA (SRAMCAN_BASE + SRAMCAN_RF0SA)
#define msgRam_RxFIFO1SA (SRAMCAN_BASE + SRAMCAN_RF1SA)
#define msgRam_TxEventFIFOSA (SRAMCAN_BASE + SRAMCAN_TEFSA)
#define msgRam_TxFIFOQSA (SRAMCAN_BASE + SRAMCAN_TFQSA)
//static const uint8_t DLCtoBytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};


void hal_fdcan_stop()
{
  uint32_t Counter = 0U;
  if (fdcan_state == HAL_FDCAN_STATE_BUSY)
  {
    /* Request initialisation */
    //SET_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;

    /* Wait until the INIT bit into CCCR register is set */
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0U)
    {
      /* Check for the Timeout */
      if (Counter > FDCAN_TIMEOUT_VALUE)
      {
        /* Update error code */
        can_error |= HAL_FDCAN_ERROR_TIMEOUT;

        /* Change FDCAN state */
        fdcan_state = HAL_FDCAN_STATE_ERROR;
      }

      /* Increment counter */
      Counter++;
    }

    /* Reset counter */
    Counter = 0U;

    /* Exit from Sleep mode */
    FDCAN1->CCCR &= ~FDCAN_CCCR_CSR;

    /* Wait until FDCAN exits sleep mode */
    while ((FDCAN1->CCCR & FDCAN_CCCR_CSA) == FDCAN_CCCR_CSA)
    {
      /* Check for the Timeout */
      if (Counter > FDCAN_TIMEOUT_VALUE)
      {
        /* Update error code */
        can_error |= HAL_FDCAN_ERROR_TIMEOUT;

        /* Change FDCAN state */
        fdcan_state = HAL_FDCAN_STATE_ERROR;
      }

      /* Increment counter */
      Counter++;
    }

    /* Enable configuration change */
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;

    /* Reset Latest Tx FIFO/Queue Request Buffer Index */
    //hfdcan->LatestTxFifoQRequest = 0U;

    /* Change FDCAN peripheral state */
    fdcan_state = HAL_FDCAN_STATE_READY;

  }
  else
  {
    /* Update error code */
    can_error |= HAL_FDCAN_ERROR_NOT_STARTED;
  }
}

bool hal_canfd_start()
{
  ///* Put CAN module in normal mode */
  if (fdcan_state == HAL_FDCAN_STATE_READY)
  {
    /* Change FDCAN peripheral state */
    fdcan_state = HAL_FDCAN_STATE_BUSY;

    /* Request leave initialisation */
    FDCAN1->CCCR &= ~FDCAN_CCCR_INIT;

    /* Reset the FDCAN ErrorCode */
    can_error = HAL_FDCAN_ERROR_NONE;
    return true;
  }
  else
  {
    /* Update error code */
    can_error |= HAL_FDCAN_ERROR_NOT_READY;
    return false;
  }
}

/**
  * @brief  FDCAN Rx header structure definition
  */
typedef struct
{
  uint32_t Identifier;            /*!< Specifies the identifier.
                                       This parameter must be a number between:
                                        - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
                                        - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID               */

  uint32_t IdType;                /*!< Specifies the identifier type of the received message.
                                       This parameter can be a value of @ref FDCAN_id_type               */

  uint32_t RxFrameType;           /*!< Specifies the the received message frame type.
                                       This parameter can be a value of @ref FDCAN_frame_type            */

  uint32_t DataLength;            /*!< Specifies the received frame length.
                                        This parameter can be a value of @ref FDCAN_data_length_code     */

  uint32_t ErrorStateIndicator;   /*!< Specifies the error state indicator.
                                       This parameter can be a value of @ref FDCAN_error_state_indicator */

  uint32_t BitRateSwitch;         /*!< Specifies whether the Rx frame is received with or without bit
                                       rate switching.
                                       This parameter can be a value of @ref FDCAN_bit_rate_switching    */

  uint32_t FDFormat;              /*!< Specifies whether the Rx frame is received in classic or FD
                                       format.
                                       This parameter can be a value of @ref FDCAN_format                */

  uint32_t RxTimestamp;           /*!< Specifies the timestamp counter value captured on start of frame
                                       reception.
                                       This parameter must be a number between 0 and 0xFFFF              */

  uint32_t FilterIndex;           /*!< Specifies the index of matching Rx acceptance filter element.
                                       This parameter must be a number between:
                                        - 0 and (SRAMCAN_FLS_NBR-1), if IdType is FDCAN_STANDARD_ID
                                        - 0 and (SRAMCAN_FLE_NBR-1), if IdType is FDCAN_EXTENDED_ID
                                       When the frame is a Non-Filter matching frame, this parameter
                                       is unused.                                                        */

  uint32_t IsFilterMatchingFrame; /*!< Specifies whether the accepted frame did not match any Rx filter.
                                       Acceptance of non-matching frames may be enabled via
                                       HAL_FDCAN_ConfigGlobalFilter().
                                       This parameter takes 0 if the frame matched an Rx filter or
                                       1 if it did not match any Rx filter                               */

} FDCAN_RxHeaderTypeDef;


typedef struct {
  __IO uint32_t RXF0S;   /*!< FDCAN Rx FIFO 0 Status register */
  __IO uint32_t RXF0A;   /*!< FDCAN Rx FIFO 0 Acknowledge register */
} FDCAN_RX_REG_TypeDef;


uint32_t can_read_receive_msg_ll(uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t *aData)
{
    static FDCAN_RxHeaderTypeDef rx_hdr;
    uint32_t *RxAddress;
    uint32_t GetIndex = 0;
    uint32_t ByteCounter;
    FDCAN_RX_REG_TypeDef* pfifo = (FDCAN_RX_REG_TypeDef*)(&FDCAN1->RXF0S) + RxFifo;
    /* Check that the Rx FIFO 0 is not empty */
    if ((pfifo->RXF0S & FDCAN_RXF0S_F0FL) == 0U)
    {
      /* Update error code */
      can_error |= HAL_FDCAN_ERROR_FIFO_EMPTY;

      return 1;
    }
    else
    {
      /* Check that the Rx FIFO 0 is full & overwrite mode is on */
      if (((pfifo->RXF0S & FDCAN_RXF0S_F0F) >> FDCAN_RXF0S_F0F_Pos) == 1U)
      {
        if (((FDCAN1->RXGFC & FDCAN_RXGFC_F0OM) >> FDCAN_RXGFC_F0OM_Pos) == FDCAN_RX_FIFO_OVERWRITE)
        {
          /* When overwrite status is on discard first message in FIFO */
          GetIndex = 1U;
        }
      }

      /* Calculate Rx FIFO 0 element index */
      GetIndex += ((pfifo->RXF0S & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos);

      /* Calculate Rx FIFO 0 element address */
      RxAddress = (uint32_t *)(msgRam_RxFIFO0SA + (RxFifo * SRAMCAN_RF0_SIZE) + (GetIndex * SRAMCAN_RF0_SIZE));
    }

     /* Retrieve IdType */
    rx_hdr.IdType = *RxAddress & FDCAN_ELEMENT_MASK_XTD;

    /* Retrieve Identifier */
    if (rx_hdr.IdType == FDCAN_STANDARD_ID) /* Standard ID element */
    {
      rx_hdr.Identifier = ((*RxAddress & FDCAN_ELEMENT_MASK_STDID) >> 18U);
    }
    else /* Extended ID element */
    {
      rx_hdr.Identifier = (*RxAddress & FDCAN_ELEMENT_MASK_EXTID);
    }

    /* Retrieve RxFrameType */
    rx_hdr.RxFrameType = (*RxAddress & FDCAN_ELEMENT_MASK_RTR) >> 28; // shift to right to mimic normal can

    /* Retrieve ErrorStateIndicator */
    //rx_hdr.ErrorStateIndicator = (*RxAddress & FDCAN_ELEMENT_MASK_ESI);

    /* Increment RxAddress pointer to second word of Rx FIFO element */
    RxAddress++;

    /* Retrieve RxTimestamp */
    //rx_hdr.RxTimestamp = (*RxAddress & FDCAN_ELEMENT_MASK_TS);

    /* Retrieve DataLength */
    rx_hdr.DataLength = ((*RxAddress & FDCAN_ELEMENT_MASK_DLC) >> 16U);

    /* Retrieve BitRateSwitch */
    //rx_hdr.BitRateSwitch = (*RxAddress & FDCAN_ELEMENT_MASK_BRS);

    /* Retrieve FDFormat */
    //rx_hdr.FDFormat = (*RxAddress & FDCAN_ELEMENT_MASK_FDF);

    /* Retrieve FilterIndex */
    //rx_hdr.FilterIndex = ((*RxAddress & FDCAN_ELEMENT_MASK_FIDX) >> 24U);

    /* Retrieve NonMatchingFrame */
    //rx_hdr.IsFilterMatchingFrame = ((*RxAddress & FDCAN_ELEMENT_MASK_ANMF) >> 31U);

    /* Increment RxAddress pointer to payload of Rx FIFO element */
    RxAddress++;

    /* Retrieve Rx payload */
    uint8_t *pData = (uint8_t *)RxAddress;
    for (ByteCounter = 0; ByteCounter < rx_hdr.DataLength; ByteCounter++)
    {
      aData[ByteCounter] = pData[ByteCounter];
    }
    /* Acknowledge the Rx FIFO 0 that the oldest element is read so that it increments the GetIndex */
    pfifo->RXF0A = GetIndex;

    pHeader->StdId = rx_hdr.Identifier;
    pHeader->DLC = rx_hdr.DataLength;
    pHeader->RTR = rx_hdr.RxFrameType;
    /* Return function status */
    return 0;
}

typedef struct
{
  uint32_t Identifier;          /*!< Specifies the identifier.
                                     This parameter must be a number between:
                                      - 0 and 0x7FF, if IdType is FDCAN_STANDARD_ID
                                      - 0 and 0x1FFFFFFF, if IdType is FDCAN_EXTENDED_ID               */

  uint32_t IdType;              /*!< Specifies the identifier type for the message that will be
                                     transmitted.
                                     This parameter can be a value of @ref FDCAN_id_type               */

  uint32_t TxFrameType;         /*!< Specifies the frame type of the message that will be transmitted.
                                     This parameter can be a value of @ref FDCAN_frame_type            */

  uint32_t DataLength;          /*!< Specifies the length of the frame that will be transmitted.
                                      This parameter can be a value of @ref FDCAN_data_length_code     */

  uint32_t ErrorStateIndicator; /*!< Specifies the error state indicator.
                                     This parameter can be a value of @ref FDCAN_error_state_indicator */

  uint32_t BitRateSwitch;       /*!< Specifies whether the Tx frame will be transmitted with or without
                                     bit rate switching.
                                     This parameter can be a value of @ref FDCAN_bit_rate_switching    */

  uint32_t FDFormat;            /*!< Specifies whether the Tx frame will be transmitted in classic or
                                     FD format.
                                     This parameter can be a value of @ref FDCAN_format                */

  uint32_t TxEventFifoControl;  /*!< Specifies the event FIFO control.
                                     This parameter can be a value of @ref FDCAN_EFC                   */

  uint32_t MessageMarker;       /*!< Specifies the message marker to be copied into Tx Event FIFO
                                     element for identification of Tx message status.
                                     This parameter must be a number between 0 and 0xFF                */

} FDCAN_TxHeaderTypeDef;

/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       buffer: Pointer to buffer to transmit
 */
int32_t can_send_message_ll(CO_CANtx_t *buffer)
{
    static FDCAN_TxHeaderTypeDef tx_hdr;
    int32_t PutIndex = -1;
    // get first free mailbox
    if ((FDCAN1->TXFQS & FDCAN_TXFQS_TFQF) == 0) // queue is not full
    {
        tx_hdr.Identifier = buffer->ident & CANID_MASK;
        tx_hdr.TxFrameType = (buffer->ident & FLAG_RTR) ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
        tx_hdr.IdType = FDCAN_STANDARD_ID;
        tx_hdr.FDFormat = FDCAN_CLASSIC_CAN;
        tx_hdr.BitRateSwitch = FDCAN_BRS_OFF;
        tx_hdr.MessageMarker = 0;
        tx_hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_hdr.DataLength = buffer->DLC;

        /* Now add message to FIFO. Should not fail */
        //success =
        //    HAL_FDCAN_AddMessageToTxFifoQ(((CANopenNodeSTM32*)CANmodule->CANptr)->CANHandle, &tx_hdr, buffer->data)
        //    == HAL_OK;
        PutIndex = ((FDCAN1->TXFQS & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos);
        //FDCAN_CopyMessageToRAM(hfdcan, pTxHeader, pTxData, PutIndex);
        uint32_t TxElementW1;
        uint32_t TxElementW2;
        uint32_t *TxAddress;
        uint32_t ByteCounter;

        /* Build first word of Tx header element */
        if (tx_hdr.IdType == FDCAN_STANDARD_ID)
        {
          TxElementW1 = (tx_hdr.ErrorStateIndicator |
                         FDCAN_STANDARD_ID |
                         tx_hdr.TxFrameType |
                         (tx_hdr.Identifier << 18U));
        }
        else /* pTxHeader->IdType == FDCAN_EXTENDED_ID */
        {
          TxElementW1 = (tx_hdr.ErrorStateIndicator |
                         FDCAN_EXTENDED_ID |
                         tx_hdr.TxFrameType |
                         tx_hdr.Identifier);
        }

        /* Build second word of Tx header element */
        TxElementW2 = ((tx_hdr.MessageMarker << 24U) |
                       tx_hdr.TxEventFifoControl |
                       tx_hdr.FDFormat |
                       tx_hdr.BitRateSwitch |
                       (tx_hdr.DataLength << 16U));

        /* Calculate Tx element address */
        TxAddress = (uint32_t *)(msgRam_TxFIFOQSA + (PutIndex * SRAMCAN_TFQ_SIZE));

        /* Write Tx element header to the message RAM */
        *TxAddress = TxElementW1;
        TxAddress++;
        *TxAddress = TxElementW2;
        TxAddress++;

        /* Write Tx payload to the message RAM */
        //for (ByteCounter = 0; ByteCounter < DLCtoBytes[tx_hdr.DataLength]; ByteCounter += 4U)
        //{
        //  *TxAddress = (((uint32_t)buffer->data[ByteCounter + 3U] << 24U) |
        //                ((uint32_t)buffer->data[ByteCounter + 2U] << 16U) |
        //                ((uint32_t)buffer->data[ByteCounter + 1U] << 8U)  |
        //                (uint32_t)buffer->data[ByteCounter]);
        //  TxAddress++;
        //}
        std::memcpy(TxAddress, buffer->data, 8);
        /* Activate the corresponding transmission request */
        FDCAN1->TXBAR = ((uint32_t)1 << PutIndex);
    }
    return PutIndex;
}

void can_init_ll()
{
    ///***************************************/
    ///* STM32 related configuration */
    ///***************************************/

    //hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    //hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    //hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    //hfdcan1.Init.AutoRetransmission = ENABLE;
    //hfdcan1.Init.TransmitPause = DISABLE;  <-
    //hfdcan1.Init.ProtocolException = DISABLE;
    //hfdcan1.Init.NominalPrescaler = 9;
    //hfdcan1.Init.NominalSyncJumpWidth = 1;
    //hfdcan1.Init.NominalTimeSeg1 = 27;
    //hfdcan1.Init.NominalTimeSeg2 = 4;
    //hfdcan1.Init.DataPrescaler = 1;
    //hfdcan1.Init.DataSyncJumpWidth = 1;
    //hfdcan1.Init.DataTimeSeg1 = 1;
    //hfdcan1.Init.DataTimeSeg2 = 1;
    //hfdcan1.Init.StdFiltersNbr = 0;
    //hfdcan1.Init.ExtFiltersNbr = 0;
    //hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
    /* Exit from Sleep mode */
    FDCAN1->CCCR &= ~FDCAN_CCCR_CSR;
    syn::DeadlineTimer timeout(10);
    /* Check Sleep mode acknowledge */
    timeout.reset(10);
    while ((FDCAN1->CCCR & FDCAN_CCCR_CSA) != 0U)
    {
        if (timeout.is_expired())
        {
            /* Update error code */
            can_error |= HAL_FDCAN_ERROR_TIMEOUT;

            /* Change CAN state */
            fdcan_state = HAL_FDCAN_STATE_ERROR;

            while (true)
                ;
        }
    }
    /* Request initialisation */
    FDCAN1->CCCR |= FDCAN_CCCR_INIT;
    /* Wait until the INIT bit into CCCR register is set */
    timeout.reset(10);
    while ((FDCAN1->CCCR & FDCAN_CCCR_INIT) == 0U)
    {
        if (timeout.is_expired())
        {
            /* Update error code */
            can_error |= HAL_FDCAN_ERROR_TIMEOUT;

            /* Change CAN state */
            fdcan_state = HAL_FDCAN_STATE_ERROR;

            while (true)
                ;
        }
    }
    /* Enable configuration change */
    FDCAN1->CCCR |= FDCAN_CCCR_CCE;
    ////CAN1->MCR = CAN_MCR_ABOM | CAN_MCR_NART | CAN_MCR_INRQ;
    // enable automatic retransmission so that sync tpdo works even on ARLO
    FDCAN1->CCCR &= ~FDCAN_CCCR_DAR;
    // clear transmit pause
    FDCAN1->CCCR &= ~FDCAN_CCCR_TXP;
    // disable proto exception handling
    FDCAN1->CCCR |= FDCAN_CCCR_PXHD;
    /* Set FDCAN Frame Format */
    FDCAN1->CCCR &= ~FDCAN_FRAME_FD_BRS;
    FDCAN1->CCCR |= FDCAN_FRAME_CLASSIC;
    // reset operation mode
    FDCAN1->CCCR &= ~(FDCAN_CCCR_TEST | FDCAN_CCCR_MON | FDCAN_CCCR_ASM);
    FDCAN1->TEST &= ~FDCAN_TEST_LBCK;

    //// http://www.bittiming.can-wiki.info/
    //// Type: bxCAN, Clock: 36MHz, max brp: 1024, SP: 87.5%, min tq: 8, max tq: 25, FD factor: undefined, SJW: 1
    uint32_t btr_reg;
    switch (baudrate)
    {
    //case 10:
    //{
    //    const uint32_t prescaler = 225;
    //    // const uint32_t sync_jump = 1;
    //    const uint32_t timeseq_1 = 13;
    //    const uint32_t timeseq_2 = 2;
    //    btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
    //    break;
    //}
    //case 20:
    //{
    //    const uint32_t prescaler = 100;
    //    // const uint32_t sync_jump = 1;
    //    const uint32_t timeseq_1 = 15;
    //    const uint32_t timeseq_2 = 2;
    //    btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
    //    break;
    //}
    //case 50:
    //{
    //    const uint32_t prescaler = 45;
    //    // const uint32_t sync_jump = 1;
    //    const uint32_t timeseq_1 = 13;
    //    const uint32_t timeseq_2 = 2;
    //    btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
    //    break;
    //}
    //case 100:
    //{
    //    const uint32_t prescaler = 20;
    //    // const uint32_t sync_jump = 1;
    //    const uint32_t timeseq_1 = 15;
    //    const uint32_t timeseq_2 = 2;
    //    btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
    //    break;
    //}
    //case 125:
    //{
    //    const uint32_t prescaler = 18;
    //    // const uint32_t sync_jump = 1;
    //    const uint32_t timeseq_1 = 13;
    //    const uint32_t timeseq_2 = 2;
    //    btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
    //    break;
    //}
    //case 250:
    //{
    //    const uint32_t prescaler = 9;
    //    // const uint32_t sync_jump = 1;
    //    const uint32_t timeseq_1 = 13;
    //    const uint32_t timeseq_2 = 2;
    //    btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
    //    break;
    //}
    case 500:
    {
        const uint32_t prescaler = 9;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 27;
        const uint32_t timeseq_2 = 4;
        btr_reg = ((prescaler - 1) << FDCAN_NBTP_NBRP_Pos) | ((timeseq_1 - 1) << FDCAN_NBTP_NTSEG1_Pos) | ((timeseq_2 - 1) << FDCAN_NBTP_NTSEG2_Pos);
        break;
    }
    //case 800:
    //{
    //    const uint32_t prescaler = 3;
    //    // const uint32_t sync_jump = 1;
    //    const uint32_t timeseq_1 = 12;
    //    const uint32_t timeseq_2 = 2;
    //    btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
    //    break;
    //}
    //case 1000:
    //{
    //    const uint32_t prescaler = 2;
    //    // const uint32_t sync_jump = 1;
    //    const uint32_t timeseq_1 = 15;
    //    const uint32_t timeseq_2 = 2;
    //    btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
    //    break;
    //}
    default:
        while (true)
        {
            ;
        }
    }
    // set framerate register
    FDCAN1->DBTP = 0;
    FDCAN1->NBTP = btr_reg;
    /* Select between Tx FIFO and Tx Queue operation modes */
    FDCAN1->TXBC |= FDCAN_TX_QUEUE_OPERATION;

    /* Flush the allocated Message RAM area */
    uint32_t *pram = (uint32_t*)SRAMCAN_BASE;
    uint32_t *pramend = (uint32_t*)(SRAMCAN_BASE + SRAMCAN_SIZE);
    while(pram < pramend)
    {
      *pram++ = 0;
    }
    // setup filter list
    FDCAN1->RXGFC = ((FDCAN_ACCEPT_IN_RX_FIFO0 << FDCAN_RXGFC_ANFS_Pos) |
                     (FDCAN_REJECT << FDCAN_RXGFC_ANFE_Pos)  |
                     (FDCAN_FILTER_REMOTE << FDCAN_RXGFC_RRFS_Pos) |
                     (FDCAN_FILTER_REMOTE << FDCAN_RXGFC_RRFE_Pos));
    ///* Initialize the error code */
    can_error = HAL_FDCAN_ERROR_NONE;

    ///* Initialize the CAN state */
    fdcan_state = HAL_FDCAN_STATE_READY;

    // enable all tx buffer tx complete irqs
    FDCAN1->TXBTIE = 0x7;
    // enable both interrupt lines
    FDCAN1->ILE = 0x3;
    // Assign interrupts to line0 and line1
    FDCAN1->ILS = FDCAN_ILS_PERR | FDCAN_ILS_BERR | FDCAN_ILS_MISC;
    // enable individual interrupts
    FDCAN1->IE = (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO1_NEW_MESSAGE
                  | FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF
                  | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR
                  | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING);
}

void can_init_hardware_ll()
{
    // set clock for fdcan pclk1
    RCC->CCIPR |= (0x2 << 24);
    // enable clock for hw
    RCC->APB1ENR1 |= RCC_APB1ENR1_FDCANEN;
    // initialize GPIO
#if !defined(SYN_CAN_1_REMAP) || SYN_CAN_1_REMAP == 0
    {
        syn::Gpio rx('a', 11);
        rx.mode(syn::Gpio::out_alt_push_pull, syn::Gpio::MHz_10, syn::Gpio::Alternate::FDCAN);
    }
    {
        syn::Gpio tx('a', 12);
        tx.mode(syn::Gpio::out_alt_push_pull, syn::Gpio::MHz_10, syn::Gpio::Alternate::FDCAN);
    }
#elif SYN_CAN_1_REMAP == 1
    Gpio::remap(Gpio::can_rx_pb8_tx_pb9);
    {
        Gpio rx('b', 8);
        rx.mode(Gpio::in_pullup, Gpio::Input);
    }
    {
        Gpio tx('b', 9);
        tx.mode(Gpio::out_alt_push_pull, Gpio::MHz_50, Gpio::Alternate::CAN);
    }
#endif
    // enable CAN interrupts
    // set priority to 8 if using OS functions inside ISR with Core::enter_isr()
    syn::Core::enable_isr(FDCAN1_IT0_IRQn, 5);
    syn::Core::enable_isr(FDCAN1_IT1_IRQn, 5);
    // we only use fifo0 (I think -- only the first filter allows any message for FIFO 0)
    // enable anyway just to avoid any error in this regard, should never fire anway
    //Core::enable_isr(CAN1_RX1_IRQn, 5);
    // Core::enable_isr(CAN1_SCE_IRQn, 5);
    // CAN1->IER |= CAN_IER_TMEIE;
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */
// static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    uint32_t err = 0;

    err = FDCAN1->PSR & (FDCAN_PSR_BO | FDCAN_PSR_EW | FDCAN_PSR_EP);

    if (CANmodule->errOld != err) {

        uint16_t status = CANmodule->CANerrorStatus;

        CANmodule->errOld = err;

        if (err & FDCAN_PSR_BO) {
            status |= CO_CAN_ERRTX_BUS_OFF;
            // In this driver we expect that the controller is automatically handling the protocol exceptions.

        } else {
            /* recalculate CANerrorStatus, first clear some flags */
            status &= 0xFFFF
                      ^ (CO_CAN_ERRTX_BUS_OFF | CO_CAN_ERRRX_WARNING | CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_WARNING
                         | CO_CAN_ERRTX_PASSIVE);

            if (err & FDCAN_PSR_EW) {
                status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
            }

            if (err & FDCAN_PSR_EP) {
                status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
            }
        }

        CANmodule->CANerrorStatus = status;
    }
}
#else


/******************************************************************************/
void CO_CANsetConfigurationMode(void *CANptr)
{
    (void)CANptr;
    /* Put CAN module in configuration mode */
    // HAL_CAN_Stop(&hcan);

    if (can_state == HAL_CAN_STATE_LISTENING)
    {
        /* Request initialisation */
        // SET_BIT(hcan->Instance->MCR, CAN_MCR_INRQ);
        CAN1->MCR |= CAN_MCR_INRQ;

        syn::DeadlineTimer timeout(10);

        /* Wait the acknowledge */
        while ((CAN1->MSR & CAN_MSR_INAK) == 0U)
        {
            /* Check for the Timeout */
            if (timeout.is_expired())
            {
                /* Update error code */
                can_error |= HAL_CAN_ERROR_TIMEOUT;

                /* Change CAN state */
                can_state = HAL_CAN_STATE_ERROR;

                while (true)
                    ;
            }
        }

        /* Exit from sleep mode */
        // CLEAR_BIT(hcan->Instance->MCR, CAN_MCR_SLEEP);
        CAN1->MCR &= ~CAN_MCR_SLEEP;

        /* Change CAN peripheral state */
        can_state = HAL_CAN_STATE_READY;
    }
    else
    {
        /* Update error code */
        can_error |= HAL_CAN_ERROR_NOT_STARTED;
    }
}

/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    /* Put CAN module in normal mode */
    if (can_state == HAL_CAN_STATE_READY)
    {
        /* Change CAN peripheral state */
        can_state = HAL_CAN_STATE_LISTENING;

        /* Request leave initialisation */
        CAN1->MCR &= ~CAN_MCR_INRQ;

        /* Get tick */
        syn::DeadlineTimer timeout(10);

        /* Wait the acknowledge */
        while ((CAN1->MSR & CAN_MSR_INAK) != 0U)
        {
            /* Check for the Timeout */
            if (timeout.is_expired())
            {
                /* Update error code */
                can_error |= HAL_CAN_ERROR_TIMEOUT;

                /* Change CAN state */
                can_state = HAL_CAN_STATE_ERROR;

                while (true)
                    ;
            }
        }

        /* Reset the CAN ErrorCode */
        can_error = HAL_CAN_ERROR_NONE;
        CANmodule->CANnormal = true;
    }
    else
    {
        /* Update error code */
        can_error |= HAL_CAN_ERROR_NOT_READY;
    }
}



uint32_t can_read_receive_msg_ll(uint32_t RxFifo, CAN_RxHeaderTypeDef *pHeader, uint8_t *aData)
{
    auto pfifo = &CAN1->sFIFOMailBox[RxFifo];

    uint32_t rir = pfifo->RIR;
    pHeader->StdId = (CAN_RI0R_STID & rir) >> CAN_TI0R_STID_Pos;
    pHeader->RTR = (CAN_RI0R_RTR & rir);
    pHeader->DLC = (CAN_RDT0R_DLC & pfifo->RDTR);

    /* Get the data */
    uint32_t data[2] = {pfifo->RDLR, pfifo->RDHR};
    // use memcpy to avoid alignment issues of 8bit buffer
    std::memcpy(aData, data, 8);

    /* Release the FIFO */
    if (RxFifo == 0) /* Rx element is assigned to Rx FIFO 0 */
    {
        /* Release RX FIFO 0 */
        // SET_BIT(hcan->Instance->RF0R, CAN_RF0R_RFOM0);
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
    else /* Rx element is assigned to Rx FIFO 1 */
    {
        /* Release RX FIFO 1 */
        // SET_BIT(hcan->Instance->RF1R, CAN_RF1R_RFOM1);
        CAN1->RF1R |= CAN_RF1R_RFOM1;
    }

    /* Return function status */
    return 0;
}

/**
 * \brief           Send CAN message to network
 * This function must be called with atomic access.
 *
 * \param[in]       buffer: Pointer to buffer to transmit
 */
int32_t can_send_message_ll(CO_CANtx_t *buffer)
{
    int32_t mailbox = -1;

    // get first free mailbox
    uint32_t tsr = CAN1->TSR;
    if (tsr & CAN_TSR_TME) // if at least one bit is set, that mailbox is free
    {
        mailbox = (tsr & CAN_TSR_CODE) >> 24;
        /*
         * RTR flag is part of identifier value
         * hence it needs to be properly decoded
         */
        uint32_t stdid = buffer->ident & CANID_MASK;
        uint32_t rtr = (buffer->ident & FLAG_RTR) ? CAN_RTR_REMOTE : CAN_RTR_DATA;

        /* Now add message to FIFO. Should not fail */
        CAN_TxMailBox_TypeDef *pbox = &CAN1->sTxMailBox[mailbox];
        pbox->TIR = ((stdid << CAN_TI0R_STID_Pos) | rtr);
        pbox->TDTR = buffer->DLC;
        // little endian machine the buffer has the same order as the registers
        uint32_t data[2];
        // use memcpy to avoid alignment issues of 8bit buffer
        std::memcpy(data, buffer->data, 8);
        pbox->TDLR = data[0];
        pbox->TDHR = data[1];
        // the code below might suffer from alignment issues!!
        // pbox->TDLR = *((uint32_t *)(&buffer->data[0]));
        // pbox->TDHR = *((uint32_t *)(&buffer->data[4]));
        // request transmit mailbox
        pbox->TIR |= CAN_TI0R_TXRQ;
    }
    return mailbox;
}

void can_init_ll()
{
    /***************************************/
    /* STM32 related configuration */
    /***************************************/
    // http://www.bittiming.can-wiki.info/
    // Type: bxCAN, Clock: 36MHz, max brp: 1024, SP: 87.5%, min tq: 8, max tq: 25, FD factor: undefined, SJW: 1
    uint32_t btr_reg;
    switch (baudrate)
    {
    case 10:
    {
        const uint32_t prescaler = 225;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 13;
        const uint32_t timeseq_2 = 2;
        btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
        break;
    }
    case 20:
    {
        const uint32_t prescaler = 100;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 15;
        const uint32_t timeseq_2 = 2;
        btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
        break;
    }
    case 50:
    {
        const uint32_t prescaler = 45;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 13;
        const uint32_t timeseq_2 = 2;
        btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
        break;
    }
    case 100:
    {
        const uint32_t prescaler = 20;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 15;
        const uint32_t timeseq_2 = 2;
        btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
        break;
    }
    case 125:
    {
        const uint32_t prescaler = 18;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 13;
        const uint32_t timeseq_2 = 2;
        btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
        break;
    }
    case 250:
    {
        const uint32_t prescaler = 9;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 13;
        const uint32_t timeseq_2 = 2;
        btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
        break;
    }
    case 500:
    {
        const uint32_t prescaler = 4;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 15;
        const uint32_t timeseq_2 = 2;
        btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
        break;
    }
    case 800:
    {
        const uint32_t prescaler = 3;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 12;
        const uint32_t timeseq_2 = 2;
        btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
        break;
    }
    case 1000:
    {
        const uint32_t prescaler = 2;
        // const uint32_t sync_jump = 1;
        const uint32_t timeseq_1 = 15;
        const uint32_t timeseq_2 = 2;
        btr_reg = ((timeseq_2 - 1) << 20) | ((timeseq_1 - 1) << 16) | (prescaler - 1);
        break;
    }
    default:
        while (true)
        {
            ;
        }
    }

    /* Request initialisation */
    CAN1->MCR |= CAN_MCR_INRQ;
    /* Wait initialisation acknowledge */
    syn::DeadlineTimer timeout(10);
    while ((CAN1->MSR & CAN_MSR_INAK) == 0U)
    {
        if (timeout.is_expired())
        {
            /* Update error code */
            can_error |= HAL_CAN_ERROR_TIMEOUT;

            /* Change CAN state */
            can_state = HAL_CAN_STATE_ERROR;

            while (true)
                ;
        }
    }
    /* Exit from sleep mode */
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    /* Check Sleep mode leave acknowledge */
    timeout.reset(10);
    while ((CAN1->MSR & CAN_MSR_SLAK) != 0U)
    {
        if (timeout.is_expired())
        {
            /* Update error code */
            can_error |= HAL_CAN_ERROR_TIMEOUT;

            /* Change CAN state */
            can_state = HAL_CAN_STATE_ERROR;

            while (true)
                ;
        }
    }
    //CAN1->MCR = CAN_MCR_ABOM | CAN_MCR_NART | CAN_MCR_INRQ;
    // enable automatic retransmission so that sync tpdo works even on ARLO
    CAN1->MCR = CAN_MCR_ABOM | CAN_MCR_INRQ;
    CAN1->BTR = btr_reg;

    /* Initialize the error code */
    can_error = HAL_CAN_ERROR_NONE;

    /* Initialize the CAN state */
    can_state = HAL_CAN_STATE_READY;

    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FMR = (28 << 8) | CAN_FMR_FINIT;
    // set filters to idmask mode
    CAN1->FM1R = 0;
    CAN1->FS1R = 3;
    // assign to FIFO 0 and 1;
    CAN1->FFA1R = 2;
    CAN1->FA1R = 3; // activate both filters
    // set filter to zero, aka dont care in mask mode
    CAN1->sFilterRegister[0].FR1 = 0;
    CAN1->sFilterRegister[0].FR2 = 0;
    CAN1->sFilterRegister[1].FR1 = 0;
    CAN1->sFilterRegister[1].FR2 = 0;
    CAN1->FMR = (28 << 8); // clear ini bit to activate filter

    /* Enable notifications */
    /* Activate the CAN notification interrupts */
    CAN1->IER = CAN_IER_FMPIE0 | CAN_IER_FMPIE1 | CAN_IER_TMEIE;
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
#if SYN_CAN_USE_TIMER != 0
    if (SYN_CAN_USE_TIMER == 4)
    {
        TIM4->CR1 = 0;
    }
    else
    {
        OS_ASSERT(SYN_CAN_USE_TIMER == 0, ERR_BAD_INDEX);
    }
#else
    //syn::CANopenNode::_cantick.suspend();
#endif
    (void)CANmodule;
    if (can_state == HAL_CAN_STATE_LISTENING)
    {
        /* Request initialisation */
        CAN1->MCR |= CAN_MCR_INRQ;

        /* Get tick */
        syn::DeadlineTimer timeout(10);

        /* Wait the acknowledge */
        while ((CAN1->MSR & CAN_MSR_INAK) == 0U)
        {
            /* Check for the Timeout */
            if (timeout.is_expired())
            {
                /* Update error code */
                can_error |= HAL_CAN_ERROR_TIMEOUT;

                /* Change CAN state */
                can_state = HAL_CAN_STATE_ERROR;

                return;
            }
        }

        /* Exit from sleep mode */
        CAN1->MCR &= ~CAN_MCR_SLEEP;

        /* Change CAN peripheral state */
        can_state = HAL_CAN_STATE_READY;
    }
    else
    {
        /* Update error code */
        can_error |= HAL_CAN_ERROR_NOT_STARTED;
    }
}

void can_init_hardware_ll()
{
    // enable clock for hw
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
    // initialize GPIO
#if !defined(SYN_CAN_1_REMAP) || SYN_CAN_1_REMAP == 0
    {
        Gpio rx('a', 11);
        rx.mode(Gpio::in_pullup, Gpio::Input);
    }
    {
        Gpio tx('a', 12);
        tx.mode(Gpio::out_alt_push_pull, Gpio::MHz_50, Gpio::Alternate::CAN);
    }
#elif SYN_CAN_1_REMAP == 1
    Gpio::remap(Gpio::can_rx_pb8_tx_pb9);
    {
        Gpio rx('b', 8);
        rx.mode(Gpio::in_pullup, Gpio::Input);
    }
    {
        Gpio tx('b', 9);
        tx.mode(Gpio::out_alt_push_pull, Gpio::MHz_50, Gpio::Alternate::CAN);
    }
#endif
    // enable CAN interrupts
    // set priority to 8 if using OS functions inside ISR with Core::enter_isr()
    Core::enable_isr(USB_HP_CAN1_TX_IRQn, 5);
    Core::enable_isr(USB_LP_CAN1_RX0_IRQn, 5);
    // we only use fifo0 (I think -- only the first filter allows any message for FIFO 0)
    // enable anyway just to avoid any error in this regard, should never fire anway
    Core::enable_isr(CAN1_RX1_IRQn, 5);
    // Core::enable_isr(CAN1_SCE_IRQn, 5);
    // CAN1->IER |= CAN_IER_TMEIE;
}

/******************************************************************************/
/* Get error counters from the module. If necessary, function may use
 * different way to determine errors. */
// static uint16_t rxErrors = 0, txErrors = 0, overflow = 0;

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    uint32_t err = 0;

    // CANOpen just care about Bus_off, Warning, Passive and Overflow
    // I didn't find overflow error register in STM32, if you find it please let me know

    err = CAN1->ESR & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_EWGF);

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
#endif

/*##############################################################################*/
/*##############################################################################*/
/*##############################################################################*/

void CO_CANsetConfigurationMode(void *CANptr)
{
  (void)CANptr;
  hal_fdcan_stop();
}

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
  (void)CANmodule;
  CANmodule->CANnormal = hal_canfd_start();
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
  if(CANmodule != NULL)
    hal_fdcan_stop();
}

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule, void *CANptr, CO_CANrx_t rxArray[], uint16_t rxSize, CO_CANtx_t txArray[],
                                   uint16_t txSize, uint16_t CANbitRate)
{
    (void)CANbitRate;
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
    //CANmodule->firstCANtxMessage = false;
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

    can_init_ll();
    
    return CO_ERROR_NO;
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

/******************************************************************************/
CO_ReturnError_t
CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if (buffer->bufferFull)
    {
        //if (!CANmodule->firstCANtxMessage)
        //{
            /* don't set error, if bootup message is still on buffers */
            CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
        //}
        err = CO_ERROR_TX_OVERFLOW;
    }

    /*
     * Send message to CAN network
     *
     * Lock interrupts for atomic operation
     */
    CO_LOCK_CAN_SEND(CANmodule);
    if (can_send_message_ll(buffer) >= 0)
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


/**
 * \brief           Read message from RX FIFO
 * \param[in]       fifo: Fifo number to use for read
 */
void can_read_received_msg(uint32_t fifo)
{
    static CO_CANrxMsg_t rcvMsg;
    CAN_RxHeaderTypeDef rx_hdr;
    /* Read received message from FIFO */
    if (can_read_receive_msg_ll(fifo, &rx_hdr, rcvMsg.data) != 0)
    {
        return;
    }
    /* Setup identifier (with RTR) and length */
    rcvMsg.ident = rx_hdr.StdId | (rx_hdr.RTR == CAN_RTR_REMOTE ? FLAG_RTR : 0x00);
    rcvMsg.dlc = rx_hdr.DLC;
    uint32_t rcvMsgIdent = rcvMsg.ident; /* identifier of the received message */

    /*
     * Hardware filters are not used for the moment
     * TODO Implement hardware filters...
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
        CO_CANrx_t *buffer = CANModule_local->rxArray;
        for (auto i = CANModule_local->rxSize; i > 0U; --i, ++buffer)
        {
            if (((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U)
            {
                /* Call specific function, which will process the message */
                if (buffer->CANrx_callback != NULL)
                {
                    buffer->CANrx_callback(buffer->object, (void *)&rcvMsg);
                }
                break;
            }
        }
    }
}

/**
 * \brief           TX buffer has been well transmitted callback
 */
void can_interrupt_TX()
{
    //CANModule_local->firstCANtxMessage = false; /* First CAN message (bootup) was sent successfully */
    CANModule_local->bufferInhibitFlag = false; /* Clear flag from previous message */
    if (CANModule_local->CANtxCount > 0U)
    {                                                      /* Are there any new messages waiting to be send */
        CO_CANtx_t *buffer = &CANModule_local->txArray[0]; /* Start with first buffer handle */
        uint16_t i;

        /*
         * Try to send more buffers, process all empty ones
         *
         * This function is always called from interrupt,
         * however to make sure no preemption can happen, interrupts are anyway locked
         * (unless you can guarantee no higher priority interrupt will try to access to CAN instance and send data,
         *  then no need to lock interrupts..)
         */
        //CO_LOCK_CAN_SEND(CANModule_local);
        for (i = CANModule_local->txSize; i > 0U; --i, ++buffer)
        {
            /* Try to send message */
            if (buffer->bufferFull)
            {
                if (can_send_message_ll(buffer) >= 0)
                {
                    buffer->bufferFull = false;
                    CANModule_local->CANtxCount -= 1;
                    CANModule_local->bufferInhibitFlag = buffer->syncFlag;
                }
            }
        }
        /* Clear counter if no more messages */
        if (i == 0U)
        {
            CANModule_local->CANtxCount = 0U;
        }
        //CO_UNLOCK_CAN_SEND(CANModule_local);
    }
}

using namespace syn;

typedef bool_t (*pFunctLSScfgStore)(void *object, uint8_t id, uint16_t bitRate);

void CANopenNode::init_lss_store(EepromBase* eeprom, uint16_t address)
{
    peeprom = eeprom;
    lss_store_address = address;
}

bool_t _store_lss_eeprom(void* pnull, uint8_t id, uint16_t bitrate)
{
    uint32_t data = (bitrate << 8) | id;
    return peeprom->write(lss_store_address, data);
}

int32_t CANopenNode::init(uint8_t desired_id, uint16_t baudrate_k)
{
    uint32_t data;
    if(peeprom != 0 && peeprom->read(lss_store_address, data))
    {
        desiredNodeID = uint8_t(data & 0xFF);
        baudrate = data >> 8;
        printf("Load id %d baudrate %d\n", desiredNodeID, baudrate);
    }
    else
    {
        if(peeprom != 0)
          printf("Err loading id & baud\n");
        desiredNodeID = desired_id;
        baudrate = baudrate_k;
        printf("Start id %d baudrate %d\n", desiredNodeID, baudrate);
    }

    can_init_hardware_ll();

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
        log_printf("Error: Can't allocate memory\n");
        return 1;
    }
    else
    {
        log_printf("Allocated %u bytes for CANopen objects\n", heapMemoryUsed);
    }
#endif
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
    err = CO_storageBlank_init(&storage, CANModule_local, OD_ENTRY_H1010_storeParameters,
                               OD_ENTRY_H1011_restoreDefaultParameters, storageEntries, storageEntriesCount,
                               &storageInitError);

    if (err != CO_ERROR_NO && err != CO_ERROR_DATA_CORRUPT)
    {
        log_printf("Error: Storage %d\n", storageInitError);
        return 2;
    }
#endif
    // Setup 1ms process timer
#if SYN_CAN_USE_TIMER != 0
    TIM_TypeDef *pTimer = NULL;
    uint32_t irqn;
    if (SYN_CAN_USE_TIMER == 4)
    {
        pTimer = TIM4;
        irqn = TIM4_IRQn;
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    }
    else
    {
        OS_ASSERT(SYN_CAN_USE_TIMER == 0, ERR_BAD_INDEX);
    }
    pTimer->CR1 = 0;
    pTimer->DIER = TIM_DIER_UIE;
    pTimer->SR = 0;
    pTimer->PSC = SystemCoreClock / 1000000 - 1;
    pTimer->ARR = 1000;
    Core::enable_isr(static_cast<IRQn_Type>(irqn), 6);
#else
    syn::CANopenNode::_cantick.start();
#endif
    return reset_com();
}

void CANopenNode::process(syn::Gpio &led_green, syn::Gpio &led_red)
{
    /* loop for normal program execution ******************************************/
    /* get time difference since last function call */
    uint32_t time_current = System::microseconds();
    uint32_t time_delta = process_time_point - time_current;

    if (static_cast<int32_t>(time_delta) <= 0)
    { // Make sure more than 1ms elapsed
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
            log_printf("CANopenNode Reset Communication request\n");
            reset_com(); // Reset Communication routine
        }
        else if (reset_status == CO_RESET_APP)
        {
            log_printf("CANopenNode Device Reset\n");
            __NVIC_SystemReset(); // Reset the STM32 Microcontroller
        }
    }
}

void CANopenNode::set_error(bool state, uint8_t err_bits, CO_EM_errorCode_t err_code, uint32_t info)
{
    if (!CO->nodeIdUnconfigured)
      CO_error(CO->em, state, (CO_EM_errorStatusBits_t)err_bits, err_code, info);
}

int32_t CANopenNode::reset_com()
{
    /* CANopen communication reset - initialize CANopen objects *******************/
    log_printf("CANopenNode - Reset communication...\n");

    /* Wait rt_thread. */
    CO->CANmodule->CANnormal = false;

    /* Enter CAN configuration. */
    CO_CANsetConfigurationMode(NULL);
    CO_CANmodule_disable(CO->CANmodule);

    /* initialize CANopen */
    err = CO_CANinit(CO, 0, 0); // Bitrate for STM32 microcontroller is being set in MXCube Settings
    if (err != CO_ERROR_NO)
    {
        log_printf("Error: CAN initialization failed: %d\n", err);
        return 1;
    }

    // get chip unique ID to populate device serial number
    OD_PERSIST_COMM.x1018_identity.serialNumber = *(syn::System::uniqueID() + 2);

    CO_LSS_address_t lssAddress = {.identity = {.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
                                                .productCode = OD_PERSIST_COMM.x1018_identity.productCode,
                                                .revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
                                                .serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber}};
    err = CO_LSSinit(CO, &lssAddress, &desiredNodeID, &baudrate);
    if (err != CO_ERROR_NO)
    {
        log_printf("Error: LSS slave initialization failed: %d\n", err);
        return 2;
    }
    if(peeprom != 0)
    {
      CO_LSSslave_initCfgStoreCallback(CO->LSSslave, 0, _store_lss_eeprom);
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
            log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
        }
        else
        {
            log_printf("Error: CANopen initialization failed: %d\n", err);
        }
        return 3;
    }

    /* Restart Timer interrupt function for execution every 1 millisecond */
#if SYN_CAN_USE_TIMER != 0
    if (SYN_CAN_USE_TIMER == 4)
    {
        TIM4->CNT = 0;
        TIM4->CR1 = TIM_CR1_CEN;
    }
    else
    {
        OS_ASSERT(SYN_CAN_USE_TIMER == 0, ERR_BAD_INDEX);
    }
#endif
    /* Configure CANopen callbacks, etc */
    if (!CO->nodeIdUnconfigured)
    {
      err = CO_CANopenInitPDO(CO, CO->em, OD, activeNodeID, &errInfo);
      if (err != CO_ERROR_NO)
      {
          if (err == CO_ERROR_OD_PARAMETERS)
          {
              log_printf("Error: Object Dictionary entry 0x%X\n", errInfo);
          }
          else
          {
              log_printf("Error: PDO initialization failed: %d\n", err);
          }
          return 4;
      }
#if (CO_CONFIG_STORAGE) & CO_CONFIG_STORAGE_ENABLE
        if (storageInitError != 0)
        {
            CO_errorReport(CO->em, CO_EM_NON_VOLATILE_MEMORY, CO_EMC_HARDWARE, storageInitError);
        }
#endif
    }
    else
    {
        log_printf("CANopenNode - Node-id not initialized\n");
    }

    /* start CAN */
    CO_CANsetNormalMode(CANModule_local);

    log_printf("CANopenNode - Running...\n");
    fflush(stdout);
    process_time_point = System::microseconds() + 1000;
    return err;
}

void CO_process_1ms()
{
    //CO_LOCK_OD(CANModule_local);
    if (!CO->nodeIdUnconfigured && CANModule_local->CANnormal)
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
    //CO_UNLOCK_OD(CANModule_local);
}

#if SYN_CAN_USE_TIMER == 0
void syn::CANopenNode::CANopenTick::run()
{
    syn::Rate<1000> rate;
    rate.init();
    while (1)
    {
        CO_process_1ms();
        rate.sleep();
    }
}
#endif

uint8_t *CANopenNode::getFlagsPDO(uint16_t canopen_index)
{
    OD_entry_t *ent = OD_find(OD, canopen_index);
    if (ent == NULL)
        return NULL;
    return OD_getFlagsPDO(ent);
}

void CANopenNode::requestTPDO(uint8_t *flagsPDO, uint8_t subidx)
{
    OD_requestTPDO(flagsPDO, subidx);
}

//void CANopenNode::TPDOtrigger::init(OD_entry_t *pObject)
//{
//    _ext.object = NULL;
//    _ext.read = OD_readOriginal;
//    _ext.write = OD_writeOriginal;
//    OD_extension_init(pObject, &_ext);
//    _flags = OD_getFlagsPDO(pObject);
//}

//void CANopenNode::TPDOtrigger::trigger()
//{
//    OD_requestTPDO(_flags, 1);
//}

extern "C"
{
#ifdef STM32G431xx
    // use IRQ 0 for TX and RX events
    void FDCAN1_IT0_IRQHandler(void)
    {
      uint32_t irqs = FDCAN1->IR;
      if(irqs & FDCAN_RX_FIFO0_MASK)
      {
        FDCAN1->IR = FDCAN_RX_FIFO0_MASK;
        can_read_received_msg(0);
      }
      if(irqs & FDCAN_RX_FIFO1_MASK)
      {
        FDCAN1->IR = FDCAN_RX_FIFO1_MASK;
        can_read_received_msg(1);
      }
      if(irqs & FDCAN_IR_TC)
      {
        FDCAN1->IR = FDCAN_IR_TC;
        can_interrupt_TX();
      }
    }

    // use IRQ 1 for errors
    void FDCAN1_IT1_IRQHandler(void)
    {
        uint32_t errorcode = HAL_CAN_ERROR_NONE;
        uint32_t irqs = FDCAN1->IR;

        int counter = 0;
        while(irqs)
        {
          ++counter;
        }

        /* Call the Error call Back in case of Errors */
        if (errorcode != HAL_CAN_ERROR_NONE)
        {
            /* Update error code in handle */
            can_error |= errorcode;
            /* Call weak (surcharged) callback */
            // HAL_CAN_ErrorCallback(hcan);
        }
    }

#else
    void USB_HP_CAN1_TX_IRQHandler(void)
    {
        // syn::Core::enter_isr();
        uint32_t errorcode = HAL_CAN_ERROR_NONE;
        uint32_t tsrflags = CAN1->TSR;
        /* Transmit Mailbox 0 management *****************************************/
        if ((tsrflags & CAN_TSR_RQCP0) != 0U)
        {
            /* Clear the Transmission Complete flag (and TXOK0,ALST0,TERR0 bits) */
            CAN1->TSR |= CAN_TSR_RQCP0;

            if ((tsrflags & CAN_TSR_TXOK0) != 0U)
            {
                can_interrupt_TX();
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
                    // HAL_CAN_TxMailbox0AbortCallback(hcan);
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
                can_interrupt_TX();
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
                    // HAL_CAN_TxMailbox1AbortCallback(hcan);
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
                can_interrupt_TX();
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
                    // HAL_CAN_TxMailbox2AbortCallback(hcan);
                }
            }
        }
        /* Call the Error call Back in case of Errors */
        if (errorcode != HAL_CAN_ERROR_NONE)
        {
            /* Update error code in handle */
            can_error |= errorcode;
            /* Call weak (surcharged) callback */
            // HAL_CAN_ErrorCallback(hcan);
        }
        // syn::Core::leave_isr();
    }

    void USB_LP_CAN1_RX0_IRQHandler(void)
    {
        // syn::Core::enter_isr();
        //  uint32_t errorcode = HAL_CAN_ERROR_NONE;
        //  uint32_t rf0rflags = CAN1->RF0R;
        //  /* Receive FIFO 0 overrun interrupt management *****************************/
        //  if ((rf0rflags & CAN_RF0R_FOVR0) != 0U)
        //  {
        //      /* Set CAN error code to Rx Fifo 0 overrun error */
        //      errorcode |= HAL_CAN_ERROR_RX_FOV0;
        //      /* Clear FIFO0 Overrun Flag */
        //      CAN1->RF0R = CAN_RF0R_FOVR0;
        //  }
        //  /* Receive FIFO 0 full interrupt management ********************************/
        //  if ((rf0rflags & CAN_RF0R_FULL0) != 0U)
        //  {
        //      /* Clear FIFO 0 full Flag */
        //      CAN1->RF0R = CAN_RF0R_FULL0;
        //      /* Call weak (surcharged) callback */
        //      // HAL_CAN_RxFifo0FullCallback(hcan);
        //  }
        /* Receive FIFO 0 message pending interrupt management *********************/
        /* Check if message is still pending */
        if ((CAN1->RF0R & CAN_RF0R_FMP0) != 0U)
        {
            /* Receive FIFO 0 message pending Callback */
            can_read_received_msg(0);
        }
        // /* Call the Error call Back in case of Errors */
        // if (errorcode != HAL_CAN_ERROR_NONE)
        // {
        //     /* Update error code in handle */
        //     hcan.ErrorCode |= errorcode;
        //     /* Call weak (surcharged) callback */
        //     // HAL_CAN_ErrorCallback(hcan);
        // }
        // syn::Core::leave_isr();
    }

    void CAN1_RX1_IRQHandler(void)
    {
        // syn::Core::enter_isr();
        //  uint32_t errorcode = HAL_CAN_ERROR_NONE;
        //  uint32_t rf1rflags = CAN1->RF1R;
        //  /* Receive FIFO 1 overrun interrupt management *****************************/
        //  if ((rf1rflags & CAN_RF1R_FOVR1) != 0U)
        //  {
        //      /* Set CAN error code to Rx Fifo 1 overrun error */
        //      errorcode |= HAL_CAN_ERROR_RX_FOV1;
        //      /* Clear FIFO1 Overrun Flag */
        //      CAN1->RF0R = CAN_RF1R_FOVR1;
        //  }
        //  /* Receive FIFO 1 full interrupt management ********************************/
        //  if ((rf1rflags & CAN_RF1R_FULL1) != 0U)
        //  {
        //      /* Clear FIFO 1 full Flag */
        //      CAN1->RF0R = CAN_RF1R_FULL1;
        //      /* Call weak (surcharged) callback */
        //      // HAL_CAN_RxFifo1FullCallback(hcan);
        //  }
        /* Receive FIFO 1 message pending interrupt management *********************/
        /* Check if message is still pending */
        if ((CAN1->RF1R & CAN_RF1R_FMP1) != 0U)
        {
            /* Receive FIFO 1 message pending Callback */
            can_read_received_msg(1);
        }
        /* Call the Error call Back in case of Errors */
        // if (errorcode != HAL_CAN_ERROR_NONE)
        // {
        //     /* Update error code in handle */
        //     hcan.ErrorCode |= errorcode;
        //     /* Call weak (surcharged) callback */
        //     // HAL_CAN_ErrorCallback(hcan);
        // }
        // syn::Core::leave_isr();
    }

    // void CAN1_SCE_IRQHandler(void)
    // {
    //     HAL_CAN_IRQHandler(&hcan);
    // }
#endif
#if SYN_CAN_USE_TIMER != 0
#if SYN_CAN_USE_TIMER == 4
    void TIM4_IRQHandler()
    {
        TIM4->SR = 0;
        CO_process_1ms();
    }
#endif
#endif
}
