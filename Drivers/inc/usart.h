/**
 * @file    usart.h
 * @brief   Universal Synchronous/Asynchronous Receiver Transmitter (USART) driver for STM32F407
 * @author  Hasan Erol
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "stm32f407xx.h"
#include "rcc.h"
#include "systick.h"

#ifdef __cplusplus
extern "C" {
#endif


/* ========================================================================== */
/*                             Type Definitions                               */
/* ========================================================================== */

/* -------------------------------------------------------------------------- */
/* Configuration Enumerations (User-level settings)                           */
/* -------------------------------------------------------------------------- */

/**
 * @brief Parity selection
 */
typedef enum {
    USART_PARITY_NONE = 0x0U,  /*!< No parity bit                          */
    USART_PARITY_EVEN = 0x1U,  /*!< Even parity                            */
    USART_PARITY_ODD  = 0x2U   /*!< Odd parity                             */
} usart_parity_t;

/**
 * @brief Stop bit configuration
 */
typedef enum {
    USART_STOPBITS_1   = 0x0U, /*!< 1 Stop bit                             */
    USART_STOPBITS_0_5 = 0x1U, /*!< 0.5 Stop bit                           */
    USART_STOPBITS_2   = 0x2U, /*!< 2 Stop bits                            */
    USART_STOPBITS_1_5 = 0x3U  /*!< 1.5 Stop bits                          */
} usart_stopbits_t;

/**
 * @brief Word length selection
 */
typedef enum {
    USART_WORDLENGTH_8BIT = 0x0U, /*!< 8-bit word length                     */
    USART_WORDLENGTH_9BIT = 0x1U  /*!< 9-bit word length                     */
} usart_wordlength_t;

/**
 * @brief USART TX/RX operating mode (bitmask-friendly)
 */
typedef enum {
    USART_MODE_RX    = 0x1U, /*!< Receive only                          */
    USART_MODE_TX    = 0x2U, /*!< Transmit only                         */
    USART_MODE_TX_RX = 0x3U  /*!< Transmit and receive                  */
} usart_mode_t;

/**
 * @brief Hardware flow control options
 */
typedef enum {
    USART_HWCONTROL_NONE    = 0x0U, /*!< No hardware flow control             */
    USART_HWCONTROL_RTS     = 0x1U, /*!< Enable RTS                           */
    USART_HWCONTROL_CTS     = 0x2U, /*!< Enable CTS                           */
    USART_HWCONTROL_RTS_CTS = 0x3U  /*!< Enable RTS and CTS                   */
} usart_hwflowctl_t;

/**
 * @brief USART synchronous/asynchronous mode
 */
typedef enum {
    USART_CLOCKMODE_ASYNCHRONOUS = 0x0U, /*!< No external clock (Async mode)     */
    USART_CLOCKMODE_SYNCHRONOUS  = 0x1U  /*!< External clock enabled (Sync mode) */
} usart_clockmode_t;

/**
 * @brief Clock polarity (CPOL)
 */
typedef enum {
    USART_CPOL_LOW  = 0x0U, /*!< Clock idle state low                  */
    USART_CPOL_HIGH = 0x1U  /*!< Clock idle state high                 */
} usart_clockpolarity_t;

/**
 * @brief Clock phase (CPHA)
 */
typedef enum {
    USART_CPHA_1EDGE = 0x0U, /*!< First clock transition capture       */
    USART_CPHA_2EDGE = 0x1U  /*!< Second clock transition capture      */
} usart_clockphase_t;

/**
 * @brief Oversampling selection
 */
typedef enum {
    USART_OVERSAMPLING_16 = 0x0U, /*!< Oversampling by 16 (default)        */
    USART_OVERSAMPLING_8  = 0x1U  /*!< Oversampling by 8                   */
} usart_oversampling_t;


/* -------------------------------------------------------------------------- */
/* Runtime Status / State / Error Enumerations                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Standard return codes (used globally across all drivers)
 */
typedef enum {
    STATUS_OK      = 0x00U, /*!< Operation successful                  */
    STATUS_ERROR   = 0x01U, /*!< General or hardware error             */
    STATUS_BUSY    = 0x02U, /*!< Peripheral or resource busy           */
    STATUS_TIMEOUT = 0x03U, /*!< Operation timed out                   */
    STATUS_PARAM   = 0x04U  /*!< Invalid parameter                     */
} status_t;

/**
 * @brief USART internal state machine
 * Tracks the runtime operation state of the USART peripheral
 */
typedef enum {
    STATE_RESET   = 0x00U, /*!< Peripheral not yet initialized         */
    STATE_READY   = 0x01U, /*!< Ready for use                          */
    STATE_BUSY_TX = 0x02U, /*!< Transmitting data                      */
    STATE_BUSY_RX = 0x04U, /*!< Receiving data                         */
    STATE_ERROR   = 0x08U  /*!< Error occurred                         */
} state_t;

/**
 * @brief USART error codes (maps directly to hardware SR flags)
 */
typedef enum {
    ERROR_NONE = 0x00U, /*!< No error                               */
    ERROR_PE   = 0x01U, /*!< Parity error                           */
    ERROR_FE   = 0x02U, /*!< Framing error                          */
    ERROR_NE   = 0x04U, /*!< Noise error                            */
    ERROR_ORE  = 0x08U  /*!< Overrun error                          */
} error_t;


/* -------------------------------------------------------------------------- */
/* Structures                                                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief USART initialization configuration structure
 * Contains all hardware-related configuration parameters.
 */
typedef struct
{
    uint32_t              BaudRate;       /*!< Baud rate (bps), e.g. 115200              */
    usart_oversampling_t  OverSampling;   /*!< Oversampling mode                         */
    usart_wordlength_t    WordLength;     /*!< Word length (8/9 bit)                     */
    usart_stopbits_t      StopBits;       /*!< Stop bits configuration                   */
    usart_parity_t        Parity;         /*!< Parity selection                          */
    usart_mode_t          Mode;           /*!< Transmit / Receive mode                   */
    usart_hwflowctl_t     HwFlowCtl;      /*!< Hardware flow control options             */
    usart_clockmode_t     ClockMode;      /*!< Clock mode (Async / Sync)                 */
    usart_clockpolarity_t ClockPolarity;  /*!< Clock polarity                            */
    usart_clockphase_t    ClockPhase;     /*!< Clock phase                               */
    uint32_t              TimeoutMs;      /*!< Timeout for blocking calls (ms)           */

} USART_InitTypeDef_t;

/**
 * @brief USART runtime handle structure
 * Holds peripheral registers, buffers, and runtime state.
 */
typedef struct
{
    USART_TypeDef_t      *Instance;     /*!< Peripheral base address                   */
    USART_InitTypeDef_t   Init;         /*!< Configuration parameters                  */

    const uint8_t        *TxBuffer;     /*!< Pointer to TX buffer                      */
    uint8_t              *RxBuffer;     /*!< Pointer to RX buffer                      */
    size_t                TxLength;     /*!< TX data length                            */
    size_t                RxLength;     /*!< RX data length                            */

    state_t               State;        /*!< Current operational state                 */
    error_t               LastError;    /*!< Last hardware error code                  */

} USART_HandleTypeDef_t;


/* ========================================================================== */
/*                     USART Register Flags & Constants                       */
/* ========================================================================== */

/* USART Baud Rate Constants */
#define USART_BAUDRATE_1200       (1200U)
#define USART_BAUDRATE_2400       (2400U)
#define USART_BAUDRATE_4800       (4800U)
#define USART_BAUDRATE_9600       (9600U)
#define USART_BAUDRATE_19200      (19200U)
#define USART_BAUDRATE_38400      (38400U)
#define USART_BAUDRATE_57600      (57600U)
#define USART_BAUDRATE_115200     (115200U)
#define USART_BAUDRATE_230400     (230400U)

/* USART Status Flags  */
#define USART_FLAG_PE     USART_SR_PE_MSK
#define USART_FLAG_FE     USART_SR_FE_MSK
#define USART_FLAG_NF     USART_SR_NF_MSK
#define USART_FLAG_ORE    USART_SR_ORE_MSK
#define USART_FLAG_IDLE   USART_SR_IDLE_MSK
#define USART_FLAG_RXNE   USART_SR_RXNE_MSK
#define USART_FLAG_TC     USART_SR_TC_MSK
#define USART_FLAG_TXE    USART_SR_TXE_MSK
#define USART_FLAG_LBD    USART_SR_LBD_MSK
#define USART_FLAG_CTS    USART_SR_CTS_MSK


/* ========================================================================== */
/*                        USART Driver API Prototypes                         */
/* ========================================================================== */

void USART_Init(USART_HandleTypeDef_t *husart);
void USART_DeInit(USART_HandleTypeDef_t *husart);

status_t USART_Transmit(USART_HandleTypeDef_t *husart, const uint8_t *pData, uint16_t Size);
status_t USART_Receive(USART_HandleTypeDef_t *husart, uint8_t *pData, uint16_t Size);

uint8_t USART_GetFlagStatus(USART_HandleTypeDef_t *husart, uint32_t flag);
void USART_ClearFlag(USART_HandleTypeDef_t *husart, uint32_t flag);

#ifdef __cplusplus
}
#endif

#endif /* INC_USART_H_ */
