/**
 * @file    i2c.h
 * @brief   I2C driver for STM32F407 
 * @author  Hasan Erol
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include "stm32f407xx.h"
#include "rcc.h"
#include "systick.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================================= */
/*                      I2C Configuration and Status Definitions                             */
/* ========================================================================================= */

/**
 * @enum status_t
 * @brief Standard return codes for all drivers (I2C, SPI, USART).
 */
typedef enum
{
    STATUS_OK      = 0x00U, /*!< Operation successful              */
    STATUS_PARAM   = 0x01U, /*!< Invalid or null parameter         */
    STATUS_BUSY    = 0x02U, /*!< Resource or bus busy              */
    STATUS_TIMEOUT = 0x03U, /*!< Operation timed out               */
    STATUS_ERROR   = 0x04U  /*!< General/unspecified error         */
} status_t;

/* ------------------------------------------------------------------------------- */
/* I2C Configuration Enumerations                                                  */
/* ------------------------------------------------------------------------------- */

/**
 * @brief I2C clock speed options.
 */
typedef enum
{
    I2C_SPEED_STANDARD = 100000U, /*!< Standard mode (100 kHz)          */
    I2C_SPEED_FAST     = 400000U  /*!< Fast mode (400 kHz)              */
} i2c_speed_t;

/**
 * @brief Acknowledge control configuration.
 */
typedef enum
{
    I2C_ACK_DISABLE = 0x00U, /*!< ACK disabled                      */
    I2C_ACK_ENABLE  = 0x01U  /*!< ACK enabled                       */
} i2c_ack_t;

/**
 * @brief Fast-mode duty cycle selection.
 */
typedef enum
{
    I2C_FM_DUTY_2    = 0x00U, /*!< t_low/t_high = 2                  */
    I2C_FM_DUTY_16_9 = 0x01U  /*!< t_low/t_high = 16/9               */
} i2c_fm_duty_t;

/**
 * @brief Repeated start configuration.
 */
typedef enum
{
    I2C_RS_DISABLE = 0x00U, /*!< Send STOP after transfer           */
    I2C_RS_ENABLE  = 0x01U  /*!< Issue repeated START condition     */
} i2c_rs_t;

/* ------------------------------------------------------------------- */
/* I2C Runtime State / Event / Error Enumerations                      */
/* ------------------------------------------------------------------- */

/**
 * @brief I2C internal state machine.
 */
typedef enum
{
    I2C_STATE_READY   = 0x00U, /*!< Peripheral ready                  */
    I2C_STATE_BUSY_RX = 0x01U, /*!< Receiving data                    */
    I2C_STATE_BUSY_TX = 0x02U, /*!< Transmitting data                 */
    I2C_STATE_ERROR   = 0x03U  /*!< Error state                       */
} i2c_state_t;

/**
 * @brief I2C event identifiers.
 * @note  Used for interrupt-driven or callback-based operations.
 */
typedef enum
{
    I2C_EVENT_TX_CMPLT = 0x00U, /*!< Transmission complete             */
    I2C_EVENT_RX_CMPLT = 0x01U, /*!< Reception complete                */
    I2C_EVENT_STOP     = 0x02U, /*!< STOP condition detected           */
    I2C_EVENT_DATA_REQ = 0x03U, /*!< Data request from master          */
    I2C_EVENT_DATA_RCV = 0x04U  /*!< Data received from master         */
} i2c_event_t;

/**
 * @brief I2C hardware error codes.
 */
typedef enum
{
    I2C_ERROR_NONE    = 0x00U, /*!< No error                          */
    I2C_ERROR_BERR    = 0x01U, /*!< Bus error                         */
    I2C_ERROR_ARLO    = 0x02U, /*!< Arbitration lost                  */
    I2C_ERROR_AF      = 0x03U, /*!< Acknowledge failure               */
    I2C_ERROR_OVR     = 0x04U, /*!< Overrun/Underrun error            */
    I2C_ERROR_TIMEOUT = 0x05U  /*!< Timeout occurred                  */
} i2c_error_t;


/* ================================================================================== */
/*                                    I2C Structures                                  */
/* ================================================================================== */

/**
 * @brief I2C initialization structure definition
 */
typedef struct
{
    uint32_t       ClockSpeedHz; /*!< I2C clock speed in Hz (e.g. 100k/400k)   */
    i2c_ack_t      AckState;     /*!< ACK enable/disable                       */
    i2c_fm_duty_t  DutyCycle;    /*!< Fast Mode duty cycle                     */
    uint32_t       TimeoutMs;    /*!< Timeout duration for blocking ops        */
} I2C_InitTypeDef_t;

/**
 * @brief I2C handle structure definition
 */
typedef struct
{
    I2C_TypeDef_t     *Instance;       /*!< I2C peripheral base address             */
    I2C_InitTypeDef_t  Init;           /*!< I2C configuration parameters            */

    const uint8_t     *TxBuffer;       /*!< Pointer to TX buffer                    */
    uint8_t           *RxBuffer;       /*!< Pointer to RX buffer                    */
    size_t             TxLength;       /*!< TX length in bytes                      */
    size_t             RxLength;       /*!< RX length in bytes                      */

    uint8_t            DevAddress;     /*!< Current slave address                   */
    i2c_state_t        State;          /*!< Communication state                     */
    i2c_rs_t           RepeatedStart;  /*!< Repeated Start condition control        */

    uint32_t           TimeoutMs;      /*!< Timeout for blocking ops                */
    i2c_error_t        LastError;      /*!< Last recorded error                     */
} I2C_HandleTypeDef_t;

/* --------------------------------------------------------------------------------------------- */
/* I2C Status Flags (SR1)                                                                        */
/* --------------------------------------------------------------------------------------------- */

#define I2C_FLAG_TXE           (1U << I2C_SR1_TXE_POS)         /*!< Data register empty (TX)     */
#define I2C_FLAG_RXNE          (1U << I2C_SR1_RXNE_POS)        /*!< Data register not empty (RX) */
#define I2C_FLAG_SB            (1U << I2C_SR1_SB_POS)          /*!< Start bit (Master mode)      */
#define I2C_FLAG_OVR           (1U << I2C_SR1_OVR_POS)         /*!< Overrun/Underrun             */
#define I2C_FLAG_AF            (1U << I2C_SR1_AF_POS)          /*!< Acknowledge failure          */
#define I2C_FLAG_ARLO          (1U << I2C_SR1_ARLO_POS)        /*!< Arbitration lost             */
#define I2C_FLAG_STOPF         (1U << I2C_SR1_STOPF_POS)       /*!< Stop detection (slave mode)  */
#define I2C_FLAG_ADD10         (1U << I2C_SR1_ADD10_POS)       /*!< 10-bit header sent           */
#define I2C_FLAG_BTF           (1U << I2C_SR1_BTF_POS)         /*!< Byte transfer finished       */
#define I2C_FLAG_ADDR          (1U << I2C_SR1_ADDR_POS)        /*!< Address sent/matched         */
#define I2C_FLAG_TIMEOUT       (1U << I2C_SR1_TIMEOUT_POS)     /*!< Timeout or Tlow error        */


/* ========================================================================================= */
/*                                   Public API Prototypes                                   */
/* ========================================================================================= */

/* Initialization / Deinitialization ------------------------------------------------------- */
void I2C_Init(I2C_HandleTypeDef_t *hi2c);
void I2C_DeInit(I2C_HandleTypeDef_t *hi2c);

/* Blocking Communication ------------------------------------------------------------------ */
status_t I2C_MasterTransmit  (I2C_HandleTypeDef_t *hi2c, uint8_t addr7, const uint8_t *buf, size_t len, i2c_rs_t rs);

status_t I2C_MasterReceive   (I2C_HandleTypeDef_t *hi2c, uint8_t addr7, uint8_t *buf, size_t len, i2c_rs_t rs);

status_t I2C_MemWrite        (I2C_HandleTypeDef_t *hi2c, uint8_t addr7, uint8_t reg, const uint8_t *data, size_t len);

status_t I2C_MemRead         (I2C_HandleTypeDef_t *hi2c, uint8_t addr7, uint8_t reg, uint8_t *data, size_t len);

/* Interrupt-based Communication ------------------------------------------------------------ */
status_t I2C_MasterTransmitIT(I2C_HandleTypeDef_t *hi2c, uint8_t addr7, const uint8_t *buf, size_t len, i2c_rs_t rs);

status_t I2C_MasterReceiveIT (I2C_HandleTypeDef_t *hi2c, uint8_t addr7, uint8_t *buf, size_t len, i2c_rs_t rs);

void I2C_CloseTransmitIT(I2C_HandleTypeDef_t *hi2c);
void I2C_CloseReceiveIT(I2C_HandleTypeDef_t *hi2c);

/* IRQ Handling ----------------------------------------------------------------------------- */
void I2C_IRQEnable           (uint8_t IRQNumber, uint8_t enable);
void I2C_IRQSetPriority      (uint8_t IRQNumber, uint32_t priority);
void I2C_EV_IRQHandler       (I2C_HandleTypeDef_t *hi2c);
void I2C_ER_IRQHandler       (I2C_HandleTypeDef_t *hi2c);

/* Application Callback --------------------------------------------------------------------- */
void I2C_ApplicationEventCallback(I2C_HandleTypeDef_t *hi2c, i2c_event_t event, i2c_error_t error);

#ifdef __cplusplus
}
#endif
#endif /* INC_I2C_H_ */
