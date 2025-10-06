/**
 * @file    spi.h
 * @brief   Bare-Metal SPI driver for STM32F407
 * @author  Hasan Erol
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "stm32f407xx.h"
#include "rcc.h"
#include "systick.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                              Common Status / State                         */
/* ========================================================================== */

/**
 * @brief Standard return codes for all peripheral drivers
 * @note  Used as a unified return type across SPI, I2C, and USART drivers.
 */
typedef enum
{
    STATUS_OK      = 0x00U, /*!< Operation completed successfully          */
    STATUS_ERROR   = 0x01U, /*!< General or hardware error                 */
    STATUS_BUSY    = 0x02U, /*!< Peripheral or resource busy               */
    STATUS_TIMEOUT = 0x03U, /*!< Operation timed out                       */
    STATUS_PARAM   = 0x04U  /*!< Invalid or null parameter                 */
} status_t;

/**
 * @brief SPI internal state machine
 * @note  Tracks the current operational state of the SPI peripheral.
 */
typedef enum
{
    SPI_STATE_RESET   = 0x00U, /*!< Not yet initialized or disabled          */
    SPI_STATE_READY   = 0x01U, /*!< Initialized and ready for use            */
    SPI_STATE_BUSY_TX = 0x02U, /*!< Transmission ongoing                     */
    SPI_STATE_BUSY_RX = 0x03U, /*!< Reception ongoing                        */
    SPI_STATE_ERROR   = 0x04U  /*!< Error detected                           */
} spi_state_t;

/**
 * @brief SPI hardware error codes
 * @note  Indicates specific hardware-related error conditions.
 */
typedef enum
{
    SPI_ERROR_NONE = 0x00U, /*!< No error                                 */
    SPI_ERROR_MODF = 0x01U, /*!< Mode fault error                         */
    SPI_ERROR_OVR  = 0x02U, /*!< Overrun error                            */
    SPI_ERROR_CRC  = 0x04U  /*!< CRC mismatch (if CRC enabled)            */
} spi_error_t;


/* ========================================================================== */
/*                              SPI Enumerations                              */
/* ========================================================================== */

/**
 * @brief SPI device operating mode
 */
typedef enum
{
    SPI_DEVICE_MODE_SLAVE  = 0U, /*!< Operates as slave device                */
    SPI_DEVICE_MODE_MASTER = 1U  /*!< Operates as master device               */
} spi_device_mode_t;

/**
 * @brief SPI bus configuration
 */
typedef enum
{
    SPI_BUS_FD      = 0U, /*!< Full-duplex mode                         */
    SPI_BUS_HD      = 1U, /*!< Half-duplex mode                         */
    SPI_BUS_RX_ONLY = 2U  /*!< Simplex receive-only mode                */
} spi_bus_config_t;

/**
 * @brief SPI clock polarity
 */
typedef enum
{
    SPI_CPOL_LOW  = 0U, /*!< Clock idle state low                     */
    SPI_CPOL_HIGH = 1U  /*!< Clock idle state high                    */
} spi_cpol_t;

/**
 * @brief SPI clock phase
 */
typedef enum
{
    SPI_CPHA_1EDGE = 0U, /*!< First clock transition is first data capture  */
    SPI_CPHA_2EDGE = 1U  /*!< Second clock transition is first data capture */
} spi_cpha_t;

/**
 * @brief SPI data frame format
 */
typedef enum
{
    SPI_DATASIZE_8BIT  = 0U, /*!< 8-bit data frame format                  */
    SPI_DATASIZE_16BIT = 1U  /*!< 16-bit data frame format                 */
} spi_datasize_t;

/**
 * @brief SPI baud rate prescaler
 */
typedef enum
{
    SPI_BAUD_DIV2   = 0U, /*!< Baud rate = f_PCLK / 2                   */
    SPI_BAUD_DIV4   = 1U, /*!< Baud rate = f_PCLK / 4                   */
    SPI_BAUD_DIV8   = 2U, /*!< Baud rate = f_PCLK / 8                   */
    SPI_BAUD_DIV16  = 3U, /*!< Baud rate = f_PCLK / 16                  */
    SPI_BAUD_DIV32  = 4U, /*!< Baud rate = f_PCLK / 32                  */
    SPI_BAUD_DIV64  = 5U, /*!< Baud rate = f_PCLK / 64                  */
    SPI_BAUD_DIV128 = 6U, /*!< Baud rate = f_PCLK / 128                 */
    SPI_BAUD_DIV256 = 7U  /*!< Baud rate = f_PCLK / 256                 */
} spi_baudrate_t;

/**
 * @brief SPI software slave management (SSM)
 */
typedef enum
{
    SPI_SSM_DISABLE = 0U, /*!< Hardware NSS management (NSS pin active) */
    SPI_SSM_ENABLE  = 1U  /*!< Software NSS management (internal)       */
} spi_ssm_t;


/* ========================================================================== */
/*                              SPI Structures                                */
/* ========================================================================== */

/**
 * @brief SPI initialization configuration
 */
typedef struct
{
    spi_device_mode_t DeviceMode;   /*!< Master/slave mode */
    spi_bus_config_t  BusConfig;    /*!< Full/Half duplex or RX only */
    spi_baudrate_t    BaudRate;     /*!< Baud rate prescaler */
    spi_datasize_t    DataSize;     /*!< 8/16-bit data frame */
    spi_cpol_t        CPOL;         /*!< Clock polarity */
    spi_cpha_t        CPHA;         /*!< Clock phase */
    spi_ssm_t         SSM;          /*!< Software slave management */
    uint32_t          TimeoutMs;    /*!< Timeout for blocking operations */
} SPI_InitTypeDef_t;

/**
 * @brief SPI handle structure
 */
typedef struct
{
    SPI_TypeDef_t    *Instance;     /*!< SPI register base address */
    SPI_InitTypeDef_t Init;         /*!< Configuration parameters  */

    const uint8_t    *TxBuffer;     /*!< Pointer to TX buffer */
    uint8_t          *RxBuffer;     /*!< Pointer to RX buffer */
    size_t            TxLength;     /*!< TX data length */
    size_t            RxLength;     /*!< RX data length */

    spi_state_t       State;        /*!< Current communication state */
    spi_error_t       LastError;    /*!< Last operation result        */
} SPI_HandleTypeDef_t;

/* ========================================================================== */
/*                              Inline Utilities                              */
/* ========================================================================== */

/**
 * @brief Enable the SPI peripheral
 */
static inline void SPI_Enable(SPI_TypeDef_t *SPIx)
{
    SPIx->CR1 |= SPI_CR1_SPE_MSK;
}

/**
 * @brief Disable the SPI peripheral
 */
static inline void SPI_Disable(SPI_TypeDef_t *SPIx)
{
    SPIx->CR1 &= ~SPI_CR1_SPE_MSK;
}

/* ========================================================================== */
/*                              SPI Flag Macros                              */
/* ========================================================================== */

#define SPI_FLAG_RXNE     SPI_SR_RXNE_MSK   /*!< Receive buffer not empty         */
#define SPI_FLAG_TXE      SPI_SR_TXE_MSK    /*!< Transmit buffer empty            */
#define SPI_FLAG_MODF     SPI_SR_MODF_MSK   /*!< Mode fault flag                  */
#define SPI_FLAG_OVR      SPI_SR_OVR_MSK    /*!< Overrun flag                     */
#define SPI_FLAG_BSY      SPI_SR_BSY_MSK    /*!< Busy flag                        */
#define SPI_FLAG_CRCERR   SPI_SR_CRCERR_MSK /*!< CRC error flag                   */
#define SPI_FLAG_FRE      SPI_SR_FRE_MSK    /*!< Frame format error (TI mode)     */

/* ========================================================================== */
/*                              API Prototypes                                */
/* ========================================================================== */

void SPI_Init(SPI_HandleTypeDef_t *hspi);
void SPI_DeInit(SPI_HandleTypeDef_t *hspi);

status_t SPI_Transmit(SPI_HandleTypeDef_t *hspi, const uint8_t *pData, uint32_t len);
status_t SPI_Receive(SPI_HandleTypeDef_t *hspi, uint8_t *pData, uint32_t len);
status_t SPI_TransmitReceive(SPI_HandleTypeDef_t *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t len);


uint8_t SPI_GetFlagStatus(SPI_TypeDef_t *SPIx, uint32_t flag);
status_t SPI_WaitFlag(SPI_HandleTypeDef_t *hspi, uint32_t flag);

#ifdef __cplusplus
}
#endif

#endif /* INC_SPI_H_ */
