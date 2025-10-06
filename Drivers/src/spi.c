/**
 * @file    spi.c
 * @brief   Bare-metal SPI driver implementation for STM32F407
 * @author  Hasan Erol
 */

#include "SPI.h"

/**
 * @brief  Get the status of the specified SPI flag.
 * @param  SPIx      Pointer to SPI peripheral base
 * @param  FlagName  Flag to check (e.g. @ref SPI_FLAG_TXE)
 * @retval 1 if set, 0 otherwise
 */
static inline uint8_t SPI_GetFlag(SPI_TypeDef_t *SPIx, uint32_t FlagName)
{
    return (uint8_t)((SPIx->SR & FlagName) ? 1U : 0U);
}


/**
 * @brief  Wait until a specific flag is set or timeout occurs.
 * @param  hspi  SPI handle pointer
 * @param  flag  Flag to monitor
 * @retval STATUS_OK if flag is set, STATUS_TIMEOUT otherwise
 */
status_t SPI_WaitFlag(SPI_HandleTypeDef_t *hspi, uint32_t flag)
{
    uint32_t start = SysTick_GetTick();
    while (!SPI_GetFlag(hspi->Instance, flag))
    {
        if ((SysTick_GetTick() - start) >= hspi->Init.TimeoutMs)
        {
            return STATUS_TIMEOUT;
        }
    }
    return STATUS_OK;
}


/**
 * @brief  Initializes the SPI peripheral according to the specified parameters
 *         in the SPI handle structure.
 * @param  hspi Pointer to a SPI_HandleTypeDef_t structure that contains
 *         the configuration information for SPI module.
 * @retval None
 */
void SPI_Init(SPI_HandleTypeDef_t *hspi)
{
    if ((hspi == NULL) || (hspi->Instance == NULL))
    {
        return;
    }

    /* Default timeout */
    if (hspi->Init.TimeoutMs == 0U)
    {
        hspi->Init.TimeoutMs = 1000U;
    }

    SPI_TypeDef_t *SPIx = hspi->Instance;
    uint32_t tempReg = 0U;

    /* Disable SPI before configuration */
    SPIx->CR1 &= ~SPI_CR1_SPE_MSK;

    /* Clear configurable bits */
    tempReg = SPIx->CR1;
    tempReg &= ~(SPI_CR1_BR_MSK | SPI_CR1_CPOL_MSK | SPI_CR1_CPHA_MSK |
                 SPI_CR1_DFF_MSK | SPI_CR1_MSTR_MSK | SPI_CR1_SSM_MSK |
                 SPI_CR1_SSI_MSK | SPI_CR1_BIDIMODE_MSK | SPI_CR1_RXONLY_MSK);

    /* Baud rate */
    tempReg |= (hspi->Init.BaudRate << SPI_CR1_BR_POS);

    /* Clock polarity and phase */
    tempReg |= (hspi->Init.CPOL << SPI_CR1_CPOL_POS) | (hspi->Init.CPHA << SPI_CR1_CPHA_POS);

    /* Data frame format */
    tempReg |= (hspi->Init.DataSize << SPI_CR1_DFF_POS);

    /* Device mode */
    tempReg |= (hspi->Init.DeviceMode << SPI_CR1_MSTR_POS);

    /* Software slave management */
    tempReg |= (hspi->Init.SSM << SPI_CR1_SSM_POS);
    if (hspi->Init.SSM == SPI_SSM_ENABLE)
    {
        tempReg |= SPI_CR1_SSI_MSK;
    }

    /* Bus configuration */
    if (hspi->Init.BusConfig == SPI_BUS_FD)
    {
        tempReg &= ~SPI_CR1_BIDIMODE_MSK;   /* Full duplex */
    }
    else if (hspi->Init.BusConfig == SPI_BUS_HD)
    {
        tempReg |= SPI_CR1_BIDIMODE_MSK;    /* Half duplex */
    }
    else if (hspi->Init.BusConfig == SPI_BUS_RX_ONLY)
    {
        tempReg &= ~SPI_CR1_BIDIMODE_MSK;
        tempReg |= SPI_CR1_RXONLY_MSK;      /* Simplex receive-only */
    }

    /* Write back configuration */
    SPIx->CR1 = tempReg;

    /* Enable SPI peripheral */
    SPIx->CR1 |= SPI_CR1_SPE_MSK;
}


/**
 * @brief  Transmits data over SPI in blocking (polling) mode.
 * @param  hspi Pointer to SPI handle structure.
 * @param  pData Pointer to transmission buffer.
 * @param  len Number of bytes to transmit.
 * @retval status_t STATUS_OK on success, STATUS_TIMEOUT or STATUS_ERROR otherwise.
 */
status_t SPI_Transmit(SPI_HandleTypeDef_t *hspi, const uint8_t *pTxBuffer, uint32_t Len)
{
    if ((hspi == NULL) || (hspi->Instance == NULL) || (pTxBuffer == NULL) || (Len == 0U)){
        return STATUS_PARAM;
    }

    SPI_TypeDef_t *SPIx = hspi->Instance;

    while (Len > 0U)
    {
        /* If half-duplex, set transmit-only (BIDIOE = 1) */
        if (SPIx->CR1 & SPI_CR1_BIDIMODE_MSK)
        {
            SPIx->CR1 |= SPI_CR1_BIDIOE_MSK;
        }

        /* Wait until TXE = 1 (transmit buffer empty) */
        if(SPI_WaitFlag(hspi, SPI_FLAG_TXE) != STATUS_OK){
        	return STATUS_TIMEOUT;
        }
        /* Check data frame format */
        if (SPIx->CR1 & SPI_CR1_DFF_MSK)
        {
            /* 16-bit frame */
            if (Len >= 2U)
            {
                *(__IO uint16_t *)&SPIx->DR = *((const uint16_t *)pTxBuffer);
                pTxBuffer += 2U;
                Len -= 2U;
            }
            else
            {
                /* Odd size protection */
                *(__IO uint16_t *)&SPIx->DR = (uint16_t)(*pTxBuffer);
                Len = 0U;
            }
        }
        else
        {
            /* 8-bit frame */
            *(__IO uint8_t *)&SPIx->DR = *pTxBuffer;
            pTxBuffer++;
            Len--;
        }
    }

    /* Wait for TXE = 1 and BSY = 0 (complete transmission) */
    if(SPI_WaitFlag(hspi, SPI_FLAG_TXE) != STATUS_OK){
    	return STATUS_TIMEOUT;
    }

    if(SPI_WaitFlag(hspi, SPI_FLAG_BSY) != STATUS_OK){
    	return STATUS_TIMEOUT;
    }

    /* Clear OVR: read DR then SR */
    (void)SPIx->DR;
    (void)SPIx->SR;

    return STATUS_OK;
}


/**
 * @brief  Receives data over SPI in blocking (polling) mode.
 * @param  hspi Pointer to SPI handle structure.
 * @param  pData Pointer to buffer to store received data.
 * @param  len Number of bytes to receive.
 * @retval status_t STATUS_OK on success, STATUS_TIMEOUT or STATUS_ERROR otherwise.
 */

status_t SPI_Receive(SPI_HandleTypeDef_t *hspi, uint8_t *pRxBuffer, uint32_t Len)
{
    if ((hspi == NULL) || (hspi->Instance == NULL) || (pRxBuffer == NULL) || (Len == 0U))
    {
        return STATUS_PARAM;
    }

    SPI_TypeDef_t *SPIx = hspi->Instance;


    while (Len > 0U)
    {
        /* If half-duplex, set receive-only direction (BIDIOE = 0) */
        if (SPIx->CR1 & SPI_CR1_BIDIMODE_MSK)
        {
            SPIx->CR1 &= ~SPI_CR1_BIDIOE_MSK;
        }

        /* Wait until TXE = 1, then send dummy to generate clock */
        while (SPI_GetFlagStatus(SPIx, SPI_FLAG_TXE) == 0U);

        if (SPIx->CR1 & SPI_CR1_DFF_MSK)
        {
            /* 16-bit dummy write */
            *(__IO uint16_t *)&SPIx->DR = 0xFFFFU;
        }
        else
        {
            /* 8-bit dummy write */
            *(__IO uint8_t *)&SPIx->DR = 0xFFU;
        }

        /* Wait until RXNE = 1 */
        if(SPI_WaitFlag(hspi, SPI_FLAG_RXNE) != STATUS_OK){
        	return STATUS_TIMEOUT;
        }
        if (SPIx->CR1 & SPI_CR1_DFF_MSK)
        {
            if (Len >= 2U)
            {
                *((uint16_t *)pRxBuffer) = *(__IO uint16_t *)&SPIx->DR;
                pRxBuffer += 2U;
                Len -= 2U;
            }
            else
            {
                /* Handle odd-length safety (rare case) */
                *(__IO uint8_t *)&SPIx->DR = 0xFFU;

                if(SPI_WaitFlag(hspi, SPI_FLAG_RXNE) != STATUS_OK){
                	return STATUS_TIMEOUT;
                }

                *pRxBuffer = *(__IO uint8_t *)&SPIx->DR;
                pRxBuffer++;
                Len--;
            }
        }
        else
        {
            *pRxBuffer = *(__IO uint8_t *)&SPIx->DR;
            pRxBuffer++;
            Len--;
        }
    }

    /* Clear potential OVR */
    (void)SPIx->DR;
    (void)SPIx->SR;

    return STATUS_OK;
}


/**
 * @brief  Transmit and receive data simultaneously over SPI (Full-Duplex mode)
 * @param  hspi     Pointer to SPI handle structure
 * @param  pTxData  Pointer to transmit buffer
 * @param  pRxData  Pointer to receive buffer
 * @param  len      Number of bytes to transmit/receive
 * @retval STATUS_OK if successful, STATUS_TIMEOUT if timed out
 */
status_t SPI_TransmitReceive(SPI_HandleTypeDef_t *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t len)
{
    if ((hspi == NULL) || (pTxData == NULL) || (pRxData == NULL) || (len == 0U))
        return STATUS_PARAM;

    SPI_TypeDef_t *SPIx = hspi->Instance;
    uint32_t startTick;

    for (uint16_t i = 0; i < len; i++)
    {
        /* Wait for TXE flag */
        startTick = SysTick_GetTick();
        while (!SPI_GetFlagStatus(SPIx, SPI_FLAG_TXE))
        {
            if ((SysTick_GetTick() - startTick) >= hspi->Init.TimeoutMs)
                return STATUS_TIMEOUT;
        }

        /* Send one byte */
        *(__IO uint8_t *)&SPIx->DR = pTxData[i];

        /* Wait for RXNE flag (data received) */
        startTick = SysTick_GetTick();
        while (!SPI_GetFlagStatus(SPIx, SPI_FLAG_RXNE))
        {
            if ((SysTick_GetTick() - startTick) >= hspi->Init.TimeoutMs)
                return STATUS_TIMEOUT;
        }

        /* Read received byte */
        pRxData[i] = *(__IO uint8_t *)&SPIx->DR;
    }

    /* Wait for BSY flag to clear (transfer complete) */
    startTick = SysTick_GetTick();
    while (SPI_GetFlagStatus(SPIx, SPI_FLAG_BSY))
    {
        if ((SysTick_GetTick() - startTick) >= hspi->Init.TimeoutMs)
            return STATUS_TIMEOUT;
    }

    return STATUS_OK;
}


/**
 * @brief  Deinitializes the specified SPI peripheral and resets its handle.
 * @param  hspi Pointer to an SPI_HandleTypeDef_t structure that contains
 *         the configuration information for the specified SPI module.
 * @retval None
 *
 * @note   Performs a hardware peripheral reset via RCC and clears the
 *         handle fields to their default values.
 */
void SPI_DeInit(SPI_HandleTypeDef_t *hspi)
{
    if ((hspi == NULL) || (hspi->Instance == NULL))
    {
        return;
    }

    /* ---------------------- Hardware reset via RCC ---------------------- */
    if (hspi->Instance == SPI1)
    {
        SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
        CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST);
    }
    else if (hspi->Instance == SPI2)
    {
        SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_SPI2RST);
        CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_SPI2RST);
    }
    else if (hspi->Instance == SPI3)
    {
        SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_SPI3RST);
        CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_SPI3RST);
    }

    /* ---------------------- Software cleanup ---------------------- */
    hspi->State      = SPI_STATE_READY;
    hspi->LastError  = SPI_ERROR_NONE;
    hspi->TxBuffer   = NULL;
    hspi->RxBuffer   = NULL;
    hspi->TxLength   = 0U;
    hspi->RxLength   = 0U;
}
