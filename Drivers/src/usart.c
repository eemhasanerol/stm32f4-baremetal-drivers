/**
 * @file    usart.c
 * @brief   USART driver implementation for STM32F407
 * @author  Hasan Erol
 */

#include "USART.h"

/**
 * @brief  Initializes the USART peripheral according to the specified parameters
 *         in the USART handle structure.
 * @param  husart Pointer to a USART_HandleTypeDef_t structure that contains
 *         the configuration information for the specified USART module.
 */
void USART_Init(USART_HandleTypeDef_t *husart)
{
	if ((husart == NULL) || (husart->Instance == NULL))
	{
		return ;
	}

	if (husart->Init.TimeoutMs== 0U)
	{
	    husart->Init.TimeoutMs = 1000U;
	}

    USART_TypeDef_t *USARTx = husart->Instance;

    /* Disable USART before configuration */
    USARTx->CR1 &=  ~USART_CR1_UE_MSK;

    /* ---------------------- Configure CR1 ---------------------- */
    uint32_t tempReg = USARTx->CR1;
    tempReg &= ~(USART_CR1_M_MSK | USART_CR1_OVER8_MSK | USART_CR1_PCE_MSK |
                 USART_CR1_TE_MSK | USART_CR1_RE_MSK | USART_CR1_PS_MSK);

    /* Word length */
    if (husart->Init.WordLength == USART_WORDLENGTH_9BIT)
    {
        tempReg |= USART_CR1_M_MSK;
    }

    /* Oversampling */
    if (husart->Init.OverSampling == USART_OVERSAMPLING_8)
    {
        tempReg |= USART_CR1_OVER8_MSK;
    }

    /* Parity configuration */
    if (husart->Init.Parity == USART_PARITY_EVEN)
    {
        tempReg |= USART_CR1_PCE_MSK;
    }
    else if (husart->Init.Parity == USART_PARITY_ODD)
    {
        tempReg |= (USART_CR1_PCE_MSK | USART_CR1_PS_MSK);
    }

    /* TX / RX mode */
    if (husart->Init.Mode & USART_MODE_RX)
    {
        tempReg |= USART_CR1_RE_MSK;
    }

    if (husart->Init.Mode & USART_MODE_TX)
    {
        tempReg |= USART_CR1_TE_MSK;
    }

    USARTx->CR1 = tempReg;


    /* ---------------------- Configure CR2 ---------------------- */
    tempReg = USARTx->CR2;
    tempReg &= ~(USART_CR2_STOP_MSK | USART_CR2_CLKEN_MSK |
                 USART_CR2_CPOL_MSK | USART_CR2_CPHA_MSK);

    /* Stop bits */
    tempReg |= (husart->Init.StopBits << USART_CR2_STOP_POS);

    /* Synchronous clock configuration */
    if (husart->Init.ClockMode == USART_CLOCKMODE_SYNCHRONOUS)
    {
        tempReg |= USART_CR2_CLKEN_MSK;

        if (husart->Init.ClockPolarity == USART_CPOL_HIGH)
		{
        	tempReg |= USART_CR2_CPOL_MSK;
		}

        if (husart->Init.ClockPhase == USART_CPHA_2EDGE)
		{
        	tempReg |= USART_CR2_CPHA_MSK;
		}
    }

    USARTx->CR2 = tempReg;

    /* ---------------------- Configure CR3 (Hardware Flow Control) ---------------------- */
    tempReg = USARTx->CR3;
    tempReg &= ~(USART_CR3_RTSE_MSK | USART_CR3_CTSE_MSK);  /* Clear RTS/CTS bits */

    if (husart->Init.HwFlowCtl & USART_HWCONTROL_RTS)
    {
        tempReg |= USART_CR3_RTSE_MSK;
    }

    if (husart->Init.HwFlowCtl & USART_HWCONTROL_CTS)
    {
        tempReg |= USART_CR3_CTSE_MSK;
    }

    USARTx->CR3 = tempReg;

    /* ---------------------- Configure BRR (Baud Rate)---------------------- */
     uint32_t pclk;
     if (USARTx == USART1 || USARTx == USART6)
     {
         pclk = RCC_GetPCLK2Freq();
     }
     else
     {
         pclk = RCC_GetPCLK1Freq();
     }

     uint32_t usartdiv;
     if (husart->Init.OverSampling == USART_OVERSAMPLING_8)
     {
         usartdiv = (25U * pclk) / (2U * husart->Init.BaudRate);
         uint32_t mantissa = (usartdiv / 100U) << 4U;
         uint32_t fraction = (((usartdiv - (100U * (mantissa >> 4U))) * 8U + 50U) / 100U) & 0x07U;
         USARTx->BRR = mantissa | fraction;
     }
     else
     {
         usartdiv = (25U * pclk) / (4U * husart->Init.BaudRate);
         uint32_t mantissa = (usartdiv / 100U) << 4U;
         uint32_t fraction = (((usartdiv - (100U * (mantissa >> 4U))) * 16U + 50U) / 100U) & 0x0FU;
         USARTx->BRR = mantissa | fraction;
     }

     /* USART Enable  */
     USARTx->CR1 |=  USART_CR1_UE_MSK;

     /* USART ready  */
     husart->State = STATE_READY;
}


/**
 * @brief  Transmits data over USART (blocking mode).
 * @param  husart   Pointer to USART handle
 * @param  pData    Pointer to data buffer
 * @param  Size     Number of bytes to transmit
 * @retval status_t
 */
status_t USART_Transmit(USART_HandleTypeDef_t *husart, const uint8_t *pData, uint16_t Size)
{
    USART_TypeDef_t *USARTx = husart->Instance;
    uint32_t startTick;

    /* --- Parameter validation --- */
    if ((pData == NULL) || (Size == 0U))
    {
        return STATUS_PARAM;
    }

    /* --- Check if peripheral is busy --- */
    if ((husart->State == STATE_BUSY_TX) || (husart->State == STATE_BUSY_RX))
    {
        return STATUS_BUSY;
    }

    husart->State = STATE_BUSY_TX;

    /* --- Transmit loop --- */
    for (uint16_t i = 0; i < Size; i++)
    {
        /* Wait until TXE flag is set */
        startTick = SysTick_GetTick();
        while (!(USARTx->SR & USART_FLAG_TXE))
        {
            if ((SysTick_GetTick() - startTick) >= husart->Init.TimeoutMs)
            {
                husart->State = STATE_READY;
                return STATUS_TIMEOUT;
            }
        }

        USARTx->DR = pData[i]; /* Send byte */
    }

    /* --- Wait for TC (Transmission Complete) --- */
    startTick = SysTick_GetTick(); /* Reset timeout counter */
    while (!(USARTx->SR & USART_FLAG_TC))
    {
        if ((SysTick_GetTick() - startTick) >= husart->Init.TimeoutMs)
        {
            husart->State = STATE_READY;
            return STATUS_TIMEOUT;
        }
    }

    husart->State = STATE_READY;
    return STATUS_OK;
}


/**
 * @brief  Receives data over USART (blocking mode).
 * @param  husart   Pointer to USART handle
 * @param  pData    Pointer to buffer to store received data
 * @param  Size     Number of bytes to receive
 * @retval status_t
 */
status_t USART_Receive(USART_HandleTypeDef_t *husart, uint8_t *pData, uint16_t Size)
{
    /* --- Parameter validation --- */
    if ((pData == NULL) || (Size == 0U))
    {
        return STATUS_PARAM;
    }

    USART_TypeDef_t *USARTx = husart->Instance;
    uint32_t startTick = SysTick_GetTick();

    /* --- Check if peripheral is busy --- */
    if ((husart->State == STATE_BUSY_TX) || (husart->State == STATE_BUSY_RX))
    {
        return STATUS_BUSY;
    }

    husart->State = STATE_BUSY_RX;

    /* --- Receive loop --- */
    for (uint16_t i = 0; i < Size; i++)
    {
        /* Wait until RXNE flag is set (data received) */
        while (!(USARTx->SR & USART_FLAG_RXNE))
        {
            if ((SysTick_GetTick() - startTick) >= husart->Init.TimeoutMs)
            {
                husart->State = STATE_READY;
                return STATUS_TIMEOUT;
            }
        }

        /* Read received data */
        pData[i] = (uint8_t)(USARTx->DR & 0xFFU);
    }

    /* --- Wait for TC (ensures end of reception) --- */
    startTick = SysTick_GetTick();
    while (!(USARTx->SR & USART_FLAG_TC))
    {
        if ((SysTick_GetTick() - startTick) >= husart->Init.TimeoutMs)
        {
            husart->State = STATE_READY;
            return STATUS_TIMEOUT;
        }
    }

    husart->State = STATE_READY;
    return STATUS_OK;
}


/**
 * @brief  Checks whether the specified USART flag is set.
 * @param  husart Pointer to USART handle
 * @param  flag   USART flag to check (e.g. @ref USART_FLAG_TC)
 * @retval uint8_t Returns 1 if the flag is set, otherwise 0.
 */
uint8_t USART_GetFlagStatus(USART_HandleTypeDef_t *husart, uint32_t flag)
{
    return ((husart->Instance->SR & flag) ? 1U : 0U);
}

/**
 * @brief  Clears the specified USART flag.
 * @param  husart Pointer to a USART handle structure.
 * @param  flag   Flag to clear. @ref USART_Flags
 * @retval None
 */
void USART_ClearFlag(USART_HandleTypeDef_t *husart, uint32_t flag)
{
    if ((husart == NULL) || (husart->Instance == NULL))
        return;

    USART_TypeDef_t *USARTx = husart->Instance;

    /* Flags cleared by writing 0 to SR */
    if ((flag == USART_FLAG_TC) || (flag == USART_FLAG_CTS) || (flag == USART_FLAG_LBD))
    {
        USARTx->SR &= ~flag;
    }
    /* Flags cleared by reading SR and DR */
    else if ((flag == USART_FLAG_PE) || (flag == USART_FLAG_FE) ||
             (flag == USART_FLAG_NF) || (flag == USART_FLAG_ORE))
    {
        (void)USARTx->SR;
        (void)USARTx->DR;
    }
    /* RXNE automatically cleared on DR read → no manual action */
}


/**
 * @brief  Resets the specified USART peripheral and clears handle state.
 * @param  husart Pointer to a USART handle structure.
 * @retval None
 */
void USART_DeInit(USART_HandleTypeDef_t *husart)
{
    if ((husart == NULL) || (husart->Instance == NULL))
    {
        return;
    }

    /* Hardware reset via RCC */
    if (husart->Instance == USART1)
    {
        SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
        CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART1RST);
    }
    else if (husart->Instance == USART6)
    {
        SET_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART6RST);
        CLEAR_BIT(RCC->APB2RSTR, RCC_APB2RSTR_USART6RST);
    }
    else if (husart->Instance == USART2)
    {
        SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART2RST);
        CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART2RST);
    }
    else if (husart->Instance == USART3)
    {
        SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART3RST);
        CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_USART3RST);
    }
    else if (husart->Instance == UART4)
    {
        SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_UART4RST);
        CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_UART4RST);
    }
    else if (husart->Instance == UART5)
    {
        SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_UART5RST);
        CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_UART5RST);
    }

    /* Reset software handle state */
    husart->State      = STATE_RESET;
    husart->LastError  = ERROR_NONE;
    husart->TxBuffer   = NULL;
    husart->RxBuffer   = NULL;
    husart->TxLength   = 0U;
    husart->RxLength   = 0U;
}
