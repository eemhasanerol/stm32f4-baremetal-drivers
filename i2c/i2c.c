	/*
 * i2c.c
 * I2C driver implementation (STM32F407) matching i2c.h
 * Created: Sep 1, 2025   Author: erol-tesla
 */

#include "I2C.h"

/* -------------------------------------------------------------------------- */
/* Local helpers (private)                                                    */
/* -------------------------------------------------------------------------- */

/** Generate START condition (sets CR1.START). */
static inline void I2C_GenerateStart(I2C_TypeDef_t *I2Cx)
{
	I2Cx->CR1 |= I2C_CR1_START_MSK;
}

/** Generate STOP condition (sets CR1.STOP). */
static inline void I2C_GenerateStop(I2C_TypeDef_t *I2Cx)
{
	I2Cx->CR1 |= I2C_CR1_STOP_MSK;
}

/** Send [Addr|W] into DR (7-bit addr). */
static inline void I2C_SendAddrWrite(I2C_TypeDef_t *I2Cx, uint8_t addr7)
{
    uint8_t v = (uint8_t)((addr7 << 1) & ~0x1U);
    I2Cx->DR = v;
}

/** Send [Addr|R] into DR (7-bit addr). */
static inline void I2C_SendAddrRead(I2C_TypeDef_t *I2Cx, uint8_t addr7)
{
    (void)I2Cx->SR1; /* clear SB by reading SR1 before writing DR in read phase (aligns with your flow) */
    uint8_t v = (uint8_t)((addr7 << 1) | 0x1U);
    I2Cx->DR = v;
}

/** Clear ADDR flag by SR1 then SR2 read sequence. */
static inline void I2C_ClearADDR(I2C_TypeDef_t *I2Cx)
{
	(void)I2Cx->SR1;
	(void)I2Cx->SR2;
}

/** Enable/disable hardware ACK bit. */
static inline void I2C_SetAck(I2C_TypeDef_t *I2Cx, i2c_ack_t en)
{
    if (en == I2C_ACK_ENABLE)
    {
    	I2Cx->CR1 |= I2C_CR1_ACK_MSK;
    }
    else
    {
    	I2Cx->CR1 &= ~I2C_CR1_ACK_MSK;
    }
}

/**
 * @brief  Get the status of the specified I2C flag.
 * @param  I2Cx      Pointer to I2C peripheral base
 * @param  FlagName  Flag to check (e.g. @ref I2C_FLAG_TXE)
 * @retval 1 if set, 0 otherwise
 */
static inline uint8_t I2C_GetFlag(I2C_TypeDef_t *I2Cx, uint32_t FlagName)
{
    return (uint8_t)((I2Cx->SR1 & FlagName) ? 1U : 0U);
}

/**
 * @brief  Wait until a specific flag is set or timeout occurs.
 * @param  hi2c  I2C handle pointer
 * @param  flag  Flag to monitor
 * @retval STATUS_OK if flag is set, STATUS_TIMEOUT otherwise
 */
static inline status_t I2C_WaitFlag(I2C_HandleTypeDef_t *hi2c, uint32_t flag)
{
    uint32_t start = SysTick_GetTick();
    while (!I2C_GetFlag(hi2c->Instance, flag))
    {
        if ((SysTick_GetTick() - start) >= hi2c->Init.TimeoutMs)
        {
            return STATUS_TIMEOUT;
        }
    }
    return STATUS_OK;
}

/* --------------------------------------------------------------------------- */
/* Blocking API                                                                */
/* These functions perform I2C transfers using polling and block CPU execution */
/* until the transfer is complete or a timeout occurs.                         */
/* --------------------------------------------------------------------------- */

/**
 * @brief  Initializes the I2C peripheral according to the specified parameters
 *         in the I2C handle structure.
 * @param  hi2c  Pointer to an I2C_HandleTypeDef_t structure that contains
 *               the configuration information for the specified I2C module.
 * @retval None
 * @note   This function:
 *         - Resets and configures the I2C peripheral registers.
 *         - Sets the clock frequency, duty cycle, and rise time.
 *         - Configures ACK control, own address, and enables the peripheral.
 */
void I2C_Init(I2C_HandleTypeDef_t *hi2c)
{
    if (!hi2c || !hi2c->Instance)
    {
    	return ;
    }

	if (hi2c->Init.TimeoutMs== 0U)
	{
		hi2c->Init.TimeoutMs = 1000U;
	}

    I2C_TypeDef_t *I2Cx = hi2c->Instance;

    /* Disable peripheral before configuration */
    I2Cx->CR1 &= ~I2C_CR1_PE_MSK;

    /* Software reset pulse */
    I2Cx->CR1 |=  I2C_CR1_SWRST_MSK;
    I2Cx->CR1 &= ~I2C_CR1_SWRST_MSK;

    uint32_t tempReg = 0U;

    /* Configure CR2.FREQ = PCLK1(MHz) */
    tempReg  = I2Cx->CR2;
    tempReg &= ~0x3FU;
    tempReg |= ((RCC_GetPCLK1Freq() / 1000000U) & 0x3FU);
    I2Cx->CR2 = tempReg;

    /* Configure TRISE */
    tempReg = 0U;
    if (hi2c->Init.ClockSpeedHz <= I2C_SPEED_STANDARD)
    {
        tempReg = (RCC_GetPCLK1Freq() / 1000000U) + 1U; /* standard mode */
    } else
    {
        tempReg = ( (RCC_GetPCLK1Freq() * 300U) / 1000000000U ) + 1U; /* fast mode */
    }
    I2Cx->TRISE = (tempReg & 0x3FU);

    /* Configure CCR */
    uint16_t ccr_value = 0U;
    tempReg = 0U;
    if (hi2c->Init.ClockSpeedHz <= I2C_SPEED_STANDARD)
    {
        ccr_value = (uint16_t)(RCC_GetPCLK1Freq() / (2U * hi2c->Init.ClockSpeedHz));
        tempReg  |= (ccr_value & 0xFFFU);
    } else
    {
        tempReg |= I2C_CCR_FS_MSK;
        tempReg |= ((uint32_t)hi2c->Init.DutyCycle << I2C_CCR_DUTY_POS);
        if (hi2c->Init.DutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = (uint16_t)(RCC_GetPCLK1Freq() / (3U  * hi2c->Init.ClockSpeedHz));
        } else
        {
            ccr_value = (uint16_t)(RCC_GetPCLK1Freq() / (25U * hi2c->Init.ClockSpeedHz));
        }
        tempReg |= (ccr_value & 0xFFFU);
    }
    I2Cx->CCR = tempReg;

    /* Configure ACK and No-Stretch */
    tempReg  = 0U;
    tempReg |= ((uint32_t)hi2c->Init.AckState << I2C_CR1_ACK_POS);
    I2Cx->CR1 |= tempReg;
    I2Cx->CR1 &= ~I2C_CR1_NOSTRETCH_MSK;

	// 7 program the device own address
	tempReg = 0;
	tempReg |= hi2c->DevAddress << 1U;
	tempReg |= (1U << 14U);
	I2Cx->OAR1 = tempReg;

    /* Enable peripheral */
    I2Cx->CR1 |= I2C_CR1_PE_MSK;

    hi2c->State = I2C_STATE_READY;
    hi2c->LastError = I2C_ERROR_NONE;
}


/**
 * @brief  Deinitializes the specified I2C peripheral and resets its software handle.
 * @param  hi2c Pointer to an I2C handle structure that contains
 *         the configuration information for the specified I2C module.
 * @retval None
 *
 * @note   Performs a hardware peripheral reset via RCC and clears the
 *         handle fields to their default values.
 */
void I2C_Deinit(I2C_HandleTypeDef_t *hi2c)
{
    if ((hi2c == NULL) || (hi2c->Instance == NULL))
        return ;

    /* Hardware reset via RCC */
    if (hi2c->Instance == I2C1)
    {
        SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_I2C1RST);
        CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_I2C1RST);
    }
    else if (hi2c->Instance == I2C2)
    {
        SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_I2C2RST);
        CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_I2C2RST);
    }
    else if (hi2c->Instance == I2C3)
    {
        SET_BIT(RCC->APB1RSTR, RCC_APB1RSTR_I2C3RST);
        CLEAR_BIT(RCC->APB1RSTR, RCC_APB1RSTR_I2C3RST);
    }

    /* Software cleanup */
    hi2c->State      = I2C_STATE_READY;
    hi2c->LastError  = I2C_ERROR_NONE;
    hi2c->TxBuffer   = NULL;
    hi2c->RxBuffer   = NULL;
    hi2c->TxLength   = 0U;
    hi2c->RxLength   = 0U;
}


/**
 * @brief  Transmits data in master mode (blocking).
 * @param  hi2c   Pointer to I2C handle structure.
 * @param  addr7  7-bit slave address.
 * @param  buf    Pointer to data buffer to transmit.
 * @param  len    Number of bytes to send.
 * @param  rs     Repeated start control (I2C_RS_ENABLE / I2C_RS_DISABLE).
 * @retval status_t Operation result (OK, TIMEOUT, PARAM).
 */
status_t I2C_MasterTransmit(I2C_HandleTypeDef_t *hi2c, uint8_t addr7, const uint8_t *buf, size_t len, i2c_rs_t rs)
{
    if (!hi2c || !hi2c->Instance || !buf || (len == 0U))
	{
    	return STATUS_PARAM;
	}

    I2C_TypeDef_t *I2Cx = hi2c->Instance;

    /* Generate START condition */
    I2C_GenerateStart(I2Cx);

    /* Wait for SB flag */
    if(I2C_WaitFlag(hi2c, I2C_FLAG_SB) != STATUS_OK){
    	return STATUS_TIMEOUT;
    }

    /* Send slave address with write bit */
    I2C_SendAddrWrite(I2Cx, addr7);

    /* Wait for ADDR flag and clear */
    if(I2C_WaitFlag(hi2c, I2C_FLAG_ADDR) != STATUS_OK){
    	return STATUS_TIMEOUT;
    }

    I2C_ClearADDR(I2Cx);

    /* Transmit data bytes */
    while (len > 0U)
    {
        if(I2C_WaitFlag(hi2c, I2C_FLAG_TXE) != STATUS_OK){
        	return STATUS_TIMEOUT;
        }

        I2Cx->DR = *(buf++);
        len--;
    }

    /* Wait for BTF flag */
    if(I2C_WaitFlag(hi2c, I2C_FLAG_BTF) != STATUS_OK){
    	return STATUS_TIMEOUT;
    }

    /* Generate STOP if no repeated start */
    if (rs == I2C_RS_DISABLE)
    {
        I2C_GenerateStop(I2Cx);
    }
    
    hi2c->State = I2C_STATE_READY;
    return STATUS_OK;
}


/**
 * @brief  Receive data from an I2C slave device in master mode (blocking).
 * @param  hi2c   Pointer to I2C handle structure.
 * @param  addr7  7-bit slave address.
 * @param  buf    Pointer to data buffer to store received bytes.
 * @param  len    Number of bytes to receive.
 * @param  rs     Repeated start option (I2C_RS_ENABLE or I2C_RS_DISABLE).
 * @retval STATUS_OK       If transmission completed successfully.
 * @retval status_t Operation result (OK, TIMEOUT, PARAM).
 */
status_t I2C_MasterReceive(I2C_HandleTypeDef_t *hi2c, uint8_t addr7, uint8_t *buf, size_t len, i2c_rs_t rs)
{
    if (!hi2c || !hi2c->Instance || !buf || (len == 0U))
	{
    	return STATUS_PARAM;
	}

    I2C_TypeDef_t *I2Cx = hi2c->Instance;

    /* Generate START condition */
    I2C_GenerateStart(I2Cx);

    /* Wait for SB flag then send slave address with read bit */
    if(I2C_WaitFlag(hi2c, I2C_FLAG_SB) != STATUS_OK){
    	return STATUS_TIMEOUT;
    }

    (void)I2Cx->SR1;                 /* Clear SB flag */
    I2C_SendAddrRead(I2Cx, addr7);

    /* Wait for ADDR flag */
    if(I2C_WaitFlag(hi2c, I2C_FLAG_ADDR) != STATUS_OK){
    	return STATUS_TIMEOUT;
    }
    if (len == 1U)
    {
        /* Single-byte reception */
        I2C_SetAck(I2Cx, I2C_ACK_DISABLE);
        I2C_ClearADDR(I2Cx);

        if(I2C_WaitFlag(hi2c, I2C_FLAG_RXNE) != STATUS_OK){
        	return STATUS_TIMEOUT;
        }

        /* Generate STOP if no repeated start */
        if (rs == I2C_RS_DISABLE)
        {
        	I2C_GenerateStop(I2Cx);
        }

        *buf = I2Cx->DR;
    } else if (len == 2U)
    {
        /* Two-byte reception */
        I2Cx->CR1 |= I2C_CR1_POS_MSK;
        I2C_SetAck(I2Cx, I2C_ACK_DISABLE);
        I2C_ClearADDR(I2Cx);

        /* Wait for BTF flag */
        if(I2C_WaitFlag(hi2c, I2C_FLAG_BTF) != STATUS_OK){
        	return STATUS_TIMEOUT;
        }

        /* Generate STOP if no repeated start */
        if (rs == I2C_RS_DISABLE)
        {
        	I2C_GenerateStop(I2Cx);
        }

        *(buf++) = I2Cx->DR;
        *(buf++) = I2Cx->DR;
        I2Cx->CR1 &= ~I2C_CR1_POS_MSK;
    } else
    {
        /* Multi-byte reception (>2 bytes) */
        size_t remaining = len;
        I2C_SetAck(I2Cx, I2C_ACK_ENABLE);
        I2C_ClearADDR(I2Cx);

        /* Receive until 3 bytes remain */
        while (remaining > 3U)
        {
            if(I2C_WaitFlag(hi2c, I2C_FLAG_RXNE) != STATUS_OK){
            	return STATUS_TIMEOUT;
            }

            *(buf++) = I2Cx->DR;
            remaining--;
        }

        /* Handle final 3-byte sequence */
        if(I2C_WaitFlag(hi2c, I2C_FLAG_BTF) != STATUS_OK){
        	return STATUS_TIMEOUT;
        }

        I2C_SetAck(I2Cx, I2C_ACK_DISABLE); /* Prepare NACK */
        *(buf++) = I2Cx->DR;               /* N-2 byte */

        if(I2C_WaitFlag(hi2c, I2C_FLAG_BTF) != STATUS_OK){
        	return STATUS_TIMEOUT;
        }
        if (rs == I2C_RS_DISABLE)
        {
        	I2C_GenerateStop(I2Cx);
        }
        *(buf++) = I2Cx->DR;     /* N-1 byte */
        *(buf++) = I2Cx->DR;     /* N byte */
    }

    hi2c->State = I2C_STATE_READY;
    return STATUS_OK;
}

/**
 * @brief  Write data to a specific register of an I2C device (memory write).
 * @param  hi2c   Pointer to I2C handle structure.
 * @param  addr7  7-bit I2C slave address.
 * @param  reg    Register/memory address to write to.
 * @param  data   Pointer to data buffer to be transmitted.
 * @param  len    Number of bytes to write.
 * @retval status_t Operation result (OK, TIMEOUT, PARAM).
 */
status_t I2C_MemWrite(I2C_HandleTypeDef_t *hi2c, uint8_t addr7, uint8_t reg, const uint8_t *data, size_t len)
{
    if (!hi2c || !hi2c->Instance || !data || (len == 0U)){
        return STATUS_PARAM;
    }

    uint8_t tx[len + 1];
    tx[0] = reg;

    for (size_t i = 0; i < len; i++)
    {
        tx[i + 1] = data[i];
    }

    return I2C_MasterTransmit(hi2c, addr7, tx, (len + 1U), I2C_RS_DISABLE);
}

/**
 * @brief  Read data from a specific memory/register address of an I2C slave device.
 * @param  hi2c   Pointer to I2C handle structure.
 * @param  addr7  7-bit slave address.
 * @param  reg    Register (memory) address to read from.
 * @param  data   Pointer to buffer to store read data.
 * @param  len    Number of bytes to read.
 * @retval STATUS_OK on success, or error code (STATUS_TIMEOUT, STATUS_PARAM, etc.)
 */
status_t I2C_MemRead(I2C_HandleTypeDef_t *hi2c, uint8_t addr7, uint8_t reg, uint8_t *data, size_t len)
{
    if (!hi2c || !hi2c->Instance || !data || (len == 0U))
	{
    	return STATUS_PARAM;
	}


    /* Write register, repeated start, then read (keeps your flow) */
    status_t status;
    status = I2C_MasterTransmit(hi2c, addr7, &reg, 1U, I2C_RS_ENABLE);
    if (status != STATUS_OK)
	{
    	return status;
	}

    status = I2C_MasterReceive(hi2c, addr7, data, len, I2C_RS_DISABLE);
    if(status != STATUS_OK){
    	return status;
    }

    return STATUS_OK;
}
/* -------------------------------------------------------------------------- */
/* Interrupt / Non-blocking API                                               */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initiates an interrupt-driven I2C master transmission.
 * @param  hi2c   Pointer to I2C handle structure.
 * @param  addr7  7-bit slave address.
 * @param  buf    Pointer to data buffer to transmit.
 * @param  len    Number of bytes to transmit.
 * @param  rs     Repeated start option (@ref i2c_rs_t).
 * @retval STATUS_OK on success, or error code (STATUS_TIMEOUT, STATUS_PARAM, etc.)
 */
status_t I2C_MasterTransmitIT(I2C_HandleTypeDef_t *hi2c, uint8_t addr7, const uint8_t *buf, size_t len, i2c_rs_t rs)
{
    if (!hi2c || !hi2c->Instance || !buf || (len == 0U))
	{
    	return STATUS_PARAM;
	}

    if ( (hi2c->State == I2C_STATE_BUSY_TX) || (hi2c->State == I2C_STATE_BUSY_RX) )
	{
    	return STATUS_BUSY;
	}

    hi2c->TxBuffer = buf;
    hi2c->TxLength = len;
    hi2c->RxBuffer = NULL;
    hi2c->RxLength = 0U;
    hi2c->RepeatedStart = rs;
    hi2c->State = I2C_STATE_BUSY_TX;

    I2C_GenerateStart(hi2c->Instance);

    /* Enable I2C event, buffer, and error interrupts */
    hi2c->Instance->CR2 |= (I2C_CR2_ITEVTEN_MSK | I2C_CR2_ITBUFEN_MSK | I2C_CR2_ITERREN_MSK);

    return STATUS_OK;
}

/**
 * @brief  Initiates an interrupt-driven I2C master reception.
 * @param  hi2c   Pointer to I2C handle structure.
 * @param  addr7  7-bit slave address.
 * @param  buf    Pointer to buffer for received data.
 * @param  len    Number of bytes to receive.
 * @param  rs     Repeated start option (@ref i2c_rs_t).
 * @retval STATUS_OK on success, or error code (STATUS_TIMEOUT, STATUS_PARAM, etc.)
 */
status_t I2C_MasterReceiveIT(I2C_HandleTypeDef_t *hi2c, uint8_t addr7, uint8_t *buf, size_t len, i2c_rs_t rs)
{
    if (!hi2c || !hi2c->Instance || !buf || (len == 0U))
	{
    	return STATUS_PARAM;
	}

    if ( (hi2c->State == I2C_STATE_BUSY_TX) || (hi2c->State == I2C_STATE_BUSY_RX) )
    {
    	return STATUS_BUSY;
    }

    hi2c->RxBuffer = buf;
    hi2c->RxLength = len;
    hi2c->TxBuffer = NULL;
    hi2c->TxLength = 0U;
    hi2c->RepeatedStart = rs;
    hi2c->State = I2C_STATE_BUSY_RX;

    hi2c->Instance->CR1 &= ~I2C_CR1_POS_MSK;

    I2C_GenerateStart(hi2c->Instance);

    /* Enable I2C event, buffer, and error interrupts */
    hi2c->Instance->CR2 |= (I2C_CR2_ITEVTEN_MSK | I2C_CR2_ITBUFEN_MSK | I2C_CR2_ITERREN_MSK);

    return STATUS_OK;
}
/**
 * @brief  Closes the I2C master transmit (interrupt mode).
 *         Disables I2C transmit interrupts, resets TX state, and clears buffer pointers.
 * @param  hi2c  Pointer to I2C handle structure.
 * @retval None
 */
void I2C_CloseTransmitIT(I2C_HandleTypeDef_t *hi2c)
{
    if (!hi2c || !hi2c->Instance)
        return;

    /* Disable event and buffer interrupts */
    hi2c->Instance->CR2 &= ~(I2C_CR2_ITEVTEN_MSK | I2C_CR2_ITBUFEN_MSK);


    /* Reset transmit state */
    hi2c->State    = I2C_STATE_READY;
    hi2c->TxBuffer = NULL;
    hi2c->TxLength = 0U;
}


/**
 * @brief  Closes the I2C master receive (interrupt mode).
 *         Disables I2C receive interrupts, resets RX state, and restores ACK if enabled.
 * @param  hi2c  Pointer to I2C handle structure.
 * @retval None
 */
void I2C_CloseReceiveIT(I2C_HandleTypeDef_t *hi2c)
{
    if (!hi2c || !hi2c->Instance)
        return;

    /* Disable event and buffer interrupts */
    hi2c->Instance->CR2 &= ~(I2C_CR2_ITEVTEN_MSK | I2C_CR2_ITBUFEN_MSK);

    /* Reset receive state */
    hi2c->State    = I2C_STATE_READY;
    hi2c->RxBuffer = NULL;
    hi2c->RxLength = 0U;

    /* Re-enable ACK if configured */
    if (hi2c->Init.AckState == I2C_ACK_ENABLE)
    {
        I2C_SetAck(hi2c->Instance, I2C_ACK_ENABLE);
    }
}
/* -------------------------------------------------------------------------- */
/* IRQ Control                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Enables or disables the specified IRQ in NVIC.
 * @param  IRQNumber: Interrupt request number (see vector table index)
 * @param  enable: 1U to enable, 0U to disable
 */
void I2C_IRQEnable(uint8_t IRQNumber, uint8_t enable)
{
    if (enable)
    {
        if (IRQNumber < 32U)
            *NVIC_ISER0 = (1U << IRQNumber);
        else if (IRQNumber < 64U)
            *NVIC_ISER1 = (1U << (IRQNumber % 32U));
        else if (IRQNumber < 96U)
            *NVIC_ISER2 = (1U << (IRQNumber % 64U));  /* düzeltme: ISER3 → ISER2 */
    }
    else
    {
        if (IRQNumber < 32U)
            *NVIC_ICER0 = (1U << IRQNumber);
        else if (IRQNumber < 64U)
            *NVIC_ICER1 = (1U << (IRQNumber % 32U));
        else if (IRQNumber < 96U)
            *NVIC_ICER2 = (1U << (IRQNumber % 64U));  /* düzeltme: ICER3 → ICER2 */
    }
}

/**
 * @brief  Sets the priority of a given IRQ line in NVIC.
 * @param  IRQNumber: Interrupt number
 * @param  priority: Priority level (0 = highest)
 */
void I2C_IRQSetPriority(uint8_t IRQNumber, uint32_t priority)
{
    uint8_t iprx         = (uint8_t)(IRQNumber / 4U);
    uint8_t iprx_section = (uint8_t)(IRQNumber % 4U);
    uint8_t shift_amount = (uint8_t)((8U * iprx_section) + (8U - NO_PR_BITS_IMPLEMENTED));

    /* clear önceki değer */
    *(NVIC_PR_BASE_ADDR + iprx) &= ~(0xFFU << (8U * iprx_section));
    *(NVIC_PR_BASE_ADDR + iprx) |= (priority << shift_amount);
}

/* -------------------------------------------------------------------------- */
/* ISR handlers                                                               */
/* -------------------------------------------------------------------------- */
/**
 * @brief  I2C event interrupt handler.
 * @param  hi2c: Pointer to I2C handle structure.
 * @note   Handles TXE, RXNE, BTF, ADDR, SB and event flags for master mode.
 */
void I2C_EV_IRQHandler(I2C_HandleTypeDef_t *hi2c)
{
    if ((hi2c == NULL) || (hi2c->Instance == NULL))
        return;

    I2C_TypeDef_t *I2Cx = hi2c->Instance;
    uint32_t sr1 = I2Cx->SR1;
    uint32_t sr2 = I2Cx->SR2;  /* Read SR2 once to latch flags (avoid stale states) */
    (void)sr2;

    /* ----------------------------- Master Transmit ----------------------------- */
    if (hi2c->State == I2C_STATE_BUSY_TX)
    {
        /* ADDR event: clear flag */
        if (sr1 & I2C_SR1_ADDR_MSK)
            I2C_ClearADDR(I2Cx);

        /* TXE event: send next byte */
        if (sr1 & I2C_SR1_TXE_MSK)
        {
            if (hi2c->TxLength > 0U)
            {
                I2Cx->DR = *(hi2c->TxBuffer++);
                hi2c->TxLength--;
            }
        }

        /* BTF event: transfer complete */
        if ((hi2c->TxLength == 0U) && (sr1 & I2C_SR1_BTF_MSK))
        {
            if (hi2c->RepeatedStart == I2C_RS_DISABLE)
                I2C_GenerateStop(I2Cx);

            I2C_CloseTransmitIT(hi2c);
            I2C_ApplicationEventCallback(hi2c, I2C_EVENT_TX_CMPLT, I2C_ERROR_NONE);
        }
    }

    /* ----------------------------- Master Receive ------------------------------ */
    else if (hi2c->State == I2C_STATE_BUSY_RX)
    {
        /* ADDR event */
        if (sr1 & I2C_SR1_ADDR_MSK)
        {
            if (hi2c->RxLength == 1U)
            {
                I2C_SetAck(I2Cx, I2C_ACK_DISABLE);
                I2C_ClearADDR(I2Cx);
                if (hi2c->RepeatedStart == I2C_RS_DISABLE)
                    I2C_GenerateStop(I2Cx);
            }
            else if (hi2c->RxLength == 2U)
            {
                I2Cx->CR1 |= I2C_CR1_POS_MSK;
                I2C_SetAck(I2Cx, I2C_ACK_DISABLE);
                I2C_ClearADDR(I2Cx);
            }
            else
            {
                I2C_SetAck(I2Cx, I2C_ACK_ENABLE);
                I2C_ClearADDR(I2Cx);
            }
        }

        /* RXNE event: read data */
        if (sr1 & I2C_SR1_RXNE_MSK)
        {
            *(hi2c->RxBuffer++) = I2Cx->DR;
            hi2c->RxLength--;

            if (hi2c->RxLength == 0U)
            {
                I2C_CloseReceiveIT(hi2c);
                I2C_ApplicationEventCallback(hi2c, I2C_EVENT_RX_CMPLT, I2C_ERROR_NONE);
            }
        }

        /* BTF event: handle 2-byte remaining sequence */
        if (sr1 & I2C_SR1_BTF_MSK)
        {
            if (hi2c->RxLength == 2U)
            {
                if (hi2c->RepeatedStart == I2C_RS_DISABLE)
                    I2C_GenerateStop(I2Cx);

                *(hi2c->RxBuffer++) = I2Cx->DR;
                *(hi2c->RxBuffer++) = I2Cx->DR;
                hi2c->RxLength = 0U;

                I2Cx->CR1 &= ~I2C_CR1_POS_MSK;
                I2C_CloseReceiveIT(hi2c);
                I2C_ApplicationEventCallback(hi2c, I2C_EVENT_RX_CMPLT, I2C_ERROR_NONE);
            }
        }
    }
}

/**
 * @brief  I2C error interrupt handler.
 * @param  hi2c: Pointer to I2C handle structure.
 * @note   Clears error flags (BERR, ARLO, AF, OVR, TIMEOUT) and reports them
 *         via user callback. Does not automatically reset peripheral.
 */
void I2C_ER_IRQHandler(I2C_HandleTypeDef_t *hi2c)
{
    if ((hi2c == NULL) || (hi2c->Instance == NULL))
        return;

    I2C_TypeDef_t *I2Cx = hi2c->Instance;

    /* Check if error interrupts are enabled */
    if (!(I2Cx->CR2 & I2C_CR2_ITERREN_MSK)){
        return;
    }

    /* ---------------- Bus Error ---------------- */
    if (I2Cx->SR1 & I2C_SR1_BERR_MSK)
    {
        I2Cx->SR1 &= ~I2C_SR1_BERR_MSK;
        hi2c->LastError = I2C_ERROR_BERR;
        I2C_ApplicationEventCallback(hi2c, I2C_EVENT_STOP, I2C_ERROR_BERR);
    }

    /* ---------------- Arbitration Lost ---------------- */
    if (I2Cx->SR1 & I2C_SR1_ARLO_MSK)
    {
        I2Cx->SR1 &= ~I2C_SR1_ARLO_MSK;
        hi2c->LastError = I2C_ERROR_ARLO;
        I2C_ApplicationEventCallback(hi2c, I2C_EVENT_STOP, I2C_ERROR_ARLO);
    }

    /* ---------------- Acknowledge Failure ---------------- */
    if (I2Cx->SR1 & I2C_SR1_AF_MSK)
    {
        I2Cx->SR1 &= ~I2C_SR1_AF_MSK;
        hi2c->LastError = I2C_ERROR_AF;
        I2C_ApplicationEventCallback(hi2c, I2C_EVENT_STOP, I2C_ERROR_AF);
    }

    /* ---------------- Overrun / Underrun ---------------- */
    if (I2Cx->SR1 & I2C_SR1_OVR_MSK)
    {
        I2Cx->SR1 &= ~I2C_SR1_OVR_MSK;
        hi2c->LastError = I2C_ERROR_OVR;
        I2C_ApplicationEventCallback(hi2c, I2C_EVENT_STOP, I2C_ERROR_OVR);
    }

    /* ---------------- Timeout Error ---------------- */
    if (I2Cx->SR1 & I2C_SR1_TIMEOUT_MSK)
    {
        I2Cx->SR1 &= ~I2C_SR1_TIMEOUT_MSK;
        hi2c->LastError = I2C_ERROR_TIMEOUT;
        I2C_ApplicationEventCallback(hi2c, I2C_EVENT_STOP, I2C_ERROR_TIMEOUT);
    }
}
