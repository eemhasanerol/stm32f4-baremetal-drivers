/**
 * @file    gpio.c
 * @brief   GPIO driver implementation for STM32F407
 * @author  Hasan Erol
 */

#include "gpio.h"

/**
 * @brief  Sets or resets the selected GPIO output pin
 * @param  GPIOx GPIO port base address
 * @param  Pin Specifies the GPIO pin(s) to be written
 * @param  PinState Desired pin state (GPIO_PIN_SET or GPIO_PIN_RESET)
 */
void GPIO_WritePin(GPIO_TypeDef_t *GPIOx, GPIO_Pin_t Pin, GPIO_PinState_t PinState)
{
    if (PinState == GPIO_PIN_SET)
    {
        GPIOx->BSRR = (uint32_t)Pin;           /* Set the selected pin */
    }
    else
    {
        GPIOx->BSRR = ((uint32_t)Pin << 16U);  /* Reset the selected pin */
    }
}


/**
 * @brief  Reads the logic level of the specified GPIO input pin
 * @param  GPIOx GPIO port base address
 * @param  Pin Specifies the GPIO pin to read
 * @retval GPIO_PinState_t
 */
GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx, GPIO_Pin_t Pin)
{
    GPIO_PinState_t bitStatus = GPIO_PIN_RESET;

    if ((GPIOx->IDR & (uint32_t)Pin) != 0U)
    {
        bitStatus = GPIO_PIN_SET;
    }

    return bitStatus;
}


/**
 * @brief  Initializes the specified GPIO pin(s) according to the configuration structure
 * @param  GPIOx GPIO port base address
 * @param  GPIO_ConfigStruct Pointer to GPIO configuration structure
 */
void GPIO_LockPin(GPIO_TypeDef_t *GPIOx, GPIO_Pin_t Pin)
{
    uint32_t tempValue = (1U << 16U) | (uint32_t)Pin;

    GPIOx->LCKR = tempValue;      /* Write 1 + pin mask */
    GPIOx->LCKR = (uint32_t)Pin;  /* Write 0 + pin mask */
    GPIOx->LCKR = tempValue;      /* Write 1 + pin mask again */
    tempValue = GPIOx->LCKR;      /* Read to confirm lock sequence */
    (void)tempValue;
}


/**
 * @brief GPIO_Init, Configures the port and pin
 * @param GPIOx = GPIO Port Base Address
 * @param GPIO_InitTypeDef_t = User config structures
 * @retval Void
 */
void GPIO_Init(GPIO_TypeDef_t *GPIOx, GPIO_InitTypeDef_t *GPIO_ConfigStruct)
{
	uint32_t position;

	for(position = 0U; position < 16U ; position++)
	{
        if ((GPIO_ConfigStruct->Pin & (1U << position)) != 0U)
		{

            /* Configure mode */
			uint32_t tempReg = GPIOx->MODER;
			tempReg &= ~(0x03U << (position * 2U));
            tempReg |= ((uint32_t)GPIO_ConfigStruct->Mode << (position * 2U));
			GPIOx->MODER= tempReg;


			if((GPIO_ConfigStruct->Mode == GPIO_MODE_OUTPUT) || (GPIO_ConfigStruct->Mode == GPIO_MODE_AF))
			{
				/* Speed Config */
				tempReg = GPIOx->OSPEEDR;
				tempReg &= ~(0x03U << (position * 2U));
                tempReg |= ((uint32_t)GPIO_ConfigStruct->Speed << (position * 2U));
				GPIOx->OSPEEDR = tempReg;

				/* Output Type Config */
				tempReg = GPIOx->OTYPER;
				tempReg &= ~(1U << position);
                tempReg |= ((uint32_t)GPIO_ConfigStruct->OType << position);
				GPIOx->OTYPER = tempReg;
			}

            /* Configure pull-up/pull-down */
			tempReg = GPIOx->PUPDR;
			tempReg &= ~(0x03U << (position * 2U));
            tempReg |= ((uint32_t)GPIO_ConfigStruct->PuPd << (position * 2U));
			GPIOx->PUPDR = tempReg;

            /* Configure alternate function */
			if(GPIO_ConfigStruct->Mode == GPIO_MODE_AF)
			{
				uint8_t temp1, temp2;

				temp1 = position / 8;
				temp2 = position % 8;

				GPIOx->AFR[temp1] &= ~(0xF << (4U * temp2) );
				GPIOx->AFR[temp1] |=  ((GPIO_ConfigStruct->Alternate & 0xFU) << (4U * temp2) );
			}
		}
	}
}


/**
 * @brief  Toggles the state of the specified GPIO pin(s)
 * @param  GPIOx GPIO port base address
 * @param  Pin Specifies the GPIO pin(s) to toggle (0–15)
 */
void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx, GPIO_Pin_t Pin)
{
    uint32_t odrState = GPIOx->ODR;
    GPIOx->BSRR = ((odrState & (uint32_t)Pin) << 16U) | (~odrState & (uint32_t)Pin);
}


/**
 * @brief  Deinitializes the specified GPIO peripheral registers.
 * @note   This function resets the GPIO registers to their default values
 *         using the corresponding RCC AHB1 peripheral reset bit.
 * @param  GPIOx: Pointer to the GPIO port base (e.g. GPIOA, GPIOB, ...)
 * @retval None
 */
void GPIO_DeInit(GPIO_TypeDef_t *GPIOx)
{
    if (GPIOx == GPIOA)
    {
        SET_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOARST);
        CLEAR_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOARST);
    }
    else if (GPIOx == GPIOB)
    {
        SET_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOBRST);
        CLEAR_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOBRST);
    }
    else if (GPIOx == GPIOC)
    {
        SET_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOCRST);
        CLEAR_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOCRST);
    }
    else if (GPIOx == GPIOD)
    {
        SET_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIODRST);
        CLEAR_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIODRST);
    }
    else if (GPIOx == GPIOE)
    {
        SET_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOERST);
        CLEAR_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOERST);
    }
    else if (GPIOx == GPIOF)
    {
        SET_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOFRST);
        CLEAR_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOFRST);
    }
    else if (GPIOx == GPIOG)
    {
        SET_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOGRST);
        CLEAR_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOGRST);
    }
    else if (GPIOx == GPIOH)
    {
        SET_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOHRST);
        CLEAR_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOHRST);
    }
    else if (GPIOx == GPIOI)
    {
        SET_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOIRST);
        CLEAR_BIT(RCC->AHB1RSTR, RCC_AHB1RSTR_GPIOIRST);
    }
}
