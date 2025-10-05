/**
 * @file    exti.c
 * @brief   External Interrupt (EXTI) driver implementation for STM32F407
 * @author  Hasan Erol
 */

#include "EXTI.h"

/* ========================================================================================= */
/*                                EXTI Line Configuration                                   */
/* ========================================================================================= */

/**
 * @brief  Configure EXTI line mapping to a GPIO port
 * @param  PortSource  GPIO port source (e.g. @ref EXTI_PORTSOURCE_GPIOA)
 * @param  LineSource  EXTI line number (e.g. @ref EXTI_LineSource_0)
 * @retval None
 */
void EXTI_LineConfig(exti_port_t PortSource, exti_line_t LineSource)
{
    uint32_t tempValue = SYSCFG->EXTI_CR[LineSource >> 2U];
    tempValue &= ~(0xFU << ((LineSource & 0x3U) * 4U));
    tempValue |= (PortSource << ((LineSource & 0x3U) * 4U));
    SYSCFG->EXTI_CR[LineSource >> 2U] = tempValue;
}


/* ========================================================================================= */
/*                                EXTI Initialization                                        */
/* ========================================================================================= */

/**
 * @brief  Initialize EXTI line for a given GPIO and trigger configuration
 * @param  EXTI_InitStruct Pointer to configuration structure
 * @retval None
 */
void EXTI_Init(EXTI_InitTypeDef_t *EXTI_InitStruct)
{
    if (EXTI_InitStruct == NULL)
        return;

    uint32_t line = EXTI_InitStruct->LineNumber;

    /* Disable interrupt and event masks first */
    EXTI->IMR &= ~(1U << line);
    EXTI->EMR &= ~(1U << line);

    /* Configure mode (Interrupt or Event) */
    if (EXTI_InitStruct->LineCmd == ENABLE)
    {
        if (EXTI_InitStruct->Mode == EXTI_MODE_INTERRUPT)
            EXTI->IMR |= (1U << line);
        else if (EXTI_InitStruct->Mode == EXTI_MODE_EVENT)
            EXTI->EMR |= (1U << line);

        /* Clear rising/falling configuration */
        EXTI->RTSR &= ~(1U << line);
        EXTI->FTSR &= ~(1U << line);

        /* Configure trigger selection */
        if (EXTI_InitStruct->TriggerSelect == EXTI_TRIGGER_RISING)
        {
            EXTI->RTSR |= (1U << line);
        }
        else if (EXTI_InitStruct->TriggerSelect == EXTI_TRIGGER_FALLING)
        {
            EXTI->FTSR |= (1U << line);
        }
        else if (EXTI_InitStruct->TriggerSelect == EXTI_TRIGGER_BOTH)
        {
            EXTI->RTSR |= (1U << line);
            EXTI->FTSR |= (1U << line);
        }
    }
    else
    {
        /* Disable line completely */
        EXTI->IMR &= ~(1U << line);
        EXTI->EMR &= ~(1U << line);
    }
}


/* ========================================================================================= */
/*                                NVIC Configuration                                         */
/* ========================================================================================= */

/**
 * @brief  Enables a given interrupt line in NVIC.
 * @param  IRQNumber: IRQ number corresponding to the interrupt line.
 * @retval None
 */
void NVIC_EnableInterrupt(EXTI_IRQNumber_t IRQNumber)
{
    uint32_t tempValue = 0U;

    /* Get current ISER register value for the IRQ group */
    tempValue = *(((uint32_t *)(NVIC_ISER0)) + (IRQNumber >> 5U));

    /* Clear and set the specific bit */
    tempValue &= ~(1U << (IRQNumber & 0x1FU));
    tempValue |=  (1U << (IRQNumber & 0x1FU));

    /* Write back to ISER register to enable interrupt */
    *(((uint32_t *)(NVIC_ISER0)) + (IRQNumber >> 5U)) = tempValue;
}
