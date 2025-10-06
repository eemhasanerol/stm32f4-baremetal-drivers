/**
 * @file    systick.c
 * @brief   SysTick Timer Driver Source for STM32F407
 * @author  Hasan Erol
 */

#include "systick.h"

SYSTICK_HandleTypeDef_t hsystick;

volatile uint32_t systick_ms;

/**
 * @brief  Initialize SysTick timer with default configuration.
 * @param  tick_hz Tick frequency in Hertz (e.g. 1000 for 1ms tick).
 * @retval None
 * @note   Uses AHB clock as source and fixed interrupt priority 0.
 */
void SysTick_Init(uint32_t tick_hz)
{
    if (tick_hz == 0U)
        return;

    /* Stop & reset */
    SysTick->CTRL = 0;
    systick_ms = 0U;

    /* Clock source: AHB */
    SysTick->CTRL |= SYSTICK_CTRL_CLKSOURCE;

    /* Reload calculation */
    uint32_t reload = SystemCoreClock / tick_hz;
    SysTick->LOAD = reload - 1U;
    SysTick->VAL  = 0U;

    /* Interrupt priority (optional, sabit 0 verilebilir) */
    uint32_t tmp = SCB_SHPR3;
    tmp &= ~(0xFFUL << 24U);
    tmp |= (0x0U << 24U);
    SCB_SHPR3 = tmp;

    /* Enable interrupt & counter */
    SysTick->CTRL |= SYSTICK_CTRL_TICKINT | SYSTICK_CTRL_ENABLE;
}


/**
 * @brief  Initialize SysTick timer with advanced configuration.
 * @param  hsystick Pointer to SYSTICK_HandleTypeDef_t structure containing
 *         configuration parameters (clock source, tick frequency, interrupt usage, priority).
 * @retval None
 * @note   Supports both AHB and AHB/8 clock sources and configurable NVIC priority.
 */
void SysTick_InitEx(SYSTICK_HandleTypeDef_t *hsystick)
{
    if (!hsystick || hsystick->tick_hz == 0U)
	{
    	return;
	}

    /* stop & reset */
    SysTick->CTRL &= ~SYSTICK_CTRL_ENABLE;
    systick_ms = 0U;

    /* clock source */
    if (hsystick->clksource == SYSTICK_CLKSRC_AHB)
        SysTick->CTRL |=  SYSTICK_CTRL_CLKSOURCE;
    else
        SysTick->CTRL &= ~SYSTICK_CTRL_CLKSOURCE;

    /* reload */
    uint32_t pres = (hsystick->clksource == SYSTICK_CLKSRC_AHB) ? 1U : 8U;
    uint32_t reload = SystemCoreClock / pres / hsystick->tick_hz;
    if (reload == 0U){
    	reload = 1U;
    }

    if (reload > SYSTICK_LOAD_MAX) {
    	return;
    }
    SysTick->LOAD = reload - 1U;
    SysTick->VAL  = 0U;

    uint32_t tmp = SCB_SHPR3;
    tmp &= ~(0xFFUL << 24U);
    tmp |= ((hsystick->nvic_priority & 0xF) << 28U);
    SCB_SHPR3 = tmp;

    /* interrupt (opsiyonel) */
    if (hsystick->use_interrupt) {
        SysTick->CTRL |= SYSTICK_CTRL_TICKINT;
    } else {
        SysTick->CTRL &= ~SYSTICK_CTRL_TICKINT;
    }

    /* start */
    SysTick->CTRL |= SYSTICK_CTRL_ENABLE;
}


void SysTick_Handler(void)
{
	systick_ms++;
}

/**
 * @brief  Get current SysTick tick count.
 * @retval Tick count in milliseconds.
 */
uint32_t SysTick_GetTick(void)
{
	return systick_ms;
}


/**
 * @brief  Blocking delay function.
 * @param  ms Delay duration in milliseconds.
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = systick_ms;
    while ((uint32_t)(systick_ms - start) < ms);
}


/**
 * @brief  Check if specified time interval has elapsed.
 * @param  ms Interval in milliseconds.
 * @retval true if elapsed, false otherwise.
 */
bool SysTick_Elapsed(uint32_t ms)
{
	static uint32_t start_ms = 0;

	if((systick_ms - start_ms) >= ms )
	{
		start_ms = systick_ms;
		return 1;
	}

	return 0;
}
