/**
 * @file    systick.c
 * @brief   SysTick Timer Driver Source for STM32F407
 * @author  Hasan Erol
 */

#include "systick.h"

SYSTICK_HandleTypeDef_t hsystick;

volatile uint32_t systick_ms;

/**
 * @brief  Initialize SysTick timer.
 * @param  hsystick Pointer to handle structure.
 */
void SysTick_Init(SYSTICK_HandleTypeDef_t *hsystick)
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
void SysTick_Delay_ms(uint32_t ms)
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
