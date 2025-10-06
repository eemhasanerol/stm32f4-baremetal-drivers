/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Simple bare-metal LED blink example using SysTick delay.
 * @author         : Hasan Erol
 ******************************************************************************
 */

#include "stm32f407xx.h"
#include "rcc.h"
#include "gpio.h"
#include "systick.h"

/* Default system clock frequency (HSI = 16 MHz) */
uint32_t SystemCoreClock = 16000000U;

/* Function prototype */
void GPIO_LedInit(void);

/**
 * @brief  Main program.
 * @note   Alternately toggles LED pairs PD13–PD15 and PD12–PD14 every 500 ms.
 */
int main(void)
{
    /* Initialize SysTick for 1ms tick */
    SysTick_Init(1000);

    /* Initialize LEDs */
    GPIO_LedInit();

    /* Main loop */
    while(1)
    {
        GPIO_TogglePin(GPIOD, (GPIO_PIN_13 | GPIO_PIN_15));
        delay_ms(500);

        GPIO_TogglePin(GPIOD, (GPIO_PIN_12 | GPIO_PIN_14));
        delay_ms(500);
    }
}

/**
 * @brief  Configure PD13 and PD15 as push-pull outputs for LEDs.
 * @note   Ensures GPIO clock is enabled and LEDs start in OFF state.
 */
void GPIO_LedInit(void)
{
    GPIO_InitTypeDef_t hgpio_led = {0};

    /* GPIO clock enable */
    RCC_GPIOD_CLK_ENABLE();

    /* Pin configuration */
    hgpio_led.Pin   = (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
    hgpio_led.Mode  = GPIO_MODE_OUTPUT;
    hgpio_led.OType = GPIO_OTYPE_PP;
    hgpio_led.PuPd  = GPIO_NOPULL;
    hgpio_led.Speed = GPIO_SPEED_MEDIUM;

    /* Initialize GPIO */
    GPIO_Init(GPIOD, &hgpio_led);

    /* Start with LEDs OFF */
    GPIO_WritePin(GPIOD, (GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15), GPIO_PIN_RESET);
}
