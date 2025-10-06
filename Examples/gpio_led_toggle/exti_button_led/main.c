/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : EXTI button interrupt example — toggles LED on PD13 when user button pressed.
 ******************************************************************************
 */


#include "stm32f407xx.h"
#include "rcc.h"
#include "gpio.h"
#include "exti.h"
#include "systick.h"

/* Default system clock frequency (HSI = 16 MHz) */
uint32_t SystemCoreClock = 16000000U;

/* Function prototypes */
void GPIO_ButtonInit(void);
void GPIO_LedInit(void);
void EXTI0_IRQHandler(void);

int main(void)
{
    SysTick_Init(1000);  // 1 ms tick

    GPIO_LedInit();
    GPIO_ButtonInit();

    while (1)
    {
        /* Main loop — everything handled by interrupt */
    }
}

/**
 * @brief Initialize User Button (PA0) as EXTI input
 */
void GPIO_ButtonInit(void)
{
    GPIO_InitTypeDef_t button = {0};

    /* Enable GPIOA clock */
    RCC_GPIOA_CLK_ENABLE();

    /* Configure PA0 as input (User Button on STM32F4 Discovery) */
    button.Pin   = GPIO_PIN_0;
    button.Mode  = GPIO_MODE_INPUT;
    button.PuPd  = GPIO_NOPULL;
    GPIO_Init(GPIOA, &button);

    /* Configure EXTI line for PA0 */
    RCC_SYSCFG_CLK_ENABLE();
    EXTI_LineConfig(EXTI_PORT_A, EXTI_LINE_0);

    /* Configure EXTI settings */
    EXTI_InitTypeDef_t exti = {0};
    exti.LineNumber     = EXTI_LINE_0;
    exti.Mode           = EXTI_MODE_INTERRUPT;
    exti.TriggerSelect  = EXTI_TRIGGER_FALLING;
    exti.LineCmd        = ENABLE;
    EXTI_Init(&exti);

    /* Enable NVIC interrupt for EXTI0 */
    NVIC_EnableInterrupt(EXTI0_IRQNumber);
}

/**
 * @brief Initialize LED (PD13)
 */
void GPIO_LedInit(void)
{
    GPIO_InitTypeDef_t led = {0};

    RCC_GPIOD_CLK_ENABLE();

    led.Pin   = GPIO_PIN_13;
    led.Mode  = GPIO_MODE_OUTPUT;
    led.OType = GPIO_OTYPE_PP;
    led.PuPd  = GPIO_NOPULL;
    led.Speed = GPIO_SPEED_MEDIUM;

    GPIO_Init(GPIOD, &led);

    GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
}

/**
 * @brief EXTI0 interrupt handler (User Button)
 */
void EXTI0_IRQHandler(void)
{
    /* Clear pending bit */
    if (EXTI->PR & (1U << 0))
    {
        EXTI->PR |= (1U << 0);  // Clear pending

        /* Toggle LED */
        GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    }
}
