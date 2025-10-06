/**
 * @file    systick.h
 * @brief   SysTick Timer Driver for STM32F407
 * @author  Hasan Erol
 */

#ifndef INC_SYSTICK_H_
#define INC_SYSTICK_H_

#include "stm32f407xx.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint32_t systick_ms;

/* -------------------------------------------------------------------------- */
/* Clock source options                                                       */
/* -------------------------------------------------------------------------- */
/**
 * @defgroup SYSTICK_ClockSource Clock Source
 * @brief SysTick clock source selection
 * @{
 */
#define SYSTICK_CLKSRC_AHB        (1U)  /*!< Processor clock (AHB) */
#define SYSTICK_CLKSRC_AHB_DIV8   (0U)  /*!< AHB clock divided by 8 */
/** @} */

/* ---------------------------------------------------------------------------- */
/* Interrupt control                                                            */
/* ---------------------------------------------------------------------------- */
/**
 * @defgroup SYSTICK_Interrupt Interrupt Control
 * @brief SysTick interrupt enable/disable options
 * @{
 */
#define SYSTICK_INT_DISABLE       (0U)  /*!< Disable SysTick interrupt */
#define SYSTICK_INT_ENABLE        (1U)  /*!< Enable SysTick interrupt */
/** @} */

/* ---------------------------------------------------------------------------- */
/* Handle structure                                                             */
/* ---------------------------------------------------------------------------- */
/**
 * @brief SysTick configuration structure
 */
typedef struct {
    uint32_t tick_hz;       /*!< Tick frequency in Hz (e.g. 1000 for 1 ms)      */
    uint32_t clksource;     /*!< Clock source     @defgroup SYSTICK_ClockSource */
    uint8_t  use_interrupt; /*!< Interrupt usage  @defgroup SYSTICK_Interrupt   */
    uint8_t  nvic_priority; /*!< Interrupt priority (0..255) */
} SYSTICK_HandleTypeDef_t;

/* ---------------------------------------------------------------------------- */
/* API Prototypes                                                             */
/* -------------------------------------------------------------------------- */
void SysTick_Init(uint32_t tick_hz);
void SysTick_InitEx(SYSTICK_HandleTypeDef_t *hsystick);

void delay_ms(uint32_t ms);
bool SysTick_Elapsed(uint32_t ms);
bool SysTick_TimeoutElapsed(uint32_t start, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* INC_SYSTICK_H_ */
