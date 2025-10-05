/**
 * @file    EXTI.h
 * @brief   External Interrupt (EXTI) driver for STM32F407
 * @author  erol-tesla
 * @date    Aug 2025
 */

#ifndef INC_EXTI_H_
#define INC_EXTI_H_

#include "stm32f407xx.h"

/* ========================================================================== */
/*                              Enumerations                                  */
/* ========================================================================== */

/**
 * @brief GPIO port source for EXTI configuration
 */
typedef enum {
    EXTI_PORT_A = 0x0U,
    EXTI_PORT_B = 0x1U,
    EXTI_PORT_C = 0x2U,
    EXTI_PORT_D = 0x3U,
    EXTI_PORT_E = 0x4U,
    EXTI_PORT_F = 0x5U,
    EXTI_PORT_G = 0x6U,
    EXTI_PORT_H = 0x7U,
    EXTI_PORT_I = 0x8U
} exti_port_t;

/**
 * @brief EXTI line selection (0–15)
 */
typedef enum {
    EXTI_LINE_0  = 0x0U,
    EXTI_LINE_1  = 0x1U,
    EXTI_LINE_2  = 0x2U,
    EXTI_LINE_3  = 0x3U,
    EXTI_LINE_4  = 0x4U,
    EXTI_LINE_5  = 0x5U,
    EXTI_LINE_6  = 0x6U,
    EXTI_LINE_7  = 0x7U,
    EXTI_LINE_8  = 0x8U,
    EXTI_LINE_9  = 0x9U,
    EXTI_LINE_10 = 0xAU,
    EXTI_LINE_11 = 0xBU,
    EXTI_LINE_12 = 0xCU,
    EXTI_LINE_13 = 0xDU,
    EXTI_LINE_14 = 0xEU,
    EXTI_LINE_15 = 0xFU
} exti_line_t;

/**
 * @brief EXTI operating mode
 */
typedef enum {
    EXTI_MODE_INTERRUPT = 0x00U,
    EXTI_MODE_EVENT     = 0x04U
} exti_mode_t;

/**
 * @brief EXTI trigger configuration
 */
typedef enum {
    EXTI_TRIGGER_RISING  = 0x08U,
    EXTI_TRIGGER_FALLING = 0x0CU,
    EXTI_TRIGGER_BOTH    = 0x10U
} exti_trigger_t;


/* ========================================================================== */
/*                              Structures                                    */
/* ========================================================================== */

/**
 * @brief  EXTI initialization structure definition
 */
typedef struct
{
    exti_line_t        LineNumber;     /*!< Line number (0–15)             */
    exti_trigger_t     TriggerSelect;  /*!< Trigger selection              */
    exti_mode_t        Mode;           /*!< Interrupt or event mode        */
    FunctionalState_t  LineCmd;        /*!< Enable/disable EXTI line       */
} EXTI_InitTypeDef_t;


/* ========================================================================== */
/*                              API Prototypes                                */
/* ========================================================================== */

/**
 * @brief  Configure EXTI line source mapping.
 * @param  PortSource GPIO port (see @ref exti_port_t)
 * @param  LineSource EXTI line (see @ref exti_line_t)
 */
void EXTI_LineConfig(exti_port_t PortSource, exti_line_t LineSource);

/**
 * @brief  Initialize EXTI peripheral according to parameters in EXTI_InitTypeDef_t.
 * @param  EXTI_InitConfig Pointer to EXTI configuration structure.
 */
void EXTI_Init(EXTI_InitTypeDef_t *EXTI_InitConfig);

/**
 * @brief  Enable NVIC interrupt for EXTI line.
 * @param  IRQNumber IRQ number to enable.
 */
void NVIC_EnableInterrupt(EXTI_IRQNumber_t IRQNumber);

#endif /* INC_EXTI_H_ */
