/**
 * @file    GPIO.h
 * @brief   General-Purpose Input/Output (GPIO) driver for STM32F407
 * @author  Hasan Erol
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "stm32f407xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Type Definitions                               */
/* ========================================================================== */

/**
 * @brief GPIO Pin numbers
 */
typedef enum
{
    GPIO_PIN_0     = 0x0001U,  /*!< GPIO Pin 0 selected  */
    GPIO_PIN_1     = 0x0002U,  /*!< GPIO Pin 1 selected  */
    GPIO_PIN_2     = 0x0004U,  /*!< GPIO Pin 2 selected  */
    GPIO_PIN_3     = 0x0008U,  /*!< GPIO Pin 3 selected  */
    GPIO_PIN_4     = 0x0010U,  /*!< GPIO Pin 4 selected  */
    GPIO_PIN_5     = 0x0020U,  /*!< GPIO Pin 5 selected  */
    GPIO_PIN_6     = 0x0040U,  /*!< GPIO Pin 6 selected  */
    GPIO_PIN_7     = 0x0080U,  /*!< GPIO Pin 7 selected  */
    GPIO_PIN_8     = 0x0100U,  /*!< GPIO Pin 8 selected  */
    GPIO_PIN_9     = 0x0200U,  /*!< GPIO Pin 9 selected  */
    GPIO_PIN_10    = 0x0400U,  /*!< GPIO Pin 10 selected */
    GPIO_PIN_11    = 0x0800U,  /*!< GPIO Pin 11 selected */
    GPIO_PIN_12    = 0x1000U,  /*!< GPIO Pin 12 selected */
    GPIO_PIN_13    = 0x2000U,  /*!< GPIO Pin 13 selected */
    GPIO_PIN_14    = 0x4000U,  /*!< GPIO Pin 14 selected */
    GPIO_PIN_15    = 0x8000U,  /*!< GPIO Pin 15 selected */
    GPIO_PIN_ALL   = 0xFFFFU   /*!< All GPIO pins selected */
} GPIO_Pin_t;


/**
 * @brief GPIO operating modes
 */
typedef enum
{
    GPIO_MODE_INPUT   = 0x0U,  /*!< Input floating or with pull-up/down */
    GPIO_MODE_OUTPUT  = 0x1U,  /*!< General purpose output mode         */
    GPIO_MODE_AF      = 0x2U,  /*!< Alternate function mode             */
    GPIO_MODE_ANALOG  = 0x3U   /*!< Analog mode                         */
} GPIO_Mode_t;


/**
 * @brief GPIO output types
 */
typedef enum
{
    GPIO_OTYPE_PP = 0x0U,      /*!< Push-pull output type (reset state) */
    GPIO_OTYPE_OD = 0x1U       /*!< Open-drain output type              */
} GPIO_OType_t;


/**
 * @brief GPIO output speed levels
 */
typedef enum
{
    GPIO_SPEED_LOW       = 0x0U,  /*!< Low speed        */
    GPIO_SPEED_MEDIUM    = 0x1U,  /*!< Medium speed     */
    GPIO_SPEED_HIGH      = 0x2U,  /*!< High speed       */
    GPIO_SPEED_VERYHIGH  = 0x3U   /*!< Very high speed  */
} GPIO_Speed_t;


/**
 * @brief GPIO pull-up/pull-down configuration
 */
typedef enum
{
    GPIO_NOPULL   = 0x0U,  /*!< No pull-up or pull-down */
    GPIO_PULLUP   = 0x1U,  /*!< Pull-up enabled         */
    GPIO_PULLDOWN = 0x2U   /*!< Pull-down enabled       */
} GPIO_PuPd_t;


/**
 * @brief GPIO pin state definition
 */
typedef enum
{
    GPIO_PIN_RESET = 0x0U,  /*!< Logic 0 */
    GPIO_PIN_SET   = 0x1U   /*!< Logic 1 */
} GPIO_PinState_t;

/* ========================================================================== */
/*                   GPIO Alternate Function (AF) Definitions                 */
/* ========================================================================== */

/* AF0  - System functions (MCO, SWD, JTMS, JTCK, etc.)      */  #define GPIO_AF0_SYSTEM       0x00U
/* AF1  - TIM1 / TIM2                                        */  #define GPIO_AF1_TIM1_2       0x01U
/* AF2  - TIM3 / TIM4 / TIM5                                 */  #define GPIO_AF2_TIM3_5       0x02U
/* AF3  - TIM8 / TIM9 / TIM10 / TIM11                        */  #define GPIO_AF3_TIM8_11      0x03U
/* AF4  - I2C1 / I2C2 / I2C3                                 */  #define GPIO_AF4_I2C1_3       0x04U
/* AF5  - SPI1 / SPI2                                        */  #define GPIO_AF5_SPI1_2       0x05U
/* AF6  - SPI3                                               */  #define GPIO_AF6_SPI3         0x06U
/* AF7  - USART1 / USART2 / USART3                           */  #define GPIO_AF7_USART1_3     0x07U
/* AF8  - USART4 / USART5 / USART6                           */  #define GPIO_AF8_USART4_6     0x08U
/* AF9  - CAN1 / CAN2 / TIM12–14                             */  #define GPIO_AF9_CAN_TIM      0x09U

/**
 * @brief GPIO initialization structure definition
 */
typedef struct
{
    GPIO_Pin_t      Pin;        /*!< Specifies the GPIO pin(s) to be configured.       @ref GPIO_Pin_t */
    GPIO_Mode_t     Mode;       /*!< Specifies the operating mode for the selected pin. @ref GPIO_Mode_t */
    GPIO_OType_t    OType;      /*!< Specifies the output type.                        @ref GPIO_OType_t */
    GPIO_PuPd_t     PuPd;       /*!< Specifies the Pull-up or Pull-down activation.    @ref GPIO_PuPd_t */
    GPIO_Speed_t    Speed;      /*!< Specifies the speed for the selected pin.         @ref GPIO_Speed_t */
    uint32_t        Alternate;  /*!< Peripheral alternate function number (0–15).      */
} GPIO_InitTypeDef_t;

/* ========================================================================== */
/*                             Function Prototypes                            */
/* ========================================================================== */

void GPIO_Init(GPIO_TypeDef_t *GPIOx, GPIO_InitTypeDef_t *GPIO_ConfigStruct);
void GPIO_DeInit(GPIO_TypeDef_t *GPIOx);


void GPIO_WritePin(GPIO_TypeDef_t *GPIOx, GPIO_Pin_t pinNumber, GPIO_PinState_t pinState);
GPIO_PinState_t GPIO_ReadPin(GPIO_TypeDef_t *GPIOx, GPIO_Pin_t pinNumber);

void GPIO_LockPin(GPIO_TypeDef_t *GPIOx, GPIO_Pin_t Pin);
void GPIO_TogglePin(GPIO_TypeDef_t *GPIOx, GPIO_Pin_t pinNumber);


#ifdef __cplusplus
}
#endif

#endif /* INC_GPIO_H_ */
