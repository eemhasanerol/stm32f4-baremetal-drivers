/**
 * @file    rcc.h
 * @brief   Reset and Clock Control (RCC) driver for STM32F407
 * @author  Hasan Erol
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#include "stm32f407xx.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------------------------------------- */
/* RCC GPIO Clock Enable Macros                                      							            */
/* -------------------------------------------------------------------------------------------------------- */
#define RCC_GPIOA_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);  					\
											tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_GPIOB_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);  					\
											tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_GPIOC_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);  					\
											tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_GPIOD_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);  					\
											tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_GPIOE_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);  					\
											tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_GPIOF_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN);  					\
											tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_GPIOG_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOGEN);  					\
											tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOGEN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_GPIOH_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN);  					\
											tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_GPIOI_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOIEN);  					\
											tempValue = READ_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOIEN);		\
											UNUSED(tempValue);												\
											}while(0);



/* ------------------------------------------------------------------------------------------------------- */
/* RCC I2C Peripheral Clock Enable Macros                       							               */
/* ------------------------------------------------------------------------------------------------------- */
#define RCC_I2C1_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);						\
											tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN);			\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_I2C2_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);						\
											tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN);			\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_I2C3_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);						\
											tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN);			\
											UNUSED(tempValue);												\
											}while(0);



/* ------------------------------------------------------------------------------------------------------- */
/* RCC SPI Peripheral Clock Enable Macros                                 							       */
/* ------------------------------------------------------------------------------------------------------- */
#define RCC_SPI1_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);						\
											tempValue = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN);		    \
											UNUSED(tempValue);												\
											}while(0);


#define RCC_SPI2_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);						\
											tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN);			\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_SPI3_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN);						\
											tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN);			\
											UNUSED(tempValue);												\
											}while(0);



/* ------------------------------------------------------------------------------------------------------- */
/* RCC USART Peripheral Clock Enable Macros                                 							   */
/* ------------------------------------------------------------------------------------------------------- */
#define RCC_USART1_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);					\
											tempValue = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_USART2_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);					\
											tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_USART3_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);					\
											tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_UART4_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN);						\
											tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_UART5_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN);						\
											tempValue = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN);		\
											UNUSED(tempValue);												\
											}while(0);


#define RCC_USART6_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN);					\
											tempValue = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN);		\
											UNUSED(tempValue);												\
											}while(0);



/* ------------------------------------------------------------------------------------------------------- */
/* RCC SYSCFG Clock Enable / Disable Macros                                   							   */
/* ------------------------------------------------------------------------------------------------------- */
#define RCC_SYSCFG_CLK_ENABLE()			do{ uint32_t tempValue = 0;  										\
											SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);  					\
											tempValue = READ_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);		\
											UNUSED(tempValue);												\
											}while(0);

#define RCC_SYSCFG_CLK_DISABLE()		CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN)



/* ------------------------------------------------------------------------------ */
/* RCC Peripheral Clock Disable Macros                                            */
/* ------------------------------------------------------------------------------ */
#define RCC_GPIOA_CLK_DISABLE()			CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN)
#define RCC_GPIOB_CLK_DISABLE()			CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOBEN)
#define RCC_GPIOC_CLK_DISABLE()			CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOCEN)
#define RCC_GPIOD_CLK_DISABLE()			CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIODEN)
#define RCC_GPIOE_CLK_DISABLE()			CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOEEN)
#define RCC_GPIOF_CLK_DISABLE()			CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOFEN)
#define RCC_GPIOG_CLK_DISABLE()			CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOGEN)
#define RCC_GPIOH_CLK_DISABLE()			CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOHEN)
#define RCC_GPIOI_CLK_DISABLE()			CLEAR_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOIEN)


#define RCC_I2C2_CLK_DISABLE()			CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C2EN)
#define RCC_I2C1_CLK_DISABLE()			CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C1EN)
#define RCC_I2C3_CLK_DISABLE()			CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_I2C3EN)


#define RCC_SPI1_CLK_DISABLE()			CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_SPI1EN)
#define RCC_SPI2_CLK_DISABLE()			CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI2EN)
#define RCC_SPI3_CLK_DISABLE()			CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_SPI3EN)


#define RCC_USART1_CLK_DISABLE()	    CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART1EN)
#define RCC_USART2_CLK_DISABLE()	    CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART2EN)
#define RCC_USART3_CLK_DISABLE()	    CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_USART3EN)
#define RCC_UART4_CLK_DISABLE()	    	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART4EN )
#define RCC_UART5_CLK_DISABLE()	    	CLEAR_BIT(RCC->APB1ENR, RCC_APB1ENR_UART5EN )
#define RCC_USART6_CLK_DISABLE()	    CLEAR_BIT(RCC->APB2ENR, RCC_APB2ENR_USART6EN)



/* ---------------------------------------------------- */
/* RCC Frequency Getter Prototypes                      */
/* ---------------------------------------------------- */
uint32_t RCC_GetPCLK1Freq(void);
uint32_t RCC_GetPCLK2Freq(void);
uint32_t RCC_GetPLLOutputClockFreq(void);

#ifdef __cplusplus
}
#endif


#endif /* INC_RCC_H_ */
