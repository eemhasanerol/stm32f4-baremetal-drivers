/**
 * @file    stm32f407xx.h
 * @brief   STM32F407xx Device Header File (Memory map and register definitions)
 * @author  Hasan Erol
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>
#define __IO volatile

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                     Processor Specific Definitions                         */
/* ========================================================================== */

/* NVIC ISER Registers (Interrupt Set Enable) -------------------------------- */
#define NVIC_ISER0          ( (__IO uint32_t*)0xE000E100 )
#define NVIC_ISER1          ( (__IO uint32_t*)0xE000E104 )
#define NVIC_ISER2          ( (__IO uint32_t*)0xE000E108 )
#define NVIC_ISER3          ( (__IO uint32_t*)0xE000E10c )

/* NVIC ICER Registers (Interrupt Clear Enable) ------------------------------ */
#define NVIC_ICER0 			((__IO uint32_t*)0XE000E180)
#define NVIC_ICER1			((__IO uint32_t*)0XE000E184)
#define NVIC_ICER2  		((__IO uint32_t*)0XE000E188)
#define NVIC_ICER3			((__IO uint32_t*)0XE000E18C)

/* NVIC Priority Register Base ---------------------------------------------- */
#define NVIC_PR_BASE_ADDR 	((__IO uint32_t*)0xE000E400)

/* Number of priority bits implemented -------------------------------------- */
#define NO_PR_BITS_IMPLEMENTED  4

extern uint32_t SystemCoreClock;


/* ========================================================================== */
/*                             Generic Types                                  */
/* ========================================================================== */
typedef enum
{
	DISABLE = 0x0U,
	ENABLE = !DISABLE
}FunctionalState_t;


/* ========================================================================== */
/*                              IRQ Numbers                                   */
/* ========================================================================== */
typedef enum
{
    EXTI0_IRQNumber      = 6,   /*!< EXTI Line0 Interrupt */
    EXTI1_IRQNumber      = 7,   /*!< EXTI Line1 Interrupt */
    EXTI2_IRQNumber      = 8,   /*!< EXTI Line2 Interrupt */
    EXTI3_IRQNumber      = 9,   /*!< EXTI Line3 Interrupt */
    EXTI4_IRQNumber      = 10,  /*!< EXTI Line4 Interrupt */
    EXTI9_5_IRQNumber    = 23,  /*!< EXTI Line[9:5] Interrupt */
    EXTI15_10_IRQNumber  = 40   /*!< EXTI Line[15:10] Interrupt */
} EXTI_IRQNumber_t;


/* ========================================================================== */
/*                         Base Memory Addresses                              */
/* ========================================================================== */
#define FLASH_BASE     		(0x08000000UL)  /* Flash memory base address (up to 1 MB) */
#define SRAM1_BASE     		(0x20000000UL)  /* SRAM1 base address (112 KB) 			  */
#define SRAM2_BASE     		(0x2001C000UL)  /* SRAM2 base address (16 KB)             */


/* ========================================================================== */
/*                       Peripheral Base Addresses                            */
/* ========================================================================== */
#define PERIPH_BASE_ADDR   (0x40000000UL)   				/* Base address of all peripherals */

#define APB1_BASE_ADDR     PERIPH_BASE_ADDR          		/* APB1 bus domain base address	   */
#define APB2_BASE_ADDR     (PERIPH_BASE_ADDR + 0x10000UL) 	/* APB2 bus domain base address    */
#define AHB1_BASE_ADDR     (PERIPH_BASE_ADDR + 0x20000UL)  	/* AHB1 bus domain base address    */
#define AHB2_BASE_ADDR     (0x50000000UL)             		/* AHB2 bus domain base address    */


/*
 * APB1 Peripherals Base Addresses
 *
 */
#define TIM2_BASE_ADDR       (APB1_BASE_ADDR + 0x0000UL)   /* TIM2 base address    */
#define TIM3_BASE_ADDR       (APB1_BASE_ADDR + 0x0400UL)   /* TIM3 base address    */
#define TIM4_BASE_ADDR       (APB1_BASE_ADDR + 0x0800UL)   /* TIM4 base address    */
#define TIM5_BASE_ADDR       (APB1_BASE_ADDR + 0x0C00UL)   /* TIM5 base address    */
#define TIM6_BASE_ADDR       (APB1_BASE_ADDR + 0x1000UL)   /* TIM6 base address    */
#define TIM7_BASE_ADDR       (APB1_BASE_ADDR + 0x1400UL)   /* TIM7 base address    */
#define TIM12_BASE_ADDR      (APB1_BASE_ADDR + 0x1800UL)   /* TIM12 base address   */
#define TIM13_BASE_ADDR      (APB1_BASE_ADDR + 0x1C00UL)   /* TIM13 base address   */
#define TIM14_BASE_ADDR      (APB1_BASE_ADDR + 0x2000UL)   /* TIM14 base address   */

#define RTC_BKP_BASE_ADDR    (APB1_BASE_ADDR + 0x2800UL)   /* RTC & BKP base       */
#define WWDG_BASE_ADDR       (APB1_BASE_ADDR + 0x2C00UL)   /* Window Watchdog      */
#define IWDG_BASE_ADDR       (APB1_BASE_ADDR + 0x3000UL)   /* Independent Watchdog */

#define SPI2_BASE_ADDR       (APB1_BASE_ADDR + 0x3800UL)   /* SPI2 / I2S2 base     */
#define SPI3_BASE_ADDR       (APB1_BASE_ADDR + 0x3C00UL)   /* SPI3 / I2S3 base     */

#define USART2_BASE_ADDR     (APB1_BASE_ADDR + 0x4400UL)   /* USART2 base address  */
#define USART3_BASE_ADDR     (APB1_BASE_ADDR + 0x4800UL)   /* USART3 base address  */
#define UART4_BASE_ADDR      (APB1_BASE_ADDR + 0x4C00UL)   /* UART4 base address   */
#define UART5_BASE_ADDR      (APB1_BASE_ADDR + 0x5000UL)   /* UART5 base address   */

#define I2C1_BASE_ADDR       (APB1_BASE_ADDR + 0x5400UL)   /* I2C1 base address    */
#define I2C2_BASE_ADDR       (APB1_BASE_ADDR + 0x5800UL)   /* I2C2 base address    */
#define I2C3_BASE_ADDR       (APB1_BASE_ADDR + 0x5C00UL)   /* I2C3 base address    */

#define CAN1_BASE_ADDR       (APB1_BASE_ADDR + 0x6400UL)   /* CAN1 base address    */
#define CAN2_BASE_ADDR       (APB1_BASE_ADDR + 0x6800UL)   /* CAN2 base address    */

#define PWR_BASE_ADDR        (APB1_BASE_ADDR + 0x7000UL)   /* Power Control (PWR)  */
#define DAC_BASE_ADDR        (APB1_BASE_ADDR + 0x7400UL)   /* DAC base address     */


/*
 * APB2 Peripherals Base Addresses
 *
 */
#define TIM1_BASE_ADDR       (APB2_BASE_ADDR + 0x0000UL)   /* TIM1 base address    */
#define TIM8_BASE_ADDR       (APB2_BASE_ADDR + 0x0400UL)   /* TIM8 base address    */

#define USART1_BASE_ADDR     (APB2_BASE_ADDR + 0x1000UL)   /* USART1 base address  */
#define USART6_BASE_ADDR     (APB2_BASE_ADDR + 0x1400UL)   /* USART6 base address  */

#define ADC1_BASE_ADDR       (APB2_BASE_ADDR + 0x2000UL)   /* ADC1 base address    */
#define ADC2_BASE_ADDR       (APB2_BASE_ADDR + 0x2100UL)   /* ADC2 base address    */
#define ADC3_BASE_ADDR       (APB2_BASE_ADDR + 0x2200UL)   /* ADC3 base address    */
#define ADC_COMMON_BASE_ADDR (APB2_BASE_ADDR + 0x2300UL)   /* ADC common registers */

#define SDIO_BASE_ADDR       (APB2_BASE_ADDR + 0x2C00UL)   /* SDIO base address    */

#define SPI1_BASE_ADDR       (APB2_BASE_ADDR + 0x3000UL)   /* SPI1 base address    */
#define SPI4_BASE_ADDR       (APB2_BASE_ADDR + 0x3400UL)   /* SPI4 base address    */

#define SYSCFG_BASE_ADDR     (APB2_BASE_ADDR + 0x3800UL)   /* SYSCFG base address  */
#define EXTI_BASE_ADDR       (APB2_BASE_ADDR + 0x3C00UL)   /* EXTI base address    */

#define TIM9_BASE_ADDR       (APB2_BASE_ADDR + 0x4000UL)   /* TIM9 base address    */
#define TIM10_BASE_ADDR      (APB2_BASE_ADDR + 0x4400UL)   /* TIM10 base address   */
#define TIM11_BASE_ADDR      (APB2_BASE_ADDR + 0x4800UL)   /* TIM11 base address   */


/*
 * AHB1 Peripherals Base Addresses
 *
 */
#define GPIOA_BASE_ADDR      (AHB1_BASE_ADDR + 0x0000UL)   /* GPIOA base address   */
#define GPIOB_BASE_ADDR      (AHB1_BASE_ADDR + 0x0400UL)   /* GPIOB base address   */
#define GPIOC_BASE_ADDR      (AHB1_BASE_ADDR + 0x0800UL)   /* GPIOC base address   */
#define GPIOD_BASE_ADDR      (AHB1_BASE_ADDR + 0x0C00UL)   /* GPIOD base address   */
#define GPIOE_BASE_ADDR      (AHB1_BASE_ADDR + 0x1000UL)   /* GPIOE base address   */
#define GPIOF_BASE_ADDR      (AHB1_BASE_ADDR + 0x1400UL)   /* GPIOF base address   */
#define GPIOG_BASE_ADDR      (AHB1_BASE_ADDR + 0x1800UL)   /* GPIOG base address   */
#define GPIOH_BASE_ADDR      (AHB1_BASE_ADDR + 0x1C00UL)   /* GPIOH base address   */
#define GPIOI_BASE_ADDR      (AHB1_BASE_ADDR + 0x2000UL)   /* GPIOI base address   */

#define CRC_BASE_ADDR        (AHB1_BASE_ADDR + 0x3000UL)   /* CRC base address     */

#define RCC_BASE_ADDR        (AHB1_BASE_ADDR + 0x3800UL)   /* RCC base address     */

#define FLASH_INTF_BASE_ADDR (AHB1_BASE_ADDR + 0x3C00UL)   /* Flash interface      */

#define DMA1_BASE_ADDR       (AHB1_BASE_ADDR + 0x6000UL)   /* DMA1 base address    */
#define DMA2_BASE_ADDR       (AHB1_BASE_ADDR + 0x6400UL)   /* DMA2 base address    */

#define ETH_BASE_ADDR        (AHB1_BASE_ADDR + 0x8000UL)   /* Ethernet MAC base    */


/*
 * AHB2 Peripherals Base Addresses
 *
 */
#define USB_OTG_FS_BASE_ADDR   (AHB2_BASE_ADDR + 0x0000UL)   /* USB OTG FS base addr */
#define DCMI_BASE_ADDR         (AHB2_BASE_ADDR + 0x50000UL)  /* DCMI base address    */
#define CRYP_BASE_ADDR         (AHB2_BASE_ADDR + 0x60000UL)  /* CRYP base address    */
#define HASH_BASE_ADDR         (AHB2_BASE_ADDR + 0x60400UL)  /* HASH base address    */
#define RNG_BASE_ADDR          (AHB2_BASE_ADDR + 0x60800UL)  /* RNG base address     */


/*
 * AHB3 Peripherals Base Addresses
 *
 */
#define FMC_BASE_ADDR         (0xA0000000UL)                /* Flexible Mem Ctrl    */


#define SYSTICK_BASE        (0xE000E010UL)




/* ========================================================================== */
/*                          Peripheral Structures                             */
/* ========================================================================== */


/**
 * @brief RCC Register Definition Structure
 */
typedef struct
{
    __IO uint32_t CR;            /*!< RCC clock control register                      		Address offset: 0x00 */
    __IO uint32_t PLLCFGR;       /*!< RCC PLL configuration register                  		Address offset: 0x04 */
    __IO uint32_t CFGR;          /*!< RCC clock configuration register                		Address offset: 0x08 */
    __IO uint32_t CIR;           /*!< RCC clock interrupt register                    		Address offset: 0x0C */
    __IO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register             		Address offset: 0x10 */
    __IO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register              		Address offset: 0x14 */
    __IO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register              		Address offset: 0x18 */
    uint32_t      RESERVED0;     /*!< RESERVED AREA                                   		Address offset: 0x1C */
    __IO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register              		Address offset: 0x20 */
    __IO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register              		Address offset: 0x24 */
    uint32_t      RESERVED1[2];  /*!< RESERVED AREA                                   		Address offset: 0x28 */
    __IO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock enable register       		Address offset: 0x30 */
    __IO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock enable register       		Address offset: 0x34 */
    __IO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock enable register       		Address offset: 0x38 */
    uint32_t      RESERVED2;     /*!< RESERVED EXTI_InitConfigAREA                          Address offset: 0x3C */
    __IO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register       		Address offset: 0x40 */
    __IO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register      		Address offset: 0x44 */
    uint32_t      RESERVED3[2];  /*!< RESERVED AREA                                   		Address offset: 0x48 */
    __IO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable low power register 	Address offset: 0x50 */
    __IO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable low power register 	Address offset: 0x54 */
    __IO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable low power register 	Address offset: 0x58 */
    uint32_t      RESERVED4;     /*!< RESERVED AREA                                  		Address offset: 0x5C */
    __IO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable low power register 	Address offset: 0x60 */
    __IO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable low power register 	Address offset: 0x64 */
    uint32_t      RESERVED5[2];  /*!< RESERVED AREA                                   		Address offset: 0x68 */
    __IO uint32_t BDCR;          /*!< RCC Backup domain control register              		Address offset: 0x70 */
    __IO uint32_t CSR;           /*!< RCC clock control & status register             		Address offset: 0x74 */
    uint32_t      RESERVED6[2];  /*!< RESERVED AREA                                   		Address offset: 0x78 */
    __IO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register   		Address offset: 0x80 */
    __IO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register               		Address offset: 0x84 */

} RCC_TypeDef_t;


/**
 * @brief GPIO Register Definition Structure
 */
typedef struct
{
    __IO uint32_t MODER;    	/*!< GPIO port mode register            					Address offset: 0x0000 */
    __IO uint32_t OTYPER;   	/*!< GPIO port output type register      					Address offset: 0x0004 */
    __IO uint32_t OSPEEDR;  	/*!< GPIO port output speed register     					Address offset: 0x0008 */
    __IO uint32_t PUPDR;    	/*!< GPIO port pull-up/pull-down register   				Address offset: 0x000C */
    __IO uint32_t IDR;     		/*!< GPIO port input data register          				Address offset: 0x0010 */
    __IO uint32_t ODR;      	/*!< GPIO port output data register         				Address offset: 0x0014 */
    __IO uint32_t BSRR;     	/*!< GPIO port bit set/reset register       				Address offset: 0x0018 */
    __IO uint32_t LCKR;     	/*!< GPIO port configuration lock reg       				Address offset: 0x001C */
    __IO uint32_t AFR[2];   	/*!< GPIO alternate function registers      				Address offset: 0x0020 */

}GPIO_TypeDef_t;


/**
 * @brief EXTI Register Definition Structure
 */
typedef struct
{
	__IO uint32_t IMR;			 /*!< EXTI Interrupt mask register 						     Address offset: 0x00 */
	__IO uint32_t EMR;			 /*!< EXTI Event mask register 						         Address offset: 0x04 */
	__IO uint32_t RTSR;			 /*!< EXTI Rising trigger selection register 				 Address offset: 0x08 */
	__IO uint32_t FTSR;			 /*!< EXTI Falling trigger selection register 				 Address offset: 0x0C */
	__IO uint32_t SWIER;		 /*!< EXTI Software interrupt event register 				 Address offset: 0x10 */
	__IO uint32_t PR;			 /*!< EXTI Pending register				 				     Address offset: 0x14 */

}EXTI_TypeDef_t;


/**
 * @brief SysTick Register Definition Structure
 */
typedef struct
{
    __IO uint32_t CTRL;      /*!< SysTick control and status register                   Address offset: 0x00 */
    __IO uint32_t LOAD;      /*!< SysTick reload value register                         Address offset: 0x04 */
    __IO uint32_t VAL;       /*!< SysTick current value register                        Address offset: 0x08 */
    __IO uint32_t CALIB;     /*!< SysTick calibration value register                    Address offset: 0x0C */
} SysTick_TypeDef_t;

/*
 * peripheral register definition structure for SYSCFG
 *
 */
typedef struct
{
	__IO uint32_t MEMRMP;		 /*!< SYSCFG memory remap register               		 	Address offset: 0x00 */
	__IO uint32_t PMC;			 /*!< SYSCFG peripheral mode configuration register         Address offset: 0x04 */
	__IO uint32_t EXTI_CR[4];	 /*!< SYSCFG external interrupt configuration register      Address offset: 0x08 */
	__IO uint32_t CMPCR;		 /*!< SYSCFG Compensation cell control register             Address offset: 0x20 */

}SYSCFG_TypeDef_t;


/*
 * peripheral register definition structure for I2C
 *
 */
typedef struct
{
	__IO uint32_t CR1;			/*!< I2C Control register 1						   	  		  Address offset: 0x00 */
	__IO uint32_t CR2;			/*!< I2C Control register 2						   	  		  Address offset: 0x04 */
	__IO uint32_t OAR1;			/*!< I2C Own address register 1						   	  	  Address offset: 0x08 */
	__IO uint32_t OAR2;			/*!< I2C Own address register 2						   	  	  Address offset: 0x0C */
	__IO uint32_t DR;			/*!< I2C Data Register 						   	  		      Address offset: 0x10 */
	__IO uint32_t SR1;			/*!< I2C Status register 1						   	  		  Address offset: 0x14 */
	__IO uint32_t SR2;			/*!< I2C Status register 2						   	  		  Address offset: 0x18 */
	__IO uint32_t CCR;			/*!< I2C Clock control register 						   	  Address offset: 0x1C */
	__IO uint32_t TRISE;		/*!< I2C TRISE register 						   	  		  Address offset: 0x20 */
	__IO uint32_t FLTR;			/*!< I2C FLTR register 						   	  		      Address offset: 0x24 */

}I2C_TypeDef_t;


/*
 * peripheral register definition structure for USART
 *
 */
typedef struct
{
	__IO uint32_t SR;			/*!< USART Status register 							   	  	  Address offset: 0x00 */
	__IO uint32_t DR;			/*!< USART Data register 1							   	      Address offset: 0x04 */
	__IO uint32_t BRR;			/*!< USART Baud Rate register 1						   	  	  Address offset: 0x08 */
	__IO uint32_t CR1;			/*!< USART Control register 1						   	  	  Address offset: 0x0C */
	__IO uint32_t CR2;			/*!< USART Control register 2						   	  	  Address offset: 0x10 */
	__IO uint32_t CR3;			/*!< USART Control register 3						   	  	  Address offset: 0x14 */
	__IO uint32_t GTPR;			/*!< USART Guard Time and prescaler register 			      Address offset: 0x18 */

}USART_TypeDef_t;


/*
 * peripheral register definition structure for SPI
 *
 */
typedef struct
{
	__IO uint32_t CR1;			/*!< SPI Control register 1							   	  	  Address offset: 0x00 */
	__IO uint32_t CR2;			/*!< SPI Control register 2							   	  	  Address offset: 0x04 */
	__IO uint32_t SR;			/*!< SPI Status register 							   	  	  Address offset: 0x08 */
	__IO uint32_t DR;			/*!< SPI Data register 							   	  	  	  Address offset: 0x0C */
	__IO uint32_t CRCPR;		/*!< SPI CRC polynomial register 							  Address offset: 0x10 */
	__IO uint32_t RXCRCR;		/*!< SPI RX CRC register					   	  	  		  Address offset: 0x00 */
	__IO uint32_t TXCRCR;		/*!< SPI TX CRC register 							   	  	  Address offset: 0x14 */
	__IO uint32_t I2SCFGR;		/*!< SPI I2S configuration register						   	  Address offset: 0x18 */
	__IO uint32_t I2SPR;		/*!< SPI I2S prescaler register 							  Address offset: 0x1C */

}SPI_TypeDef_t;




/* CTRL bits */
#define SYSTICK_CTRL_ENABLE     (1U << 0)
#define SYSTICK_CTRL_TICKINT    (1U << 1)
#define SYSTICK_CTRL_CLKSOURCE  (1U << 2)
#define SYSTICK_CTRL_COUNTFLAG  (1U << 16)

/* LOAD limit */
#define SYSTICK_LOAD_MAX        (0xFFFFFFUL)

/*
 * Peripheral definitions
 */

#define GPIOA   				( (GPIO_TypeDef_t *)(GPIOA_BASE_ADDR)  )
#define GPIOB   				( (GPIO_TypeDef_t *)(GPIOB_BASE_ADDR)  )
#define GPIOC   				( (GPIO_TypeDef_t *)(GPIOC_BASE_ADDR)  )
#define GPIOD   				( (GPIO_TypeDef_t *)(GPIOD_BASE_ADDR)  )
#define GPIOE   				( (GPIO_TypeDef_t *)(GPIOE_BASE_ADDR)  )
#define GPIOF   				( (GPIO_TypeDef_t *)(GPIOF_BASE_ADDR)  )
#define GPIOG  					( (GPIO_TypeDef_t *)(GPIOG_BASE_ADDR)  )
#define GPIOH  				    ( (GPIO_TypeDef_t *)(GPIOH_BASE_ADDR)  )
#define GPIOI  				    ( (GPIO_TypeDef_t *)(GPIOI_BASE_ADDR)  )

#define RCC						( (RCC_TypeDef_t * )(RCC_BASE_ADDR)    )

#define SYSCFG					( (SYSCFG_TypeDef_t*)(SYSCFG_BASE_ADDR))

#define EXTI					( (EXTI_TypeDef_t*)(EXTI_BASE_ADDR)  )

#define I2C1					( (I2C_TypeDef_t* )(I2C1_BASE_ADDR))
#define I2C2					( (I2C_TypeDef_t* )(I2C2_BASE_ADDR))
#define I2C3					( (I2C_TypeDef_t* )(I2C3_BASE_ADDR))

#define SPI1					( (SPI_TypeDef_t*)(SPI1_BASE_ADDR))
#define SPI2					( (SPI_TypeDef_t*)(SPI2_BASE_ADDR))
#define SPI3					( (SPI_TypeDef_t*)(SPI3_BASE_ADDR))

#define USART1					( (USART_TypeDef_t*)(USART1_BASE_ADDR))
#define USART2					( (USART_TypeDef_t*)(USART2_BASE_ADDR))
#define USART3					( (USART_TypeDef_t*)(USART3_BASE_ADDR))
#define UART4					( (USART_TypeDef_t*)(UART4_BASE_ADDR))
#define UART5					( (USART_TypeDef_t*)(UART5_BASE_ADDR))
#define USART6					( (USART_TypeDef_t*)(USART6_BASE_ADDR))

#define SysTick                 ((SysTick_TypeDef_t *)(SYSTICK_BASE))


/*
 * IRQ(Interrupt Request) Numbers
 */

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_I2C1_EV      31
#define IRQ_NO_I2C1_ER      32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71


/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI0    0
#define NVIC_IRQ_PRI15    15


/* ========================================================================== */
/*                          Common Macros                                    */
/* ========================================================================== */
#define FLAG_RESET  (0U)
#define FLAG_SET  	(1U)

#define SET_BIT(REG, BIT)		( (REG) |=  (BIT) )
#define CLEAR_BIT(REG, BIT)		( (REG) &= ~(BIT) )
#define READ_BIT(REG, BIT)		( (REG) &  (BIT) )
#define UNUSED(x)				(void)x

/* ======================================================================================================= */
/*                      RCC PERIPHERAL CLOCK ENABLE REGISTER BITS                                          */
/* ======================================================================================================= */
/* ------------------------------------------ RCC AHB1ENR register bits ---------------------------------- */
#define RCC_AHB1ENR_GPIOAEN_POS    (0U)     						 /*!< GPIOA clock enable bit position  */
#define RCC_AHB1ENR_GPIOAEN_MSK    (1U << RCC_AHB1ENR_GPIOAEN_POS)   /*!< GPIOA clock enable mask          */
#define RCC_AHB1ENR_GPIOAEN        RCC_AHB1ENR_GPIOAEN_MSK           /*!< GPIOA clock enable macro         */

#define RCC_AHB1ENR_GPIOBEN_POS    (1U)      						 /*!< GPIOB clock enable bit position  */
#define RCC_AHB1ENR_GPIOBEN_MSK    (1U << RCC_AHB1ENR_GPIOBEN_POS)   /*!< GPIOB clock enable mask          */
#define RCC_AHB1ENR_GPIOBEN        RCC_AHB1ENR_GPIOBEN_MSK           /*!< GPIOB clock enable macro         */

#define RCC_AHB1ENR_GPIOCEN_POS    (2U)      						 /*!< GPIOC clock enable bit position  */
#define RCC_AHB1ENR_GPIOCEN_MSK    (1U << RCC_AHB1ENR_GPIOCEN_POS)   /*!< GPIOC clock enable mask          */
#define RCC_AHB1ENR_GPIOCEN        RCC_AHB1ENR_GPIOCEN_MSK           /*!< GPIOC clock enable macro         */

#define RCC_AHB1ENR_GPIODEN_POS    (3U)      						 /*!< GPIOD clock enable bit position  */
#define RCC_AHB1ENR_GPIODEN_MSK    (1U << RCC_AHB1ENR_GPIODEN_POS)   /*!< GPIOD clock enable mask          */
#define RCC_AHB1ENR_GPIODEN        RCC_AHB1ENR_GPIODEN_MSK           /*!< GPIOD clock enable macro         */

#define RCC_AHB1ENR_GPIOEEN_POS    (4U)      						 /*!< GPIOE clock enable bit position  */
#define RCC_AHB1ENR_GPIOEEN_MSK    (1U << RCC_AHB1ENR_GPIOEEN_POS)   /*!< GPIOE clock enable mask          */
#define RCC_AHB1ENR_GPIOEEN        RCC_AHB1ENR_GPIOEEN_MSK           /*!< GPIOE clock enable macro         */

#define RCC_AHB1ENR_GPIOFEN_POS    (5U)      						 /*!< GPIOF clock enable bit position  */
#define RCC_AHB1ENR_GPIOFEN_MSK    (1U << RCC_AHB1ENR_GPIOFEN_POS)   /*!< GPIOF clock enable mask          */
#define RCC_AHB1ENR_GPIOFEN        RCC_AHB1ENR_GPIOFEN_MSK           /*!< GPIOF clock enable macro         */

#define RCC_AHB1ENR_GPIOGEN_POS    (6U)                              /*!< GPIOG clock enable bit position  */
#define RCC_AHB1ENR_GPIOGEN_MSK    (1U << RCC_AHB1ENR_GPIOGEN_POS)   /*!< GPIOG clock enable mask          */
#define RCC_AHB1ENR_GPIOGEN        RCC_AHB1ENR_GPIOGEN_MSK           /*!< GPIOG clock enable macro         */

#define RCC_AHB1ENR_GPIOHEN_POS    (7U)                              /*!< GPIOH clock enable bit position  */
#define RCC_AHB1ENR_GPIOHEN_MSK    (1U << RCC_AHB1ENR_GPIOHEN_POS)   /*!< GPIOH clock enable mask          */
#define RCC_AHB1ENR_GPIOHEN        RCC_AHB1ENR_GPIOHEN_MSK           /*!< GPIOH clock enable macro         */

#define RCC_AHB1ENR_GPIOIEN_POS    (8U)                              /*!< GPIOI clock enable bit position  */
#define RCC_AHB1ENR_GPIOIEN_MSK    (1U << RCC_AHB1ENR_GPIOIEN_POS)   /*!< GPIOI clock enable mask          */
#define RCC_AHB1ENR_GPIOIEN        RCC_AHB1ENR_GPIOIEN_MSK           /*!< GPIOI clock enable macro         */


/* ------------------------------------------ RCC APB1ENR register bits ---------------------------------- */
#define RCC_APB1ENR_SPI2EN_POS     (14U)                            /*!< SPI2 clock enable bit position     */
#define RCC_APB1ENR_SPI2EN_MSK     (1U << RCC_APB1ENR_SPI2EN_POS)   /*!< SPI2 clock enable mask             */
#define RCC_APB1ENR_SPI2EN         RCC_APB1ENR_SPI2EN_MSK           /*!< SPI2 clock enable macro            */

#define RCC_APB1ENR_SPI3EN_POS     (15U)                            /*!< SPI3 clock enable bit position     */
#define RCC_APB1ENR_SPI3EN_MSK     (1U << RCC_APB1ENR_SPI3EN_POS)   /*!< SPI3 clock enable mask             */
#define RCC_APB1ENR_SPI3EN         RCC_APB1ENR_SPI3EN_MSK           /*!< SPI3 clock enable macro            */

#define RCC_APB1ENR_USART2EN_POS   (17U)     			        	/*!< USART2 clock enable bit position   */
#define RCC_APB1ENR_USART2EN_MSK   (1U << RCC_APB1ENR_USART2EN_POS) /*!< USART2 clock enable mask           */
#define RCC_APB1ENR_USART2EN       RCC_APB1ENR_USART2EN_MSK         /*!< USART2 clock enable macro          */

#define RCC_APB1ENR_USART3EN_POS   (18U)                            /*!< USART3 clock enable bit position   */
#define RCC_APB1ENR_USART3EN_MSK   (1U << RCC_APB1ENR_USART3EN_POS) /*!< USART3 clock enable mask           */
#define RCC_APB1ENR_USART3EN       RCC_APB1ENR_USART3EN_MSK         /*!< USART3 clock enable macro          */

#define RCC_APB1ENR_UART4EN_POS    (19U)                            /*!< UART4 clock enable bit position    */
#define RCC_APB1ENR_UART4EN_MSK    (1U << RCC_APB1ENR_UART4EN_POS)  /*!< UART4 clock enable mask            */
#define RCC_APB1ENR_UART4EN        RCC_APB1ENR_UART4EN_MSK          /*!< UART4 clock enable macro           */

#define RCC_APB1ENR_UART5EN_POS    (20U)                            /*!< UART5 clock enable bit position    */
#define RCC_APB1ENR_UART5EN_MSK    (1U << RCC_APB1ENR_UART5EN_POS)  /*!< UART5 clock enable mask            */
#define RCC_APB1ENR_UART5EN        RCC_APB1ENR_UART5EN_MSK          /*!< UART5 clock enable macro           */

#define RCC_APB1ENR_I2C1EN_POS     (21U)                            /*!< I2C1 clock enable bit position     */
#define RCC_APB1ENR_I2C1EN_MSK     (1U << RCC_APB1ENR_I2C1EN_POS)   /*!< I2C1 clock enable mask             */
#define RCC_APB1ENR_I2C1EN         RCC_APB1ENR_I2C1EN_MSK           /*!< I2C1 clock enable macro            */

#define RCC_APB1ENR_I2C2EN_POS     (22U)                            /*!< I2C2 clock enable bit position     */
#define RCC_APB1ENR_I2C2EN_MSK     (1U << RCC_APB1ENR_I2C2EN_POS)   /*!< I2C2 clock enable mask             */
#define RCC_APB1ENR_I2C2EN         RCC_APB1ENR_I2C2EN_MSK           /*!< I2C2 clock enable macro            */

#define RCC_APB1ENR_I2C3EN_POS     (23U)                            /*!< I2C3 clock enable bit position     */
#define RCC_APB1ENR_I2C3EN_MSK     (1U << RCC_APB1ENR_I2C3EN_POS)   /*!< I2C3 clock enable mask             */
#define RCC_APB1ENR_I2C3EN         RCC_APB1ENR_I2C3EN_MSK           /*!< I2C3 clock enable macro            */


/* ------------------------------------------ RCC APB2ENR register bits ----------------------------------- */
#define RCC_APB2ENR_USART1EN_POS   (4U)                             /*!< USART1 clock enable bit position   */
#define RCC_APB2ENR_USART1EN_MSK   (1U << RCC_APB2ENR_USART1EN_POS) /*!< USART1 clock enable mask           */
#define RCC_APB2ENR_USART1EN       RCC_APB2ENR_USART1EN_MSK         /*!< USART1 clock enable macro          */

#define RCC_APB2ENR_USART6EN_POS   (5U)                             /*!< USART6 clock enable bit position   */
#define RCC_APB2ENR_USART6EN_MSK   (1U << RCC_APB2ENR_USART6EN_POS) /*!< USART6 clock enable mask           */
#define RCC_APB2ENR_USART6EN       RCC_APB2ENR_USART6EN_MSK         /*!< USART6 clock enable macro          */

#define RCC_APB2ENR_SPI1EN_POS     (12U)                            /*!< SPI1 clock enable bit position     */
#define RCC_APB2ENR_SPI1EN_MSK     (1U << RCC_APB2ENR_SPI1EN_POS)   /*!< SPI1 clock enable mask             */
#define RCC_APB2ENR_SPI1EN         RCC_APB2ENR_SPI1EN_MSK           /*!< SPI1 clock enable macro            */

#define RCC_APB2ENR_SYSCFGEN_POS   (14U)                            /*!< SYSCFG clock enable bit position   */
#define RCC_APB2ENR_SYSCFGEN_MSK   (1U << RCC_APB2ENR_SYSCFGEN_POS) /*!< SYSCFG clock enable mask           */
#define RCC_APB2ENR_SYSCFGEN       RCC_APB2ENR_SYSCFGEN_MSK         /*!< SYSCFG clock enable macro          */


/* ============================================================================================================ */
/*                      RCC PERIPHERAL RESET REGISTER BITS                                              	    */
/* ============================================================================================================ */
/* -------------------------------------------- RCC AHB1RSTR register bits ------------------------------------ */
#define RCC_AHB1RSTR_GPIOARST_POS   (0U)                              /*!< GPIOA peripheral reset bit position  */
#define RCC_AHB1RSTR_GPIOARST_MSK   (1U << RCC_AHB1RSTR_GPIOARST_POS) /*!< GPIOA peripheral reset mask          */
#define RCC_AHB1RSTR_GPIOARST       RCC_AHB1RSTR_GPIOARST_MSK         /*!< GPIOA peripheral reset macro         */

#define RCC_AHB1RSTR_GPIOBRST_POS   (1U)                              /*!< GPIOB peripheral reset bit position  */
#define RCC_AHB1RSTR_GPIOBRST_MSK   (1U << RCC_AHB1RSTR_GPIOBRST_POS) /*!< GPIOB peripheral reset mask          */
#define RCC_AHB1RSTR_GPIOBRST       RCC_AHB1RSTR_GPIOBRST_MSK         /*!< GPIOB peripheral reset macro         */

#define RCC_AHB1RSTR_GPIOCRST_POS   (2U)                              /*!< GPIOC peripheral reset bit position  */
#define RCC_AHB1RSTR_GPIOCRST_MSK   (1U << RCC_AHB1RSTR_GPIOCRST_POS) /*!< GPIOC peripheral reset mask          */
#define RCC_AHB1RSTR_GPIOCRST       RCC_AHB1RSTR_GPIOCRST_MSK         /*!< GPIOC peripheral reset macro         */

#define RCC_AHB1RSTR_GPIODRST_POS   (3U)                              /*!< GPIOD peripheral reset bit position  */
#define RCC_AHB1RSTR_GPIODRST_MSK   (1U << RCC_AHB1RSTR_GPIODRST_POS) /*!< GPIOD peripheral reset mask          */
#define RCC_AHB1RSTR_GPIODRST       RCC_AHB1RSTR_GPIODRST_MSK         /*!< GPIOD peripheral reset macro         */

#define RCC_AHB1RSTR_GPIOERST_POS   (4U)                              /*!< GPIOE peripheral reset bit position  */
#define RCC_AHB1RSTR_GPIOERST_MSK   (1U << RCC_AHB1RSTR_GPIOERST_POS) /*!< GPIOE peripheral reset mask          */
#define RCC_AHB1RSTR_GPIOERST       RCC_AHB1RSTR_GPIOERST_MSK         /*!< GPIOE peripheral reset macro         */

#define RCC_AHB1RSTR_GPIOFRST_POS   (5U)                              /*!< GPIOF peripheral reset bit position  */
#define RCC_AHB1RSTR_GPIOFRST_MSK   (1U << RCC_AHB1RSTR_GPIOFRST_POS) /*!< GPIOF peripheral reset mask          */
#define RCC_AHB1RSTR_GPIOFRST       RCC_AHB1RSTR_GPIOFRST_MSK         /*!< GPIOF peripheral reset macro         */

#define RCC_AHB1RSTR_GPIOGRST_POS   (6U)                              /*!< GPIOG peripheral reset bit position  */
#define RCC_AHB1RSTR_GPIOGRST_MSK   (1U << RCC_AHB1RSTR_GPIOGRST_POS) /*!< GPIOG peripheral reset mask          */
#define RCC_AHB1RSTR_GPIOGRST       RCC_AHB1RSTR_GPIOGRST_MSK         /*!< GPIOG peripheral reset macro         */

#define RCC_AHB1RSTR_GPIOHRST_POS   (7U)                              /*!< GPIOH peripheral reset bit position  */
#define RCC_AHB1RSTR_GPIOHRST_MSK   (1U << RCC_AHB1RSTR_GPIOHRST_POS) /*!< GPIOH peripheral reset mask          */
#define RCC_AHB1RSTR_GPIOHRST       RCC_AHB1RSTR_GPIOHRST_MSK         /*!< GPIOH peripheral reset macro         */

#define RCC_AHB1RSTR_GPIOIRST_POS   (8U)                              /*!< GPIOI peripheral reset bit position  */
#define RCC_AHB1RSTR_GPIOIRST_MSK   (1U << RCC_AHB1RSTR_GPIOIRST_POS) /*!< GPIOI peripheral reset mask          */
#define RCC_AHB1RSTR_GPIOIRST       RCC_AHB1RSTR_GPIOIRST_MSK         /*!< GPIOI peripheral reset macro         */


/* -------------------------------------------- RCC APB1RSTR register bits ------------------------------------ */
#define RCC_APB1RSTR_SPI2RST_POS    (14U)                             /*!< SPI2 peripheral reset bit position   */
#define RCC_APB1RSTR_SPI2RST_MSK    (1U << RCC_APB1RSTR_SPI2RST_POS)  /*!< SPI2 peripheral reset mask           */
#define RCC_APB1RSTR_SPI2RST        RCC_APB1RSTR_SPI2RST_MSK          /*!< SPI2 peripheral reset macro          */

#define RCC_APB1RSTR_SPI3RST_POS    (15U)                             /*!< SPI3 peripheral reset bit position   */
#define RCC_APB1RSTR_SPI3RST_MSK    (1U << RCC_APB1RSTR_SPI3RST_POS)  /*!< SPI3 peripheral reset mask           */
#define RCC_APB1RSTR_SPI3RST        RCC_APB1RSTR_SPI3RST_MSK          /*!< SPI3 peripheral reset macro          */

#define RCC_APB1RSTR_USART2RST_POS  (17U)                             /*!< USART2 peripheral reset bit position */
#define RCC_APB1RSTR_USART2RST_MSK  (1U << RCC_APB1RSTR_USART2RST_POS)/*!< USART2 peripheral reset mask         */
#define RCC_APB1RSTR_USART2RST      RCC_APB1RSTR_USART2RST_MSK        /*!< USART2 peripheral reset macro        */

#define RCC_APB1RSTR_USART3RST_POS  (18U)                             /*!< USART3 peripheral reset bit position */
#define RCC_APB1RSTR_USART3RST_MSK  (1U << RCC_APB1RSTR_USART3RST_POS)/*!< USART3 peripheral reset mask         */
#define RCC_APB1RSTR_USART3RST      RCC_APB1RSTR_USART3RST_MSK        /*!< USART3 peripheral reset macro        */

#define RCC_APB1RSTR_UART4RST_POS   (19U)                             /*!< UART4 peripheral reset bit position  */
#define RCC_APB1RSTR_UART4RST_MSK   (1U << RCC_APB1RSTR_UART4RST_POS) /*!< UART4 peripheral reset mask          */
#define RCC_APB1RSTR_UART4RST       RCC_APB1RSTR_UART4RST_MSK         /*!< UART4 peripheral reset macro         */

#define RCC_APB1RSTR_UART5RST_POS   (20U)                             /*!< UART5 peripheral reset bit position  */
#define RCC_APB1RSTR_UART5RST_MSK   (1U << RCC_APB1RSTR_UART5RST_POS) /*!< UART5 peripheral reset mask          */
#define RCC_APB1RSTR_UART5RST       RCC_APB1RSTR_UART5RST_MSK         /*!< UART5 peripheral reset macro         */

#define RCC_APB1RSTR_I2C1RST_POS    (21U)                             /*!< I2C1 peripheral reset bit position   */
#define RCC_APB1RSTR_I2C1RST_MSK    (1U << RCC_APB1RSTR_I2C1RST_POS)  /*!< I2C1 peripheral reset mask           */
#define RCC_APB1RSTR_I2C1RST        RCC_APB1RSTR_I2C1RST_MSK          /*!< I2C1 peripheral reset macro          */

#define RCC_APB1RSTR_I2C2RST_POS    (22U)                             /*!< I2C2 peripheral reset bit position   */
#define RCC_APB1RSTR_I2C2RST_MSK    (1U << RCC_APB1RSTR_I2C2RST_POS)  /*!< I2C2 peripheral reset mask           */
#define RCC_APB1RSTR_I2C2RST        RCC_APB1RSTR_I2C2RST_MSK          /*!< I2C2 peripheral reset macro          */

#define RCC_APB1RSTR_I2C3RST_POS    (23U)                             /*!< I2C3 peripheral reset bit position   */
#define RCC_APB1RSTR_I2C3RST_MSK    (1U << RCC_APB1RSTR_I2C3RST_POS)  /*!< I2C3 peripheral reset mask           */
#define RCC_APB1RSTR_I2C3RST        RCC_APB1RSTR_I2C3RST_MSK          /*!< I2C3 peripheral reset macro          */


/* -------------------------------------------- RCC APB2RSTR register bits ------------------------------------ */
#define RCC_APB2RSTR_USART1RST_POS  (4U)                              /*!< USART1 peripheral reset bit position */
#define RCC_APB2RSTR_USART1RST_MSK  (1U << RCC_APB2RSTR_USART1RST_POS)/*!< USART1 peripheral reset mask         */
#define RCC_APB2RSTR_USART1RST      RCC_APB2RSTR_USART1RST_MSK        /*!< USART1 peripheral reset macro        */

#define RCC_APB2RSTR_USART6RST_POS  (5U)                              /*!< USART6 peripheral reset bit position */
#define RCC_APB2RSTR_USART6RST_MSK  (1U << RCC_APB2RSTR_USART6RST_POS)/*!< USART6 peripheral reset mask         */
#define RCC_APB2RSTR_USART6RST      RCC_APB2RSTR_USART6RST_MSK        /*!< USART6 peripheral reset macro        */

#define RCC_APB2RSTR_SPI1RST_POS    (12U)                             /*!< SPI1 peripheral reset bit position   */
#define RCC_APB2RSTR_SPI1RST_MSK    (1U << RCC_APB2RSTR_SPI1RST_POS)  /*!< SPI1 peripheral reset mask           */
#define RCC_APB2RSTR_SPI1RST        RCC_APB2RSTR_SPI1RST_MSK          /*!< SPI1 peripheral reset macro          */

#define RCC_APB2RSTR_SYSCFGRST_POS  (14U)                             /*!< SYSCFG peripheral reset bit position */
#define RCC_APB2RSTR_SYSCFGRST_MSK  (1U << RCC_APB2RSTR_SYSCFGRST_POS)/*!< SYSCFG peripheral reset mask         */
#define RCC_APB2RSTR_SYSCFGRST      RCC_APB2RSTR_SYSCFGRST_MSK        /*!< SYSCFG peripheral reset macro        */



/* ===================================================================================== */
/*                        I2C REGISTER BIT DEFINITIONS                             		 */
/* ===================================================================================== */

/* --------------------------------- I2C_CR1 register bits ----------------------------- */

/* Bit positions */
#define I2C_CR1_PE_POS           (0U)    /*!< Peripheral enable                          */
#define I2C_CR1_SMBUS_POS        (1U)    /*!< SMBus mode                                 */
#define I2C_CR1_SMBTYPE_POS      (3U)    /*!< SMBus type                                 */
#define I2C_CR1_ENARP_POS        (4U)    /*!< ARP enable                                 */
#define I2C_CR1_ENPEC_POS        (5U)    /*!< PEC enable                                 */
#define I2C_CR1_ENGC_POS         (6U)    /*!< General call enable                        */
#define I2C_CR1_NOSTRETCH_POS    (7U)    /*!< Clock stretching disable (slave mode)      */
#define I2C_CR1_START_POS        (8U)    /*!< Start generation                           */
#define I2C_CR1_STOP_POS         (9U)    /*!< Stop generation                            */
#define I2C_CR1_ACK_POS          (10U)   /*!< Acknowledge enable                         */
#define I2C_CR1_POS_POS          (11U)   /*!< Acknowledge/PEC position (data reception)  */
#define I2C_CR1_PEC_POS          (12U)   /*!< Packet error checking                      */
#define I2C_CR1_ALERT_POS        (13U)   /*!< SMBus alert                                */
#define I2C_CR1_SWRST_POS        (15U)   /*!< Software reset                             */

/* Bit masks */
#define I2C_CR1_PE_MSK           (1U << I2C_CR1_PE_POS)
#define I2C_CR1_SMBUS_MSK        (1U << I2C_CR1_SMBUS_POS)
#define I2C_CR1_SMBTYPE_MSK      (1U << I2C_CR1_SMBTYPE_POS)
#define I2C_CR1_ENARP_MSK        (1U << I2C_CR1_ENARP_POS)
#define I2C_CR1_ENPEC_MSK        (1U << I2C_CR1_ENPEC_POS)
#define I2C_CR1_ENGC_MSK         (1U << I2C_CR1_ENGC_POS)
#define I2C_CR1_NOSTRETCH_MSK    (1U << I2C_CR1_NOSTRETCH_POS)
#define I2C_CR1_START_MSK        (1U << I2C_CR1_START_POS)
#define I2C_CR1_STOP_MSK         (1U << I2C_CR1_STOP_POS)
#define I2C_CR1_ACK_MSK          (1U << I2C_CR1_ACK_POS)
#define I2C_CR1_POS_MSK          (1U << I2C_CR1_POS_POS)
#define I2C_CR1_PEC_MSK          (1U << I2C_CR1_PEC_POS)
#define I2C_CR1_ALERT_MSK        (1U << I2C_CR1_ALERT_POS)
#define I2C_CR1_SWRST_MSK        (1U << I2C_CR1_SWRST_POS)


/* --------------------------------- I2C_CR2 register bits ----------------------------- */

/* Bit positions */
#define I2C_CR2_FREQ_POS         (0U)    /*!< Peripheral clock frequency                 */
#define I2C_CR2_ITERREN_POS      (8U)    /*!< Error interrupt enable                     */
#define I2C_CR2_ITEVTEN_POS      (9U)    /*!< Event interrupt enable                     */
#define I2C_CR2_ITBUFEN_POS      (10U)   /*!< Buffer interrupt enable                    */
#define I2C_CR2_DMAEN_POS        (11U)   /*!< DMA requests enable                        */
#define I2C_CR2_LAST_POS         (12U)   /*!< DMA last transfer                          */

/* Bit masks */
#define I2C_CR2_FREQ_MSK         (0x3FU << I2C_CR2_FREQ_POS)
#define I2C_CR2_ITERREN_MSK      (1U << I2C_CR2_ITERREN_POS)
#define I2C_CR2_ITEVTEN_MSK      (1U << I2C_CR2_ITEVTEN_POS)
#define I2C_CR2_ITBUFEN_MSK      (1U << I2C_CR2_ITBUFEN_POS)
#define I2C_CR2_DMAEN_MSK        (1U << I2C_CR2_DMAEN_POS)
#define I2C_CR2_LAST_MSK         (1U << I2C_CR2_LAST_POS)


/* --------------------------------- I2C_OAR1 register bits ----------------------------- */

/* Bit positions */
#define I2C_OAR1_ADD0_POS        (0U)    /*!< Interface address bit 0                     */
#define I2C_OAR1_ADD7_1_POS      (1U)    /*!< Interface address bits 7:1                  */
#define I2C_OAR1_ADD9_8_POS      (8U)    /*!< Interface address bits 9:8                  */
#define I2C_OAR1_ADDMODE_POS     (15U)   /*!< Addressing mode (7-bit / 10-bit)            */

/* Bit masks */
#define I2C_OAR1_ADD0_MSK        (1U << I2C_OAR1_ADD0_POS)
#define I2C_OAR1_ADD7_1_MSK      (0x7FU << I2C_OAR1_ADD7_1_POS)
#define I2C_OAR1_ADD9_8_MSK      (0x3U << I2C_OAR1_ADD9_8_POS)
#define I2C_OAR1_ADDMODE_MSK     (1U << I2C_OAR1_ADDMODE_POS)


/* --------------------------------- I2C_SR1 register bits ----------------------------- */

/* Bit positions */
#define I2C_SR1_SB_POS           (0U)    /*!< Start bit (Master mode)                    */
#define I2C_SR1_ADDR_POS         (1U)    /*!< Address sent/matched                       */
#define I2C_SR1_BTF_POS          (2U)    /*!< Byte transfer finished                     */
#define I2C_SR1_ADD10_POS        (3U)    /*!< 10-bit header sent                         */
#define I2C_SR1_STOPF_POS        (4U)    /*!< Stop detection (slave mode)                */
#define I2C_SR1_RXNE_POS         (6U)    /*!< Data register not empty (receivers)        */
#define I2C_SR1_TXE_POS          (7U)    /*!< Data register empty (transmitters)         */
#define I2C_SR1_BERR_POS         (8U)    /*!< Bus error                                  */
#define I2C_SR1_ARLO_POS         (9U)    /*!< Arbitration lost                           */
#define I2C_SR1_AF_POS           (10U)   /*!< Acknowledge failure                        */
#define I2C_SR1_OVR_POS          (11U)   /*!< Overrun/Underrun                           */
#define I2C_SR1_PECERR_POS       (12U)   /*!< PEC error in reception                     */
#define I2C_SR1_TIMEOUT_POS      (14U)   /*!< Timeout or Tlow error                      */
#define I2C_SR1_SMBALERT_POS     (15U)   /*!< SMBus alert                                */

/* Bit masks */
#define I2C_SR1_SB_MSK           (1U << I2C_SR1_SB_POS)
#define I2C_SR1_ADDR_MSK         (1U << I2C_SR1_ADDR_POS)
#define I2C_SR1_BTF_MSK          (1U << I2C_SR1_BTF_POS)
#define I2C_SR1_ADD10_MSK        (1U << I2C_SR1_ADD10_POS)
#define I2C_SR1_STOPF_MSK        (1U << I2C_SR1_STOPF_POS)
#define I2C_SR1_RXNE_MSK         (1U << I2C_SR1_RXNE_POS)
#define I2C_SR1_TXE_MSK          (1U << I2C_SR1_TXE_POS)
#define I2C_SR1_BERR_MSK         (1U << I2C_SR1_BERR_POS)
#define I2C_SR1_ARLO_MSK         (1U << I2C_SR1_ARLO_POS)
#define I2C_SR1_AF_MSK           (1U << I2C_SR1_AF_POS)
#define I2C_SR1_OVR_MSK          (1U << I2C_SR1_OVR_POS)
#define I2C_SR1_PECERR_MSK       (1U << I2C_SR1_PECERR_POS)
#define I2C_SR1_TIMEOUT_MSK      (1U << I2C_SR1_TIMEOUT_POS)
#define I2C_SR1_SMBALERT_MSK     (1U << I2C_SR1_SMBALERT_POS)


/* --------------------------------- I2C_SR2 register bits ----------------------------- */

/* Bit positions */
#define I2C_SR2_MSL_POS          (0U)    /*!< Master/slave                               */
#define I2C_SR2_BUSY_POS         (1U)    /*!< Bus busy                                   */
#define I2C_SR2_TRA_POS          (2U)    /*!< Transmitter/receiver                       */
#define I2C_SR2_GENCALL_POS      (4U)    /*!< General call address received              */
#define I2C_SR2_SMBDEFAULT_POS   (5U)    /*!< SMBus device default address               */
#define I2C_SR2_SMBHOST_POS      (6U)    /*!< SMBus host header                          */
#define I2C_SR2_DUALF_POS        (7U)    /*!< Dual flag                                  */
#define I2C_SR2_PEC_POS          (8U)    /*!< Packet error checking register             */

/* Bit masks */
#define I2C_SR2_MSL_MSK          (1U << I2C_SR2_MSL_POS)
#define I2C_SR2_BUSY_MSK         (1U << I2C_SR2_BUSY_POS)
#define I2C_SR2_TRA_MSK          (1U << I2C_SR2_TRA_POS)
#define I2C_SR2_GENCALL_MSK      (1U << I2C_SR2_GENCALL_POS)
#define I2C_SR2_SMBDEFAULT_MSK   (1U << I2C_SR2_SMBDEFAULT_POS)
#define I2C_SR2_SMBHOST_MSK      (1U << I2C_SR2_SMBHOST_POS)
#define I2C_SR2_DUALF_MSK        (1U << I2C_SR2_DUALF_POS)
#define I2C_SR2_PEC_MSK          (0xFFU << I2C_SR2_PEC_POS)


/* --------------------------------- I2C_CCR register bits ----------------------------- */

/* Bit positions */
#define I2C_CCR_CCR_POS          (0U)    /*!< Clock control bits                         */
#define I2C_CCR_DUTY_POS         (14U)   /*!< Fast mode duty cycle                       */
#define I2C_CCR_FS_POS           (15U)   /*!< Fast mode selection                        */

/* Bit masks */
#define I2C_CCR_CCR_MSK          (0xFFFU << I2C_CCR_CCR_POS)
#define I2C_CCR_DUTY_MSK         (1U << I2C_CCR_DUTY_POS)
#define I2C_CCR_FS_MSK           (1U << I2C_CCR_FS_POS)

/* =============================================================================== */
/*                      SPI REGISTER BIT DEFINITIONS                               */
/* =============================================================================== */

/* ------------------------------ SPI_CR1 register bits -------------------------- */

/* Bit positions */
#define SPI_CR1_CPHA_POS       (0U)  /*!< Clock phase (1st or 2nd edge)            */
#define SPI_CR1_CPOL_POS       (1U)  /*!< Clock polarity (low or high)             */
#define SPI_CR1_MSTR_POS       (2U)  /*!< Master selection                         */
#define SPI_CR1_BR_POS         (3U)  /*!< Baud rate control bits [5:3]             */
#define SPI_CR1_SPE_POS        (6U)  /*!< SPI enable                               */
#define SPI_CR1_LSBFIRST_POS   (7U)  /*!< Frame format: LSB/MSB first              */
#define SPI_CR1_SSI_POS        (8U)  /*!< Internal slave select                    */
#define SPI_CR1_SSM_POS        (9U)  /*!< Software slave management                */
#define SPI_CR1_RXONLY_POS     (10U) /*!< Receive only mode enable                 */
#define SPI_CR1_DFF_POS        (11U) /*!< Data frame format (8-bit or 16-bit)      */
#define SPI_CR1_CRCNEXT_POS    (12U) /*!< Transmit CRC next                        */
#define SPI_CR1_CRCEN_POS      (13U) /*!< Hardware CRC calculation enable          */
#define SPI_CR1_BIDIOE_POS     (14U) /*!< Output enable in bidirectional mode      */
#define SPI_CR1_BIDIMODE_POS   (15U) /*!< Bidirectional data mode enable           */

/* Bit masks */
#define SPI_CR1_CPHA_MSK       (1U << SPI_CR1_CPHA_POS)
#define SPI_CR1_CPOL_MSK       (1U << SPI_CR1_CPOL_POS)
#define SPI_CR1_MSTR_MSK       (1U << SPI_CR1_MSTR_POS)
#define SPI_CR1_BR_MSK         (0x7U << SPI_CR1_BR_POS)
#define SPI_CR1_SPE_MSK        (1U << SPI_CR1_SPE_POS)
#define SPI_CR1_LSBFIRST_MSK   (1U << SPI_CR1_LSBFIRST_POS)
#define SPI_CR1_SSI_MSK        (1U << SPI_CR1_SSI_POS)
#define SPI_CR1_SSM_MSK        (1U << SPI_CR1_SSM_POS)
#define SPI_CR1_RXONLY_MSK     (1U << SPI_CR1_RXONLY_POS)
#define SPI_CR1_DFF_MSK        (1U << SPI_CR1_DFF_POS)
#define SPI_CR1_CRCNEXT_MSK    (1U << SPI_CR1_CRCNEXT_POS)
#define SPI_CR1_CRCEN_MSK      (1U << SPI_CR1_CRCEN_POS)
#define SPI_CR1_BIDIOE_MSK     (1U << SPI_CR1_BIDIOE_POS)
#define SPI_CR1_BIDIMODE_MSK   (1U << SPI_CR1_BIDIMODE_POS)


/* ------------------------------ SPI_CR2 register bits -------------------------- */

/* Bit positions */
#define SPI_CR2_RXDMAEN_POS    (0U)  /*!< Rx buffer DMA enable                     */
#define SPI_CR2_TXDMAEN_POS    (1U)  /*!< Tx buffer DMA enable                     */
#define SPI_CR2_SSOE_POS       (2U)  /*!< SS output enable (master mode)           */
#define SPI_CR2_FRF_POS        (4U)  /*!< Frame format (TI mode select)            */
#define SPI_CR2_ERRIE_POS      (5U)  /*!< Error interrupt enable                   */
#define SPI_CR2_RXNEIE_POS     (6U)  /*!< RX buffer not empty interrupt enable     */
#define SPI_CR2_TXEIE_POS      (7U)  /*!< TX buffer empty interrupt enable         */

/* Bit masks */
#define SPI_CR2_RXDMAEN_MSK    (1U << SPI_CR2_RXDMAEN_POS)
#define SPI_CR2_TXDMAEN_MSK    (1U << SPI_CR2_TXDMAEN_POS)
#define SPI_CR2_SSOE_MSK       (1U << SPI_CR2_SSOE_POS)
#define SPI_CR2_FRF_MSK        (1U << SPI_CR2_FRF_POS)
#define SPI_CR2_ERRIE_MSK      (1U << SPI_CR2_ERRIE_POS)
#define SPI_CR2_RXNEIE_MSK     (1U << SPI_CR2_RXNEIE_POS)
#define SPI_CR2_TXEIE_MSK      (1U << SPI_CR2_TXEIE_POS)


/* ------------------------------ SPI_SR register bits --------------------------- */

/* Bit positions */
#define SPI_SR_RXNE_POS        (0U)  /*!< Receive buffer not empty flag            */
#define SPI_SR_TXE_POS         (1U)  /*!< Transmit buffer empty flag               */
#define SPI_SR_CHSIDE_POS      (2U)  /*!< Channel side flag (left/right)           */
#define SPI_SR_UDR_POS         (3U)  /*!< Underrun flag (slave mode)               */
#define SPI_SR_CRCERR_POS      (4U)  /*!< CRC error flag                           */
#define SPI_SR_MODF_POS        (5U)  /*!< Mode fault flag                          */
#define SPI_SR_OVR_POS         (6U)  /*!< Overrun flag                             */
#define SPI_SR_BSY_POS         (7U)  /*!< Busy flag                                */

/* Bit masks */
#define SPI_SR_RXNE_MSK        (1U << SPI_SR_RXNE_POS)
#define SPI_SR_TXE_MSK         (1U << SPI_SR_TXE_POS)
#define SPI_SR_CHSIDE_MSK      (1U << SPI_SR_CHSIDE_POS)
#define SPI_SR_UDR_MSK         (1U << SPI_SR_UDR_POS)
#define SPI_SR_CRCERR_MSK      (1U << SPI_SR_CRCERR_POS)
#define SPI_SR_MODF_MSK        (1U << SPI_SR_MODF_POS)
#define SPI_SR_OVR_MSK         (1U << SPI_SR_OVR_POS)
#define SPI_SR_BSY_MSK         (1U << SPI_SR_BSY_POS)

/* ========================================================================== */
/*                      USART REGISTER BIT DEFINITIONS                        */
/* ========================================================================== */

/* ---------------------------- USART_SR register bits ---------------------- */

/* Bit positions */
#define USART_SR_PE_POS        (0U)   /*!< Parity error flag                  */
#define USART_SR_FE_POS        (1U)   /*!< Framing error flag                 */
#define USART_SR_NF_POS        (2U)   /*!< Noise detected flag                */
#define USART_SR_ORE_POS       (3U)   /*!< Overrun error flag                 */
#define USART_SR_IDLE_POS      (4U)   /*!< IDLE line detected flag            */
#define USART_SR_RXNE_POS      (5U)   /*!< Read data register not empty flag  */
#define USART_SR_TC_POS        (6U)   /*!< Transmission complete flag         */
#define USART_SR_TXE_POS       (7U)   /*!< Transmit data register empty flag  */
#define USART_SR_LBD_POS       (8U)   /*!< LIN break detection flag           */
#define USART_SR_CTS_POS       (9U)   /*!< CTS flag                           */

/* Bit masks */
#define USART_SR_PE_MSK        (1U << USART_SR_PE_POS)
#define USART_SR_FE_MSK        (1U << USART_SR_FE_POS)
#define USART_SR_NF_MSK        (1U << USART_SR_NF_POS)
#define USART_SR_ORE_MSK       (1U << USART_SR_ORE_POS)
#define USART_SR_IDLE_MSK      (1U << USART_SR_IDLE_POS)
#define USART_SR_RXNE_MSK      (1U << USART_SR_RXNE_POS)
#define USART_SR_TC_MSK        (1U << USART_SR_TC_POS)
#define USART_SR_TXE_MSK       (1U << USART_SR_TXE_POS)
#define USART_SR_LBD_MSK       (1U << USART_SR_LBD_POS)
#define USART_SR_CTS_MSK       (1U << USART_SR_CTS_POS)


/* ---------------------------- USART_CR1 register bits --------------------- */

/* Bit positions */
#define USART_CR1_SBK_POS      (0U)   /*!< Send break 						  */
#define USART_CR1_RWU_POS      (1U)   /*!< Receiver wakeup					  */
#define USART_CR1_RE_POS       (2U)   /*!< Receiver enable 					  */
#define USART_CR1_TE_POS       (3U)   /*!< Transmitter enable				  */
#define USART_CR1_IDLEIE_POS   (4U)   /*!< IDLE interrupt enable			  */
#define USART_CR1_RXNEIE_POS   (5U)   /*!< RXNE interrupt enable 			  */
#define USART_CR1_TCIE_POS     (6U)   /*!< Transmission complete interrupt en */
#define USART_CR1_TXEIE_POS    (7U)   /*!< TXE interrupt enable 			  */
#define USART_CR1_PEIE_POS     (8U)   /*!< Parity error interrupt enable	  */
#define USART_CR1_PS_POS       (9U)   /*!< Parity selection 				  */
#define USART_CR1_PCE_POS      (10U)  /*!< Parity control enable 			  */
#define USART_CR1_WAKE_POS     (11U)  /*!< Wakeup method 					  */
#define USART_CR1_M_POS        (12U)  /*!< Word length 						  */
#define USART_CR1_UE_POS       (13U)  /*!< USART enable 					  */
#define USART_CR1_OVER8_POS    (15U)  /*!< Oversampling mode */

/* Bit masks */
#define USART_CR1_SBK_MSK       (1U << USART_CR1_SBK_POS)
#define USART_CR1_RWU_MSK       (1U << USART_CR1_RWU_POS)
#define USART_CR1_RE_MSK        (1U << USART_CR1_RE_POS)
#define USART_CR1_TE_MSK        (1U << USART_CR1_TE_POS)
#define USART_CR1_IDLEIE_MSK    (1U << USART_CR1_IDLEIE_POS)
#define USART_CR1_RXNEIE_MSK    (1U << USART_CR1_RXNEIE_POS)
#define USART_CR1_TCIE_MSK      (1U << USART_CR1_TCIE_POS)
#define USART_CR1_TXEIE_MSK     (1U << USART_CR1_TXEIE_POS)
#define USART_CR1_PEIE_MSK      (1U << USART_CR1_PEIE_POS)
#define USART_CR1_PS_MSK        (1U << USART_CR1_PS_POS)
#define USART_CR1_PCE_MSK       (1U << USART_CR1_PCE_POS)
#define USART_CR1_WAKE_MSK      (1U << USART_CR1_WAKE_POS)
#define USART_CR1_M_MSK         (1U << USART_CR1_M_POS)
#define USART_CR1_UE_MSK        (1U << USART_CR1_UE_POS)
#define USART_CR1_OVER8_MSK     (1U << USART_CR1_OVER8_POS)


/* ---------------------------- USART_CR2 register bits --------------------- */

/* Bit positions */
#define USART_CR2_ADD_POS      (0U)   /*!< Address of the USART node          */
#define USART_CR2_LBDL_POS     (5U)   /*!< LIN break detection length 		  */
#define USART_CR2_LBDIE_POS    (6U)   /*!< LIN break detection interrupt enab */
#define USART_CR2_LBCL_POS     (8U)   /*!< Last bit clock pulse  			  */
#define USART_CR2_CPHA_POS     (9U)   /*!< Clock phase                        */
#define USART_CR2_CPOL_POS     (10U)  /*!< Clock polarity                     */
#define USART_CR2_CLKEN_POS    (11U)  /*!< Clock enable                       */
#define USART_CR2_STOP_POS     (12U)  /*!< STOP bits                          */
#define USART_CR2_LINEN_POS    (14U)  /*!< LIN mode enable                    */


/* Bit masks */
#define USART_CR2_ADD_MSK         (0xFU << USART_CR2_ADD_POS)
#define USART_CR2_LBDL_MSK        (1U << USART_CR2_LBDL_POS)
#define USART_CR2_LBDIE_MSK       (1U << USART_CR2_LBDIE_POS)
#define USART_CR2_LBCL_MSK        (1U << USART_CR2_LBCL_POS)
#define USART_CR2_CPHA_MSK        (1U << USART_CR2_CPHA_POS)
#define USART_CR2_CPOL_MSK        (1U << USART_CR2_CPOL_POS)
#define USART_CR2_CLKEN_MSK       (1U << USART_CR2_CLKEN_POS)
#define USART_CR2_STOP_MSK        (0x3U << USART_CR2_STOP_POS)
#define USART_CR2_LINEN_MSK       (1U << USART_CR2_LINEN_POS)


/* ---------------------------- USART_CR3 register bits --------------------- */

/* Bit positions */
#define USART_CR3_EIE_POS         (0U)   /*!< Error interrupt enable          */
#define USART_CR3_IREN_POS        (1U)   /*!< IrDA mode enable                */
#define USART_CR3_IRLP_POS        (2U)   /*!< IrDA low-power mode             */
#define USART_CR3_HDSEL_POS       (3U)   /*!< Half-duplex mode enable         */
#define USART_CR3_NACK_POS        (4U)   /*!< Smartcard NACK enable           */
#define USART_CR3_SCEN_POS        (5U)   /*!< Smartcard mode enable           */
#define USART_CR3_DMAR_POS        (6U)   /*!< DMA enable receiver             */
#define USART_CR3_DMAT_POS        (7U)   /*!< DMA enable transmitter          */
#define USART_CR3_RTSE_POS        (8U)   /*!< RTS enable (hrdwr flow control) */
#define USART_CR3_CTSE_POS        (9U)   /*!< CTS enable (hrdwr flow control) */
#define USART_CR3_CTSIE_POS       (10U)  /*!< CTS interrupt enable            */
#define USART_CR3_ONEBIT_POS      (11U)  /*!< One sample bit method enable    */

/* Bit masks */
#define USART_CR3_EIE_MSK         (1U << USART_CR3_EIE_POS)
#define USART_CR3_IREN_MSK        (1U << USART_CR3_IREN_POS)
#define USART_CR3_IRLP_MSK        (1U << USART_CR3_IRLP_POS)
#define USART_CR3_HDSEL_MSK       (1U << USART_CR3_HDSEL_POS)
#define USART_CR3_NACK_MSK        (1U << USART_CR3_NACK_POS)
#define USART_CR3_SCEN_MSK        (1U << USART_CR3_SCEN_POS)
#define USART_CR3_DMAR_MSK        (1U << USART_CR3_DMAR_POS)
#define USART_CR3_DMAT_MSK        (1U << USART_CR3_DMAT_POS)
#define USART_CR3_RTSE_MSK        (1U << USART_CR3_RTSE_POS)
#define USART_CR3_CTSE_MSK        (1U << USART_CR3_CTSE_POS)
#define USART_CR3_CTSIE_MSK       (1U << USART_CR3_CTSIE_POS)
#define USART_CR3_ONEBIT_MSK      (1U << USART_CR3_ONEBIT_POS)



/* CTRL bits */
#define SYSTICK_CTRL_ENABLE     (1U << 0)
#define SYSTICK_CTRL_TICKINT    (1U << 1)
#define SYSTICK_CTRL_CLKSOURCE  (1U << 2)


/* SysTick IRQ priority register (SCB->SHPR3[31:24]) */
#define SCB_SHPR3               (*(volatile uint32_t *)0xE000ED20UL)

#ifdef __cplusplus
}
#endif

#endif /* INC_STM32F407XX_H_ */
