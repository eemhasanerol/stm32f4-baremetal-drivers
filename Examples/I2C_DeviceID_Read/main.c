/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Example project communicating with an I2C device (e.g. MPU6050)
 *                   via I2C1 (PB6=SCL, PB7=SDA);
 *                   LED blinks fast if the device is found, slow if not.
 ******************************************************************************
 */

#include "stm32f407xx.h"
#include "rcc.h"
#include "gpio.h"
#include "systick.h"
#include "i2c.h"

/* Default system clock frequency (HSI = 16 MHz) */
uint32_t SystemCoreClock = 16000000U;

/* MPU6050 definitions */
#define MPU6050_ADDR       0x68U		/* AD0 = GND -> 0x68, AD0 = VCC -> 0x69 */
#define MPU6050_WHO_AM_I   0x75U

/* Function prototypes */
static void GPIO_I2C1_Init(void);
static void GPIO_LedInit(void);

int main(void)
{
    SysTick_Init(1000);  // 1 ms tick

    GPIO_I2C1_Init();
    GPIO_LedInit();

    /* Configure I2C handle */
    I2C_HandleTypeDef_t hi2c1 = {0};
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeedHz = I2C_SPEED_STANDARD;
    hi2c1.Init.AckState     = I2C_ACK_ENABLE;
    hi2c1.Init.DutyCycle    = I2C_FM_DUTY_2;
    hi2c1.Init.TimeoutMs    = 1000U;

    I2C_Init(&hi2c1);

    /* Buffer for read */
    uint8_t who_am_i = 0x00U;

    /* Read WHO_AM_I register */
    status_t status = I2C_MemRead(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I, &who_am_i, 1U);

    if (status == STATUS_OK && who_am_i == 0x68U)
    {
        /* Blink LED fast if MPU6050 detected */
        while (1)
        {
            GPIO_TogglePin(GPIOD, GPIO_PIN_13);
            delay_ms(200);
        }
    }
    else
    {
        /* Blink LED slow if device not found */
        while (1)
        {
            GPIO_TogglePin(GPIOD, GPIO_PIN_13);
            delay_ms(1000);
        }
    }
}

/**
 * @brief Configure GPIO pins PB6 (SCL) and PB7 (SDA) for I2C1
 */
static void GPIO_I2C1_Init(void)
{
    GPIO_InitTypeDef_t gpio_i2c = {0};

    RCC_GPIOB_CLK_ENABLE();
    RCC_I2C1_CLK_ENABLE();

    gpio_i2c.Pin       = (GPIO_PIN_6 | GPIO_PIN_7);
    gpio_i2c.Mode      = GPIO_MODE_AF;
    gpio_i2c.OType     = GPIO_OTYPE_OD;
    gpio_i2c.PuPd      = GPIO_PULLUP;
    gpio_i2c.Speed     = GPIO_SPEED_HIGH;
    gpio_i2c.Alternate = GPIO_AF4_I2C1_3;

    GPIO_Init(GPIOB, &gpio_i2c);
}

/**
 * @brief Configure LED (PD13)
 */
static void GPIO_LedInit(void)
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
