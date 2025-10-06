/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : SPI1 ↔ MFRC522 example — reads VersionReg (0x37).
 *                   PA5=SCK, PA6=MISO, PA7=MOSI, PB0=CS.
 *                   LED PD13 blinks fast if device found, stays off otherwise.
 ******************************************************************************
 */

#include "stm32f407xx.h"
#include "rcc.h"
#include "gpio.h"
#include "systick.h"
#include "spi.h"

/* 16 MHz HSI clock */
uint32_t SystemCoreClock = 16000000U;

/* --- MFRC522 connections --- */
#define RC522_CS_PORT   GPIOB
#define RC522_CS_PIN    GPIO_PIN_0
#define RC522_CS_LOW()  GPIO_WritePin(RC522_CS_PORT, RC522_CS_PIN, GPIO_PIN_RESET)
#define RC522_CS_HIGH() GPIO_WritePin(RC522_CS_PORT, RC522_CS_PIN, GPIO_PIN_SET)

#define MFRC522_REG_VERSION   0x37U
#define MFRC522_VER_091        0x91U
#define MFRC522_VER_092        0x92U

/* Function prototypes */
static void SPI1_Init(void);
static void GPIO_LedInit(void);
static uint8_t RC522_ReadReg(SPI_HandleTypeDef_t *hspi, uint8_t reg);

/* SPI handle */
static SPI_HandleTypeDef_t hspi1;

int main(void)
{
    SysTick_Init(1000);   /* 1 ms SysTick delay */
    SPI1_Init();          /* SPI + GPIO + RCC setup */
    GPIO_LedInit();       /* LED setup */

    /* Read Version register (0x37), expect 0x91 or 0x92 */
    uint8_t version = RC522_ReadReg(&hspi1, MFRC522_REG_VERSION);

    while (1)
    {
        if ((version == MFRC522_VER_091) || (version == MFRC522_VER_092))
        {
            GPIO_TogglePin(GPIOD, GPIO_PIN_13);  /* Device found */
            delay_ms(200);
        }
        else
        {
            GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET); /* Off */
        }
    }
}

/* -------------------------------------------------------------------------- */
/*                            Peripheral Setup                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief Configure SPI1 (PA5=SCK, PA6=MISO, PA7=MOSI, PB0=CS)
 */
static void SPI1_Init(void)
{
    GPIO_InitTypeDef_t gpio = {0};

    /* Enable clocks */
    RCC_GPIOA_CLK_ENABLE();
    RCC_GPIOB_CLK_ENABLE();
    RCC_SPI1_CLK_ENABLE();

    /* PA5,6,7 → SPI1 AF5 */
    gpio.Pin       = (GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);
    gpio.Mode      = GPIO_MODE_AF;
    gpio.OType     = GPIO_OTYPE_PP;
    gpio.PuPd      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1_2;
    GPIO_Init(GPIOA, &gpio);

    /* PB0 → CS pin */
    gpio.Pin   = RC522_CS_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT;
    gpio.OType = GPIO_OTYPE_PP;
    gpio.PuPd  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_HIGH;
    GPIO_Init(RC522_CS_PORT, &gpio);
    RC522_CS_HIGH();

    /* SPI1 configuration */
    hspi1.Instance        = SPI1;
    hspi1.Init.DeviceMode = SPI_DEVICE_MODE_MASTER;
    hspi1.Init.BusConfig  = SPI_BUS_FD;
    hspi1.Init.BaudRate   = SPI_BAUD_DIV16;      /* ~1 MHz */
    hspi1.Init.DataSize   = SPI_DATASIZE_8BIT;
    hspi1.Init.CPOL       = SPI_CPOL_LOW;
    hspi1.Init.CPHA       = SPI_CPHA_1EDGE;
    hspi1.Init.SSM        = SPI_SSM_ENABLE;
    hspi1.Init.TimeoutMs  = 1000U;
    SPI_Init(&hspi1);
}

/**
 * @brief Configure PD13 LED
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

/* -------------------------------------------------------------------------- */
/*                           RC522 SPI helper                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Read one register from MFRC522 via SPI
 */
static uint8_t RC522_ReadReg(SPI_HandleTypeDef_t *hspi, uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0};

    /* Read command format: [addr<<1 & 0x7E] | 0x80 */
    tx[0] = (uint8_t)(((reg << 1) & 0x7EU) | 0x80U);
    tx[1] = 0x00U;

    RC522_CS_LOW();
    SPI_TransmitReceive(hspi, tx, rx, 2);
    RC522_CS_HIGH();

    return rx[1];
}
