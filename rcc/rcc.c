/**
 * @file    rcc.c
 * @brief   Reset and Clock Control (RCC) driver implementation for STM32F407
 * @author  Hasan Erol
 */

#include "rcc.h"

/* -------------------------------------------------------------------------- */
/* RCC Prescaler Tables                                                       */
/* -------------------------------------------------------------------------- */
static const uint16_t AHB_PreScaler[8] =  {2,4,8,16,64,128,256,512};
static const uint8_t  APB1_PreScaler[4] = {2,4,8,16};


/**
 * @brief  Returns the APB1 peripheral clock frequency (PCLK1).
 * @note   This function reads current clock configuration from RCC registers
 *         and computes PCLK1 frequency based on AHB and APB1 prescalers.
 * @retval uint32_t  PCLK1 frequency in Hz
 */

uint32_t RCC_GetPCLK1Freq(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clksrc, tempValue, ahbp, apb1p;

	clksrc = ((RCC->CFGR>> 2U) & 0x3U);

	if(clksrc == 0)
	{
		SystemClk = 16000000U;
	}else if(clksrc == 1U)
	{
		SystemClk = 8000000U;
	}else if(clksrc == 2U)
	{
		SystemClk = RCC_GetPLLOutputClockFreq();
	}

	// for ahbp
	tempValue = ((RCC->CFGR >> 4U) & 0xFU);

	if(tempValue < 8U)
	{
		ahbp = 1U;

	}else
	{
		ahbp = AHB_PreScaler[tempValue - 8U];

	}


	// for apb1
	tempValue = ((RCC->CFGR >> 10U) & 0x7U);

	if(tempValue < 4U)
	{
		apb1p = 1U;

	}else
	{
		apb1p  = APB1_PreScaler[tempValue - 4U];
	}

	pclk1 = ((SystemClk / ahbp) /apb1p);

	return pclk1;
}

/**
 * @brief  Computes the PLL output clock frequency (PLLCLK).
 * @note   Formula:
 *         PLLCLK = ((Input clock / PLL_M) * PLL_N) / PLL_P
 * @retval uint32_t  PLL output frequency in Hz
 */

uint32_t RCC_GetPLLOutputClockFreq(void)
{
    /* PLL input source: 0 = HSI (16 MHz), 1 = HSE (HSE_VALUE) */
    uint32_t pll_src = (RCC->PLLCFGR >> 22U) & 0x1U;
    uint32_t pll_in_hz = (pll_src == 0U) ? 16000000U : (uint32_t)8000000;

    /* PLL factors */
    uint32_t pll_m =  (RCC->PLLCFGR & 0x3FU);            /* [5:0]  */
    uint32_t pll_n = ((RCC->PLLCFGR >> 6U) & 0x1FFU);    /* [14:6] */
    uint32_t pll_p_bits = ((RCC->PLLCFGR >> 16U) & 0x3U);/* [17:16] */
    uint32_t pll_p_div = (pll_p_bits + 1U) * 2U;         /* 00->2, 01->4, 10->6, 11->8 */

    /* Guard: avoid div-by-zero (shouldn't happen with valid config) */
    if (pll_m == 0U || pll_p_div == 0U) {
        return 0U;
    }

    uint64_t vco_in  = ((uint64_t)pll_in_hz) / (uint64_t)pll_m;
    uint64_t vco_out = vco_in * (uint64_t)pll_n;
    uint64_t pllclk  = vco_out / (uint64_t)pll_p_div;

    return (uint32_t)pllclk;
}


/**
 * @brief  Returns the APB2 peripheral clock frequency (PCLK2).
 * @note   This function reads current clock configuration from RCC registers
 *         and computes PCLK2 frequency based on AHB and APB2 prescalers.
 * @retval uint32_t  PCLK2 frequency in Hz
 */

uint32_t RCC_GetPCLK2Freq(void)
{
    uint32_t pclk2, SystemClk;
    uint8_t clksrc, tempValue, ahbp, apb2p;

    /* Get system clock source */
    clksrc = ((RCC->CFGR >> 2U) & 0x3U);

    if (clksrc == 0U)
    {
        SystemClk = 16000000U;  /* HSI */
    }
    else if (clksrc == 1U)
    {
        SystemClk = 8000000U;   /* HSE */
    }
    else if (clksrc == 2U)
    {
        SystemClk = RCC_GetPLLOutputClockFreq(); /* PLL */
    }
    else
    {
        SystemClk = 16000000U; /* Default safe fallback */
    }

    /* AHB prescaler */
    tempValue = ((RCC->CFGR >> 4U) & 0xFU);
    if (tempValue < 8U)
    {
        ahbp = 1U;
    }
    else
    {
        ahbp = AHB_PreScaler[tempValue - 8U];
    }

    /* APB2 prescaler */
    tempValue = ((RCC->CFGR >> 13U) & 0x7U);
    if (tempValue < 4U)
    {
        apb2p = 1U;
    }
    else
    {
        apb2p = APB1_PreScaler[tempValue - 4U];
    }

    /* Final PCLK2 frequency */
    pclk2 = (SystemClk / ahbp) / apb2p;

    return pclk2;
}


