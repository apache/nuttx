/****************************************************************************
 * boards/arm/stm32h7/openh743i/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32H7_OPENH743I_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H7_OPENH743I_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Openh743i board provides the following clock sources:
 *
 *   X2:  32.768 KHz crystal for LSE
 *   X2:  8 MHz HSE crystal oscillator
 *
 * So we have these clock source available within the STM32
 *
 *   HSI: 16 MHz RC factory-trimmed
 *   LSI: 32 KHz RC
 *   HSE: 8 MHz oscillator X2
 *   LSE: 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* Main PLL Configuration.
 *
 * PLL source is HSE = 8,000,000
 *
 * When STM32_HSE_FREQUENCY / PLLM <= 2MHz VCOL must be selected.
 * VCOH otherwise.
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 63
 *     4 <= PLLN <= 512
 *   150 MHz <= PLL_VCOL <= 420MHz
 *   192 MHz <= PLL_VCOH <= 836MHz
 *
 * SYSCLK  = PLL_VCO / PLLP
 * CPUCLK  = SYSCLK / D1CPRE
 * Subject to
 *
 *   PLLP1   = {2, 4, 6, 8, ..., 128}
 *   PLLP2,3 = {2, 3, 4, ..., 128}
 *   CPUCLK <= 400 MHz
 */

#define STM32_BOARD_USEHSE
#undef STM32_HSEBYP_ENABLE

#define STM32_PLLCFG_PLLSRC      RCC_PLLCKSELR_PLLSRC_HSE

/* PLL1, wide 4 - 8 MHz input, enable DIVP, DIVQ, DIVR
 *
 *   PLL1_VCO = (8,000,000 / 2) * 200 = 800 MHz
 *
 *   PLL1P = PLL1_VCO/2  = 800 MHz / 2   = 400 MHz
 *   PLL1Q = PLL1_VCO/4  = 800 MHz / 4   = 200 MHz
 *   PLL1R = PLL1_VCO/8  = 800 MHz / 8   = 100 MHz
 */

#define STM32_PLLCFG_PLL1CFG     (RCC_PLLCFGR_PLL1VCOSEL_WIDE | \
                                  RCC_PLLCFGR_PLL1RGE_4_8_MHZ | \
                                  RCC_PLLCFGR_DIVP1EN | \
                                  RCC_PLLCFGR_DIVQ1EN | \
                                  RCC_PLLCFGR_DIVR1EN)
#define STM32_PLLCFG_PLL1M       RCC_PLLCKSELR_DIVM1(2)
#define STM32_PLLCFG_PLL1N       RCC_PLL1DIVR_N1(200)
#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR_P1(2)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR_Q1(4)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR_R1(8)

#define STM32_VCO1_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 200)
#define STM32_PLL1P_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 8)

/* PLL2 */

#define STM32_PLLCFG_PLL2CFG (RCC_PLLCFGR_PLL2VCOSEL_WIDE | \
                              RCC_PLLCFGR_PLL2RGE_4_8_MHZ | \
                              RCC_PLLCFGR_DIVP2EN)
#define STM32_PLLCFG_PLL2M       RCC_PLLCKSELR_DIVM2(2)
#define STM32_PLLCFG_PLL2N       RCC_PLL2DIVR_N2(200)
#define STM32_PLLCFG_PLL2P       RCC_PLL2DIVR_P2(40)
#define STM32_PLLCFG_PLL2Q       0
#define STM32_PLLCFG_PLL2R       0

#define STM32_VCO2_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 200)
#define STM32_PLL2P_FREQUENCY    (STM32_VCO2_FREQUENCY / 2)
#define STM32_PLL2Q_FREQUENCY
#define STM32_PLL2R_FREQUENCY

/* PLL3, wide 1 - 2 MHz input, enable DIVQ
 *
 *   PLL3_VCO = (8,000,000 / 8) * 336 = 336 MHz
 *
 *   PLL3P - TODO
 *   PLL3Q = PLL3_VCO / 7  = 336 MHz / 7   = 48 MHz
 *   PLL3R - TODO
 */

#define STM32_PLLCFG_PLL3CFG  (RCC_PLLCFGR_PLL3VCOSEL_WIDE | \
                               RCC_PLLCFGR_PLL3RGE_1_2_MHZ | \
                               RCC_PLLCFGR_DIVQ3EN)
#define STM32_PLLCFG_PLL3M    RCC_PLLCKSELR_DIVM3(8)
#define STM32_PLLCFG_PLL3N    RCC_PLL3DIVR_N3(336)
#define STM32_PLLCFG_PLL3P    RCC_PLL3DIVR_P3(2)
#define STM32_PLLCFG_PLL3Q    RCC_PLL3DIVR_Q3(7)
#define STM32_PLLCFG_PLL3R    RCC_PLL3DIVR_R3(2)

#define STM32_VCO3_FREQUENCY  ((STM32_HSE_FREQUENCY / 2) * 100)
#define STM32_PLL3P_FREQUENCY
#define STM32_PLL3Q_FREQUENCY (STM32_VCO3_FREQUENCY / 8)
#define STM32_PLL3R_FREQUENCY

/* SYSCLK = PLL1P = 400 MHz
 * CPUCLK = SYSCLK / 1 = 400 MHz
 */

#define STM32_RCC_D1CFGR_D1CPRE  (RCC_D1CFGR_D1CPRE_SYSCLK)
#define STM32_SYSCLK_FREQUENCY   (STM32_PLL1P_FREQUENCY)
#define STM32_CPUCLK_FREQUENCY   (STM32_SYSCLK_FREQUENCY / 1)

/* Configure Clock Assignments */

/* AHB clock (HCLK) is SYSCLK/2 (200 MHz max)
 * HCLK1 = HCLK2 = HCLK3 = HCLK4
 */

#define STM32_RCC_D1CFGR_HPRE   RCC_D1CFGR_HPRE_SYSCLKd2        /* HCLK  = SYSCLK / 2 */
#define STM32_ACLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* ACLK in D1, HCLK3 in D1 */
#define STM32_HCLK_FREQUENCY    (STM32_CPUCLK_FREQUENCY / 2)    /* HCLK in D2, HCLK4 in D3 */

/* APB1 clock (PCLK1) is HCLK/4 (54 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE1  RCC_D2CFGR_D2PPRE1_HCLKd4       /* PCLK1 = HCLK / 4 */
#define STM32_PCLK1_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB2 clock (PCLK2) is HCLK/4 (54 MHz) */

#define STM32_RCC_D2CFGR_D2PPRE2  RCC_D2CFGR_D2PPRE2_HCLKd4       /* PCLK2 = HCLK / 4 */
#define STM32_PCLK2_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB3 clock (PCLK3) is HCLK/4 (54 MHz) */

#define STM32_RCC_D1CFGR_D1PPRE   RCC_D1CFGR_D1PPRE_HCLKd4        /* PCLK3 = HCLK / 4 */
#define STM32_PCLK3_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* APB4 clock (PCLK4) is HCLK/4 (54 MHz) */

#define STM32_RCC_D3CFGR_D3PPRE   RCC_D3CFGR_D3PPRE_HCLKd4       /* PCLK4 = HCLK / 4 */
#define STM32_PCLK4_FREQUENCY     (STM32_HCLK_FREQUENCY/4)

/* Timer clock frequencies */

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be twice PCLK2 */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (2*STM32_PCLK2_FREQUENCY)

/* Kernel Clock Configuration
 *
 * Note: look at Table 54 in ST Manual
 */

/* I2C123 clock source - HSI */

#define STM32_RCC_D2CCIP2R_I2C123SRC RCC_D2CCIP2R_I2C123SEL_HSI

/* I2C4 clock source - HSI */

#define STM32_RCC_D3CCIPR_I2C4SRC    RCC_D3CCIPR_I2C4SEL_HSI

/* SPI123 clock source - PLL1Q */

#define STM32_RCC_D2CCIP1R_SPI123SRC RCC_D2CCIP1R_SPI123SEL_PLL1

/* SPI45 clock source - APB (PCLK2?) */

#define STM32_RCC_D2CCIP1R_SPI45SRC  RCC_D2CCIP1R_SPI45SEL_APB

/* SPI6 clock source - APB (PCLK4) */

#define STM32_RCC_D3CCIPR_SPI6SRC    RCC_D3CCIPR_SPI6SEL_PCLK4

#if 1
/* USB 1 and 2 clock source - HSI48 */

#  define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_HSI48
#else
/* USB 1 and 2 clock source - PLL3Q */

#  define STM32_RCC_D2CCIP2R_USBSRC    RCC_D2CCIP2R_USBSEL_PLL3
#endif

/* ADC 1 2 3 clock source - pll2_pclk */

#define STM32_RCC_D3CCIPR_ADCSEL     RCC_D3CCIPR_ADCSEL_PLL2

/* FLASH wait states
 *
 *  ------------ ---------- -----------
 *  Vcore        MAX ACLK   WAIT STATES
 *  ------------ ---------- -----------
 *  1.15-1.26 V     70 MHz    0
 *  (VOS1 level)   140 MHz    1
 *                 210 MHz    2
 *  1.05-1.15 V     55 MHz    0
 *  (VOS2 level)   110 MHz    1
 *                 165 MHz    2
 *                 220 MHz    3
 *  0.95-1.05 V     45 MHz    0
 *  (VOS3 level)    90 MHz    1
 *                 135 MHz    2
 *                 180 MHz    3
 *                 225 MHz    4
 *  ------------ ---------- -----------
 */

#define BOARD_FLASH_WAITSTATES 2
#define BOARD_FLASH_PROGDELAY  3
#define STM32_PWR_VOS_SCALE    PWR_D3CR_VOS_SCALE_1
#define STM32_VOS_OVERDRIVE    1 /* Enable VOS0 */

/* SDMMC definitions ********************************************************/

/* Init 400kHz, PLL1Q/(2*250) */

#define STM32_SDMMC_INIT_CLKDIV     (250 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

/* Set these to 20 MHz (PLL1Q/(2*5)).
 * Higher frequency doesn't work, probably due to poor board
 * signal integrity
 */

#define STM32_SDMMC_MMCXFR_CLKDIV   (5 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)
#define STM32_SDMMC_SDXFR_CLKDIV    (5 << STM32_SDMMC_CLKCR_CLKDIV_SHIFT)

#define STM32_SDMMC_CLKCR_EDGE      STM32_SDMMC_CLKCR_NEGEDGE

/* LED definitions **********************************************************/

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_LED3        2
#define BOARD_LED4        3
#define BOARD_NLEDS       4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)
#define BOARD_LED3_BIT    (1 << BOARD_LED3)
#define BOARD_LED4_BIT    (1 << BOARD_LED4)

/* Alternate function pin selections ****************************************/

/* USART1 (CP2102 converter) */

#define GPIO_USART1_RX     GPIO_USART1_RX_2    /* PA10 */
#define GPIO_USART1_TX     GPIO_USART1_TX_2    /* PA9 */

/* USART3 */

#define GPIO_USART3_RX     GPIO_USART3_RX_3    /* PD9 */
#define GPIO_USART3_TX     GPIO_USART3_TX_3    /* PD8 */

/* USB OTG ULPI */

#define GPIO_OTG_HS_ULPI_CK  (GPIO_OTG_HS_ULPI_CK_0|GPIO_SPEED_100MHz)
#define GPIO_OTG_HS_ULPI_D0  (GPIO_OTG_HS_ULPI_D0_0|GPIO_SPEED_100MHz)
#define GPIO_OTG_HS_ULPI_D1  (GPIO_OTG_HS_ULPI_D1_0|GPIO_SPEED_100MHz)
#define GPIO_OTG_HS_ULPI_D2  (GPIO_OTG_HS_ULPI_D2_0|GPIO_SPEED_100MHz)
#define GPIO_OTG_HS_ULPI_D3  (GPIO_OTG_HS_ULPI_D3_0|GPIO_SPEED_100MHz)
#define GPIO_OTG_HS_ULPI_D4  (GPIO_OTG_HS_ULPI_D4_0|GPIO_SPEED_100MHz)
#define GPIO_OTG_HS_ULPI_D5  (GPIO_OTG_HS_ULPI_D5_0|GPIO_SPEED_100MHz)
#define GPIO_OTG_HS_ULPI_D6  (GPIO_OTG_HS_ULPI_D6_0|GPIO_SPEED_100MHz)
#define GPIO_OTG_HS_ULPI_D7  (GPIO_OTG_HS_ULPI_D7_0|GPIO_SPEED_100MHz)
#define GPIO_OTG_HS_ULPI_DIR (GPIO_OTG_HS_ULPI_DIR_1|GPIO_SPEED_100MHz) /* PC2 */
#define GPIO_OTG_HS_ULPI_NXT (GPIO_OTG_HS_ULPI_NXT_1|GPIO_SPEED_100MHz) /* PC3 */
#define GPIO_OTG_HS_ULPI_STP (GPIO_OTG_HS_ULPI_STP_0|GPIO_SPEED_100MHz)

/* OTGFS */

#define GPIO_OTGFS_DM (GPIO_OTGFS_DM_0|GPIO_SPEED_100MHz)
#define GPIO_OTGFS_DP (GPIO_OTGFS_DP_0|GPIO_SPEED_100MHz)
#define GPIO_OTGFS_ID (GPIO_OTGFS_ID_0|GPIO_SPEED_100MHz)

/* SDMMC */

#define GPIO_SDMMC1_D0  (GPIO_SDMMC1_D0_0|GPIO_SPEED_50MHz)
#define GPIO_SDMMC1_D1  (GPIO_SDMMC1_D1_0|GPIO_SPEED_50MHz)
#define GPIO_SDMMC1_D2  (GPIO_SDMMC1_D2_0|GPIO_SPEED_50MHz)
#define GPIO_SDMMC1_D3  (GPIO_SDMMC1_D3_0|GPIO_SPEED_50MHz)
#define GPIO_SDMMC1_CK  (GPIO_SDMMC1_CK_0|GPIO_SPEED_50MHz)
#define GPIO_SDMMC1_CMD (GPIO_SDMMC1_CMD_0|GPIO_SPEED_50MHz)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32H7_OPENH743I_INCLUDE_BOARD_H */
