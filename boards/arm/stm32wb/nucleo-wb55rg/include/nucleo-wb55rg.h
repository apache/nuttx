/****************************************************************************
 * boards/arm/stm32wb/nucleo-wb55rg/include/nucleo-wb55rg.h
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

#ifndef __BOARDS_ARM_STM32WB_NUCLEO_WB55RG_INCLUDE_NUCLEO_WB55RG_H
#define __BOARDS_ARM_STM32WB_NUCLEO_WB55RG_INCLUDE_NUCLEO_WB55RG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The Nucleo WB55RG supports both HSE and LSE crystals (X1 and X2). As
 * shipped, the HSE is a 32MHz crystal X1. Therefore, the Nucleo WB55RG can
 * run off the HSI clock, or the MSI, or the HSE.  Here we configure HSE to
 * give us 64MHz system clock (maximum supported for STM32WB chips)
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * MSI - variable up to 48 MHz, synchronized to LSE
 * HSE - 32 MHz installed
 * LSE - 32.768 kHz installed
 * HSI48 - 48 MHz fine-granularity trimmable RC with CRS
 */

#define STM32WB_HSI_FREQUENCY           16000000ul
#define STM32WB_LSI_FREQUENCY           32000
#define STM32WB_LSE_FREQUENCY           32768
#define STM32WB_HSE_FREQUENCY           32000000ul

/* XXX there needs to be independent selections for the System Clock Mux and
 * the PLL Source Mux; currently System Clock Mux always is PLL, and PLL
 * Source Mux is chosen by the following define.  This is probably OK in many
 * cases, but should be separated to support other power configurations.
 */

#if 0
#  define HSI_CLOCK_CONFIG              1 /* HSI clock configuration */
#elif 1
#  define HSE_CLOCK_CONFIG              1 /* HSE with 32MHz xtal */
#else
#  define MSI_CLOCK_CONFIG              1 /* MSI @ 4MHz autotrimmed via LSE */
#endif

#if 0
#  define STM32WB_BOARD_RFWKP_USEHSE    1 /* CPU2 use HSE/1024 on RF wakeup */
#elif 1
#  define STM32WB_BOARD_RFWKP_USELSE    1 /* CPU2 use LSE on RF wakeup */
#endif

#if defined(HSI_CLOCK_CONFIG)

#define STM32WB_BOARD_USEHSI            1

#define STM32WB_SYSCLK_FREQUENCY        64000000ul

/* Prescaler common to all PLL inputs; will be 1 */

#define STM32WB_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as (((16MHz / 1) * 8) / 2) = 64MHz
 */

#define STM32WB_PLLCFG_PLLN             RCC_PLLCFG_PLLN(8)
#define STM32WB_PLLCFG_PLLR_ENABLED
#define STM32WB_PLLCFG_PLLR             RCC_PLLCFG_PLLR(2)

/* 'SAIPLL1' is not used */

#define STM32WB_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(8)

/* CLK48 will come from HSI48 */

#define STM32WB_USE_CLK48               1
#define STM32WB_CLK48_SEL               RCC_CCIPR_CLK48SEL_HSI48
#define STM32WB_HSI48_SYNCSRC           SYNCSRC_LSE

/* Enable LSE oscillator, used automatically trim the HSI48, and for RTC */

#define STM32WB_USE_LSE                 1

#elif defined(HSE_CLOCK_CONFIG)

/* Use the HSE */

#define STM32WB_BOARD_USEHSE            1

#define STM32WB_SYSCLK_FREQUENCY        64000000ul

/* Prescaler common to all PLL inputs; will be 2 */

#define STM32WB_PLLCFG_PLLM             RCC_PLLCFG_PLLM(2)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as (((32MHz / 2) * 12) / 3) = 64MHz
 *  And the Q output is set as (((32MHz / 2) * 12) / 4) = 48MHz
 */

#define STM32WB_PLLCFG_PLLN             RCC_PLLCFG_PLLN(12)
#define STM32WB_PLLCFG_PLLR_ENABLED
#define STM32WB_PLLCFG_PLLR             RCC_PLLCFG_PLLR(3)
#define STM32WB_PLLCFG_PLLQ_ENABLED
#define STM32WB_PLLCFG_PLLQ             RCC_PLLCFG_PLLQ(4)

/* 'SAIPLL1' is not used */

#define STM32WB_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(8)

/* CLK48 will come from the PLLMAIN via the Q output */

#define STM32WB_USE_CLK48               1
#define STM32WB_CLK48_SEL               RCC_CCIPR_CLK48SEL_PLLMAIN
#define STM32WB_HSI48_SYNCSRC           SYNCSRC_NONE

/* Enable LSE (for the RTC) */

#define STM32WB_USE_LSE                 1

#elif defined(MSI_CLOCK_CONFIG)

/* Use the MSI */

#define STM32WB_BOARD_USEMSI            1

#define STM32WB_BOARD_MSIRANGE          RCC_CR_MSIRANGE_4M

#define STM32WB_SYSCLK_FREQUENCY        64000000ul

/* Prescaler common to all PLL inputs; will be 1 */

#define STM32WB_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as (((4MHz / 1) * 48) / 3) = 64MHz
 *  And the Q output is set as (((4MHz / 1) * 48) / 4) = 48MHz
 */

#define STM32WB_PLLCFG_PLLN             RCC_PLLCFG_PLLN(48)
#define STM32WB_PLLCFG_PLLR_ENABLED
#define STM32WB_PLLCFG_PLLR             RCC_PLLCFG_PLLR(3)
#define STM32WB_PLLCFG_PLLQ_ENABLED
#define STM32WB_PLLCFG_PLLQ             RCC_PLLCFG_PLLQ(4)

/* 'SAIPLL1' is not used */

#define STM32WB_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(8)

/* CLK48 will come from the PLLMAIN via the Q output */

#define STM32WB_USE_CLK48               1
#define STM32WB_CLK48_SEL               RCC_CCIPR_CLK48SEL_PLLMAIN
#define STM32WB_HSI48_SYNCSRC           SYNCSRC_NONE

/* Enable the LSE oscillator, used automatically trim the MSI, and for RTC */

#define STM32WB_USE_LSE                 1

#endif

/* AHB clock (HCLK) is SYSCLK (64MHz) */

#define BOARD_AHB_FREQUENCY         STM32WB_SYSCLK_FREQUENCY

#define STM32WB_RCC_CFGR_HPRE       RCC_CFGR_HPRE_SYSCLK
#define STM32WB_HCLK_FREQUENCY      STM32WB_SYSCLK_FREQUENCY

/* CPU2 clock (HCLK2) is SYSCLK/2 (32MHz) */

#define STM32WB_RCC_EXTCFGR_C2HPRE  RCC_EXTCFGR_C2HPRE_2

/* AHB4 clock (HCLK4) is SYSCLK (64MHz) */

#define STM32WB_RCC_EXTCFGR_SHDHPRE RCC_EXTCFGR_SHDHPRE_1

/* APB1 clock (PCLK1) is HCLK/1 (64MHz) */

#define STM32WB_RCC_CFGR_PPRE1      RCC_CFGR_PPRE1_HCLK1
#define STM32WB_PCLK1_FREQUENCY     (STM32WB_HCLK_FREQUENCY / 1)

/* APB2 clock (PCLK2) is HCLK/1 (64MHz) */

#define STM32WB_RCC_CFGR_PPRE2      RCC_CFGR_PPRE2_HCLK1
#define STM32WB_PCLK2_FREQUENCY     (STM32WB_HCLK_FREQUENCY / 1)

/* Timer Frequencies, if APB prescaler is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,16,17 are on APB2, TIM2 is on APB1
 */

/* Timers driven from APB1 will be the same frequency as PCLK1 */

#define STM32WB_APB1_TIM2_CLKIN     (1 * STM32WB_PCLK1_FREQUENCY)

/* Timers driven from APB2 will be the same frequency as PCLK2 */

#define STM32WB_APB2_TIM1_CLKIN     (1 * STM32WB_PCLK2_FREQUENCY)
#define STM32WB_APB2_TIM16_CLKIN    (1 * STM32WB_PCLK2_FREQUENCY)
#define STM32WB_APB2_TIM17_CLKIN    (1 * STM32WB_PCLK2_FREQUENCY)

#define BOARD_TIM1_FREQUENCY        STM32WB_APB2_TIM1_CLKIN
#define BOARD_TIM2_FREQUENCY        STM32WB_APB1_TIM2_CLKIN
#define BOARD_TIM16_FREQUENCY       STM32WB_APB2_TIM16_CLKIN
#define BOARD_TIM17_FREQUENCY       STM32WB_APB2_TIM17_CLKIN

/* Higher SYSCLK reguires more flash wait states. */

#define BOARD_FLASH_WAITSTATES      3

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
#endif /* __BOARDS_ARM_STM32WB_NUCLEO_WB55RG_INCLUDE_NUCLEO_WB55RG_H */
