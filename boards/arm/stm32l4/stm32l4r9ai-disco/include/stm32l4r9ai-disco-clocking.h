/****************************************************************************
 * boards/arm/stm32l4/stm32l4r9ai-disco/include/stm32l4r9ai-disco-clocking.h
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

#ifndef __BOARDS_ARM_STM32L4_STM32L4R9AI_DISCO_INCLUDE_STM32L4R9AI_DISCO_CLOCKING_H
#define __BOARDS_ARM_STM32L4_STM32L4R9AI_DISCO_INCLUDE_STM32L4R9AI_DISCO_CLOCKING_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The stm32l4r9ai-disco supports both HSE and LSE crystals.  As shipped,
 * the HSE is a 16 MHz crystal X2. Therefore the stm32l4r9ai-disco can run
 * off the 16MHz HSI clock, or the MSI, or the HSE. Here we configure HSE
 * to give us 120MHz system clock (maximum supported for STM32L4+ chips)
 * instead of the more traditional 80MHz that is used by most STM32L4 boards
 * supported by NuttX.
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * MSI - variable up to 48 MHz, synchronized to LSE
 * HSE - 16 MHz installed
 * LSE - 32.768 kHz installed
 */

#define STM32L4_HSI_FREQUENCY     16000000ul
#define STM32L4_LSI_FREQUENCY     32000
#define STM32L4_LSE_FREQUENCY     32768
#define STM32L4_HSE_FREQUENCY     16000000ul

#define STM32L4_SYSCLK_FREQUENCY  120000000ul
#define BOARD_AHB_FREQUENCY       STM32L4_SYSCLK_FREQUENCY

/* Higher SYSCLK reguires more flash wait states. */

#define BOARD_FLASH_WAITSTATES    5

/* XXX there needs to be independent selections for the System Clock Mux and
 * the PLL Source Mux; currently System Clock Mux always is PLL, and PLL
 * Source Mux is chosen by the following define.  This is probably OK in many
 * cases, but should be separated to support other power configurations.
 */

#if 0
#  define HSI_CLOCK_CONFIG             1 /* HSI-16 clock configuration */
#elif 1
#  define HSE_CLOCK_CONFIG             1 /* HSE with 16 MHz xtal */
#else
#  define MSI_CLOCK_CONFIG             1 /* MSI @ 4 MHz autotrimmed via LSE */
#endif

#if defined(HSI_CLOCK_CONFIG)

#define STM32L4_BOARD_USEHSI           1

/* Prescaler common to all PLL inputs; will be 1 */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as 16 MHz / 1 * 15 / 2 = 120 MHz
 *
 * XXX NOTE:
 * currently the main PLL is implicitly turned on and is implicitly
 * the system clock; this should be configurable since not all
 * applications may want things done this way.
 */

#define STM32L4_PLLCFG_PLLN             RCC_PLLCFG_PLLN(15)
#define STM32L4_PLLCFG_PLLP             0
#undef  STM32L4_PLLCFG_PLLP_ENABLED
#define STM32L4_PLLCFG_PLLQ             RCC_PLLCFG_PLLQ_2
#define STM32L4_PLLCFG_PLLQ_ENABLED
#define STM32L4_PLLCFG_PLLR             RCC_PLLCFG_PLLR_2
#define STM32L4_PLLCFG_PLLR_ENABLED

/* 'SAIPLL1' is used to generate the 48 MHz clock, since we can't
 * do that with the main PLL's N value.  We set N = 13, and enable
 * the Q output (ultimately for CLK48) with /4.  So,
 * 16 MHz / 1 * 12 / 4 = 48 MHz
 *
 * XXX NOTE:  currently the SAIPLL /must/ be explicitly selected in the
 * menuconfig, or else all this is a moot point, and the various 48 MHz
 * peripherals will not work (RNG at present).  I would suggest removing
 * that option from Kconfig altogether, and simply making it an option
 * that is selected via a #define here, like all these other params.
 */

#define STM32L4_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(12)
#define STM32L4_PLLSAI1CFG_PLLP         0
#undef  STM32L4_PLLSAI1CFG_PLLP_ENABLED
#define STM32L4_PLLSAI1CFG_PLLQ         RCC_PLLSAI1CFG_PLLQ_4
#define  STM32L4_PLLSAI1CFG_PLLQ_ENABLED
#define STM32L4_PLLSAI1CFG_PLLR         0
#undef  STM32L4_PLLSAI1CFG_PLLR_ENABLED

/* 'SAIPLL2' is not used in this application */

#define STM32L4_PLLSAI2CFG_PLLN         RCC_PLLSAI2CFG_PLLN(8)
#define STM32L4_PLLSAI2CFG_PLLP         0
#undef  STM32L4_PLLSAI2CFG_PLLP_ENABLED
#define STM32L4_PLLSAI2CFG_PLLR         0
#undef  STM32L4_PLLSAI2CFG_PLLR_ENABLED

/* CLK48 will come from PLLSAI1 (implicitly Q) */

#if defined(CONFIG_STM32L4_OTGFS) || defined(STM32L4_SDMMC) || defined(CONFIG_STM32L4_RNG)
#  define STM32L4_USE_CLK48       1
#  define STM32L4_CLK48_SEL       RCC_CCIPR_CLK48SEL_PLLSAI1
#  define STM32L4_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

/* Enable the LSE oscillator, used automatically trim the MSI, and for RTC */

#define STM32L4_USE_LSE           1

/* HSI16 used as I2C clock */

#define STM32L4_I2C_USE_HSI16     1

/* AHB clock (HCLK) is SYSCLK (120 MHz) */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/1 (120 MHz) */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

/* The timer clock frequencies are automatically defined by hardware.
 * If the APB prescaler equals 1, the timer clock frequencies are set to the
 * same frequency as that of the APB domain. Otherwise they are set to twice.
 *
 * REVISIT : this can be configured
 */

#define STM32L4_APB1_TIM2_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM3_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM4_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM5_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM6_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM7_CLKIN   (STM32L4_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (120 MHz) */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

/* The timer clock frequencies are automatically defined by hardware.
 * If the APB prescaler equals 1, the timer clock frequencies are set to the
 * same frequency as that of the APB domain. Otherwise they are set to twice.
 *
 * REVISIT : this can be configured
 */

#define STM32L4_APB2_TIM1_CLKIN   (STM32L4_PCLK2_FREQUENCY)
#define STM32L4_APB2_TIM8_CLKIN   (STM32L4_PCLK2_FREQUENCY)

#elif defined(HSE_CLOCK_CONFIG)

/* Use the HSE */

#define STM32L4_BOARD_USEHSE      1

/* Prescaler common to all PLL inputs */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as 16 MHz / 1 * 15 / 2 = 120 MHz
 *
 * XXX NOTE:
 * currently the main PLL is implicitly turned on and is implicitly
 * the system clock; this should be configurable since not all
 * applications may want things done this way.
 */

#define STM32L4_PLLCFG_PLLN             RCC_PLLCFG_PLLN(15)
#define STM32L4_PLLCFG_PLLP             0
#undef  STM32L4_PLLCFG_PLLP_ENABLED
#define STM32L4_PLLCFG_PLLQ             RCC_PLLCFG_PLLQ_2
#define STM32L4_PLLCFG_PLLQ_ENABLED
#define STM32L4_PLLCFG_PLLR             RCC_PLLCFG_PLLR_2
#define STM32L4_PLLCFG_PLLR_ENABLED

/* 'SAIPLL1' is used to generate the 48 MHz clock, since we can't
 * do that with the main PLL's N value.  We set N = 12, and enable
 * the Q output (ultimately for CLK48) with /4.  So,
 * 16 MHz / 1 * 12 / 4 = 48 MHz
 *
 * XXX NOTE:  currently the SAIPLL /must/ be explicitly selected in the
 * menuconfig, or else all this is a moot point, and the various 48 MHz
 * peripherals will not work (RNG at present).  I would suggest removing
 * that option from Kconfig altogether, and simply making it an option
 * that is selected via a #define here, like all these other params.
 */

#define STM32L4_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(12)
#define STM32L4_PLLSAI1CFG_PLLP         0
#undef  STM32L4_PLLSAI1CFG_PLLP_ENABLED
#define STM32L4_PLLSAI1CFG_PLLQ         RCC_PLLSAI1CFG_PLLQ_4
#define STM32L4_PLLSAI1CFG_PLLQ_ENABLED
#define STM32L4_PLLSAI1CFG_PLLR         0
#undef  STM32L4_PLLSAI1CFG_PLLR_ENABLED

/* 'SAIPLL2' is not used in this application */

#define STM32L4_PLLSAI2CFG_PLLN         RCC_PLLSAI2CFG_PLLN(8)
#define STM32L4_PLLSAI2CFG_PLLP         0
#undef  STM32L4_PLLSAI2CFG_PLLP_ENABLED
#define STM32L4_PLLSAI2CFG_PLLR         0
#undef  STM32L4_PLLSAI2CFG_PLLR_ENABLED

/* Enable CLK48; get it from PLLSAI1 */

#if defined(CONFIG_STM32L4_OTGFS) || defined(STM32L4_SDMMC) || defined(CONFIG_STM32L4_RNG)
#  define STM32L4_USE_CLK48       1
#  define STM32L4_CLK48_SEL       RCC_CCIPR_CLK48SEL_PLLSAI1
#  define STM32L4_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

/* Enable LSE (for the RTC) */

#define STM32L4_USE_LSE           1

/* HSI16 used as I2C clock */

#define STM32L4_I2C_USE_HSI16     1

/* Configure the HCLK divisor (for the AHB bus, core, memory, and DMA */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* Configure the APB1 prescaler */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

#define STM32L4_APB1_TIM2_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM3_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM4_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM5_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM6_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM7_CLKIN   (STM32L4_PCLK1_FREQUENCY)

/* Configure the APB2 prescaler */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

#define STM32L4_APB2_TIM1_CLKIN   (STM32L4_PCLK2_FREQUENCY)
#define STM32L4_APB2_TIM8_CLKIN   (STM32L4_PCLK2_FREQUENCY)

#elif defined(MSI_CLOCK_CONFIG)

/* Use the MSI; frequ = 4 MHz; autotrim from LSE */

#define STM32L4_BOARD_USEMSI      1
#define STM32L4_BOARD_MSIRANGE    RCC_CR_MSIRANGE_4M

/* Prescaler common to all PLL inputs */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as 4 MHz / 1 * 60 / 2 = 120 MHz
 *
 * XXX NOTE:
 * currently the main PLL is implicitly turned on and is implicitly
 * the system clock; this should be configurable since not all
 * applications may want things done this way.
 */

#define STM32L4_PLLCFG_PLLN             RCC_PLLCFG_PLLN(60)
#define STM32L4_PLLCFG_PLLP             0
#undef  STM32L4_PLLCFG_PLLP_ENABLED
#define STM32L4_PLLCFG_PLLQ             RCC_PLLCFG_PLLQ_2
#define STM32L4_PLLCFG_PLLQ_ENABLED
#define STM32L4_PLLCFG_PLLR             RCC_PLLCFG_PLLR_2
#define STM32L4_PLLCFG_PLLR_ENABLED

/* 'SAIPLL1' is used to generate the 48 MHz clock, since we can't
 * do that with the main PLL's N value.  We set N = 12, and enable
 * the Q output (ultimately for CLK48) with /4.  So,
 * 4 MHz / 1 * 24 / 2 = 48 MHz
 *
 * XXX NOTE:  currently the SAIPLL /must/ be explicitly selected in the
 * menuconfig, or else all this is a moot point, and the various 48 MHz
 * peripherals will not work (RNG at present).  I would suggest removing
 * that option from Kconfig altogether, and simply making it an option
 * that is selected via a #define here, like all these other params.
 */

#define STM32L4_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(24)
#define STM32L4_PLLSAI1CFG_PLLP         0
#undef  STM32L4_PLLSAI1CFG_PLLP_ENABLED
#define STM32L4_PLLSAI1CFG_PLLQ         RCC_PLLSAI1CFG_PLLQ_2
#define STM32L4_PLLSAI1CFG_PLLQ_ENABLED
#define STM32L4_PLLSAI1CFG_PLLR         0
#undef  STM32L4_PLLSAI1CFG_PLLR_ENABLED

/* 'SAIPLL2' is not used in this application */

#define STM32L4_PLLSAI2CFG_PLLN         RCC_PLLSAI2CFG_PLLN(8)
#define STM32L4_PLLSAI2CFG_PLLP         0
#undef  STM32L4_PLLSAI2CFG_PLLP_ENABLED
#define STM32L4_PLLSAI2CFG_PLLR         0
#undef  STM32L4_PLLSAI2CFG_PLLR_ENABLED

/* Enable CLK48; get it from PLLSAI1 */

#if defined(CONFIG_STM32L4_OTGFS) || defined(STM32L4_SDMMC) || defined(CONFIG_STM32L4_RNG)
#  define STM32L4_USE_CLK48       1
#  define STM32L4_CLK48_SEL       RCC_CCIPR_CLK48SEL_PLLSAI1
#  define STM32L4_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

/* Enable LSE (for the RTC) */

#define STM32L4_USE_LSE           1

/* HSI16 used as I2C clock */

#define STM32L4_I2C_USE_HSI16     1

/* Configure the HCLK divisor (for the AHB bus, core, memory, and DMA */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* Configure the APB1 prescaler */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

#define STM32L4_APB1_TIM2_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM3_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM4_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM5_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM6_CLKIN   (STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM7_CLKIN   (STM32L4_PCLK1_FREQUENCY)

/* Configure the APB2 prescaler */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

#define STM32L4_APB2_TIM1_CLKIN   (STM32L4_PCLK2_FREQUENCY)
#define STM32L4_APB2_TIM8_CLKIN   (STM32L4_PCLK2_FREQUENCY)

#endif /* clock selection */

/* The timer clock frequencies are automatically defined by hardware.
 * If the APB prescaler equals 1, the timer clock frequencies are set to the
 * same frequency as that of the APB domain. Otherwise they are set to twice.
 * Note: TIM1,8,15,16,17 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM8_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM15_FREQUENCY   STM32L4_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY   STM32L4_HCLK_FREQUENCY
#define BOARD_TIM17_FREQUENCY   STM32L4_HCLK_FREQUENCY
#define BOARD_LPTIM1_FREQUENCY  STM32L4_HCLK_FREQUENCY
#define BOARD_LPTIM2_FREQUENCY  STM32L4_HCLK_FREQUENCY

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
#endif /* __BOARDS_ARM_STM32L4_STM32L4R9AI_DISCO_INCLUDE_STM32L4R9AI_DISCO_CLOCKING_H */
