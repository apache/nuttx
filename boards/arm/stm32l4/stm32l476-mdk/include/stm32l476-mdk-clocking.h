/****************************************************************************
 * boards/arm/stm32l4/stm32l476-mdk/include/stm32l476-mdk-clocking.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2016 Motorola Mobility, LLC. All rights reserved.
 *   Author: dev@ziggurat29.com
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32L4_STM32L476_MDK_INCLUDE_STM32L476_MDK_CLOCKING_H
#define __BOARDS_ARM_STM32L4_STM32L476_MDK_INCLUDE_STM32L476_MDK_CLOCKING_H

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

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - not installed
 * LSE - not installed
 */

#define STM32L4_HSI_FREQUENCY     16000000ul
#define STM32L4_LSI_FREQUENCY     32000
#define STM32L4_LSE_FREQUENCY     32768

#define BOARD_AHB_FREQUENCY       80000000ul

/* XXX there needs to be independent selections for the System Clock Mux and
 * the PLL Source Mux; currently System Clock Mux always is PLL, and PLL
 * Source Mux is chosen by the following define.  This is probably OK in many
 * cases, but should be separated to support other power configurations.
 */

#if 0
#  define HSI_CLOCK_CONFIG          /* HSI-16 clock configuration */
#else
#  define MSI_CLOCK_CONFIG          /* MSI @ 4 MHz autotrimmed via LSE */
#endif

#if defined(HSI_CLOCK_CONFIG)

#define STM32L4_BOARD_USEHSI           1

/* Prescaler common to all PLL inputs; will be 1 (XXX source is implicitly
 * as per comment above HSI)
 */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as 16 MHz / 1 * 10 / 2 = 80 MHz
 *
 * XXX NOTE:
 * currently the main PLL is implicitly turned on and is implicitly the
 * system clock; this should be configurable since not all applications
 * may want things done this way.
 */

#define STM32L4_PLLCFG_PLLN             RCC_PLLCFG_PLLN(10)
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
#define STM32L4_PLLSAI1CFG_PLLQ_ENABLED
#define STM32L4_PLLSAI1CFG_PLLR         0
#undef  STM32L4_PLLSAI1CFG_PLLR_ENABLED

/* 'SAIPLL2' is not used in this application */

#define STM32L4_PLLSAI2CFG_PLLN         RCC_PLLSAI2CFG_PLLN(8)
#define STM32L4_PLLSAI2CFG_PLLP         0
#undef  STM32L4_PLLSAI2CFG_PLLP_ENABLED
#define STM32L4_PLLSAI2CFG_PLLR         0
#undef  STM32L4_PLLSAI2CFG_PLLR_ENABLED

#define STM32L4_SYSCLK_FREQUENCY  80000000ul

/* CLK48 will come from PLLSAI1 (implicitly Q) */

#define STM32L4_USE_CLK48         1
#define STM32L4_CLK48_SEL         RCC_CCIPR_CLK48SEL_PLLSAI1

/* Enable the LSE oscillator, used automatically trim the MSI, and for RTC */

#define STM32L4_USE_LSE           1

/* AHB clock (HCLK) is SYSCLK (80MHz) */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/1 (80MHz) */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

/* Timers driven from APB1 will be twice PCLK1 */

/* REVISIT : this can be configured */

#define STM32L4_APB1_TIM2_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM3_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM4_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM5_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM6_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM7_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (80MHz) */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

/* Timers driven from APB2 will be twice PCLK2 */

/* REVISIT : this can be configured */

#define STM32L4_APB2_TIM1_CLKIN   (2*STM32L4_PCLK2_FREQUENCY)
#define STM32L4_APB2_TIM8_CLKIN   (2*STM32L4_PCLK2_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */

#elif defined(MSI_CLOCK_CONFIG)

/* Use the MSI; frequ = 4 MHz; autotrim from LSE */

#define STM32L4_BOARD_USEMSI      1
#define STM32L4_BOARD_MSIRANGE    RCC_CR_MSIRANGE_4M

/* Prescaler common to all PLL inputs */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock */

#define STM32L4_PLLCFG_PLLN             RCC_PLLCFG_PLLN(40)
#define STM32L4_PLLCFG_PLLP             0
#undef  STM32L4_PLLCFG_PLLP_ENABLED
#define STM32L4_PLLCFG_PLLQ             0
#undef  STM32L4_PLLCFG_PLLQ_ENABLED
#define STM32L4_PLLCFG_PLLR             RCC_PLLCFG_PLLR_2
#define STM32L4_PLLCFG_PLLR_ENABLED

/* 'SAIPLL1' is used to generate the 48 MHz clock */

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

#define STM32L4_SYSCLK_FREQUENCY  80000000ul

/* Enable CLK48; get it from PLLSAI1 */

#define STM32L4_USE_CLK48
#define STM32L4_CLK48_SEL         RCC_CCIPR_CLK48SEL_PLLSAI1

/* Disable LSE (for the RTC) */

#undef STM32L4_USE_LSE

/* Configure the HCLK divisor (for the AHB bus, core, memory, and DMA */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* Configure the APB1 prescaler */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK       /* PCLK1 = HCLK / 1 */
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

#define STM32L4_APB1_TIM2_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM3_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM4_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM5_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM6_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)
#define STM32L4_APB1_TIM7_CLKIN   (2*STM32L4_PCLK1_FREQUENCY)

/* Configure the APB2 prescaler */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY/1)

#define STM32L4_APB2_TIM1_CLKIN   (2*STM32L4_PCLK2_FREQUENCY)
#define STM32L4_APB2_TIM8_CLKIN   (2*STM32L4_PCLK2_FREQUENCY)

#endif

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8,15,16,17 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM3_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM4_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM5_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM6_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM7_FREQUENCY    (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_TIM8_FREQUENCY    STM32L4_HCLK_FREQUENCY
#define BOARD_TIM15_FREQUENCY   STM32L4_HCLK_FREQUENCY
#define BOARD_TIM16_FREQUENCY   STM32L4_HCLK_FREQUENCY
#define BOARD_TIM17_FREQUENCY   STM32L4_HCLK_FREQUENCY
#define BOARD_LPTIM1_FREQUENCY  (STM32L4_HCLK_FREQUENCY / 2)
#define BOARD_LPTIM2_FREQUENCY  (STM32L4_HCLK_FREQUENCY / 2)

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
#endif /* __BOARDS_ARM_STM32L4_STM32L476_MDK_INCLUDE_STM32L476_MDK_CLOCKING_H */
