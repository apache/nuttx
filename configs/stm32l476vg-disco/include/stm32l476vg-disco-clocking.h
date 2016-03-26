/************************************************************************************
 * configs/stm32l476vg-disco/include/stm32l476vg-disco-clocking.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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
 ************************************************************************************/

#ifndef __CONFIGS_STM32L476VG_DISCO_INCLUDE_STM32L476VG_DISCO_CLOCKING_H
#define __CONFIGS_STM32L476VG_DISCO_INCLUDE_STM32L476VG_DISCO_CLOCKING_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/

/* The stm32l476vg-disco supports both HSE and LSE crystals (X2 and X3).  However, as
 * shipped, the HSE X2 crystal is not populated.  Therefore the stm32l476vg-disco
 * will need to run off the 16MHz HSI clock, or the 32khz-synced MSI.
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * MSI - variable up to 48 MHz, synchronized to LSE
 * HSE - not installed
 * LSE - 32.768 kHz installed
 */

#define STM32L4_HSI_FREQUENCY     16000000ul
#define STM32L4_LSI_FREQUENCY     32000
#define STM32L4_LSE_FREQUENCY     32768

/* XXX review the STM32L4_BOARD_USEHSI usage, it has too much influence in
 * stm32l4x6xx_rcc.c.  I suspect it is fine for it to turn on and off that
 * ocillator, but really that's all it should do (e.g. it also controls
 * input of teh PLLs.  Also, it should be fine/desireable to support things
 * like turning on both HSI and MSI, because they plausibly can both be
 * used at the same time; currently those choices HSE/HSI16/MSI are
 * mutually exclusive.
 */

#define STM32L4_BOARD_USEHSI      1

/* prescaler common to all PLL inputs; will be 1 (XXX source is implicitly
 as per comment above HSI) */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)

/* 'main' PLL config; we use this to generate our system clock via the R
 *  output.  We set it up as 16 MHz / 1 * 10 / 2 = 80 MHz
 *
 * XXX NOTE:  currently the main PLL is implicitly turned on and is implicitly
 * the system clock; this should be configurable since not all applications may
 * want things done this way.
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
#define  STM32L4_PLLSAI1CFG_PLLQ_ENABLED
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

#define STM32L4_USE_CLK48
#define STM32L4_CLK48_SEL         RCC_CCIPR_CLK48SEL_PLLSAI1

/* AHB clock (HCLK) is SYSCLK (80MHz) */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY
#define STM32L4_BOARD_HCLK        STM32L4_HCLK_FREQUENCY    /* Same as above, to satisfy compiler */

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
/* REVISIT : this can be configured */

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_STM32L476VG_DISCO_INCLUDE_STM32L476VG_DISCO_CLOCKING_H */
