/****************************************************************************
 * boards/arm/stm32/nucleo-f4x1re/include/nucleo-f411re.h
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __BOARDS_ARM_STM32_NUCLEO_F401RE_INCLUDE_NUCLEO_F411RE_H
#define __BOARDS_ARM_STM32_NUCLEO_F401RE_INCLUDE_NUCLEO_F411RE_H

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

/* Clocking *************************************************************************/
/* The NUCLEOF411RE supports both HSE and LSE crystals (X2 and X3).  However, as
 * shipped, the X2 and X3 crystals are not populated.  Therefore the Nucleo-FF411RE
 * will need to run off the 16MHz HSI clock.
 *
 *   System Clock source           : PLL (HSI)
 *   SYSCLK(Hz)                    : 104000000    Determined by PLL configuration
 *   HCLK(Hz)                      : 104000000    (STM32_RCC_CFGR_HPRE)
 *   AHB Prescaler                 : 1            (STM32_RCC_CFGR_HPRE)
 *   APB1 Prescaler                : 2            (STM32_RCC_CFGR_PPRE1)
 *   APB2 Prescaler                : 1            (STM32_RCC_CFGR_PPRE2)
 *   HSI Frequency(Hz)             : 16000000     (nominal)
 *   PLLM                          : 8            (STM32_PLLCFG_PLLM)
 *   PLLN                          : 216          (STM32_PLLCFG_PLLN)
 *   PLLP                          : 4            (STM32_PLLCFG_PLLP)
 *   PLLQ                          : 9            (STM32_PLLCFG_PPQ)
 *   Flash Latency(WS)             : 4
 *   Prefetch Buffer               : OFF
 *   Instruction cache             : ON
 *   Data cache                    : ON
 *   Require 48MHz for USB OTG FS, : Enabled
 *   SDIO and RNG clock
 */

/* HSI - 16 MHz RC factory-trimmed
 * LSI - 32 KHz RC
 * HSE - not installed
 * LSE - not installed
 */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_BOARD_USEHSI      1

/* Main PLL Configuration.
 *
 * Formulae:
 *
 *   VCO input frequency        = PLL input clock frequency / PLLM, 2 <= PLLM <= 63
 *   VCO output frequency       = VCO input frequency × PLLN,       192 <= PLLN <= 432
 *   PLL output clock frequency = VCO frequency / PLLP,             PLLP = 2, 4, 6, or 8
 *   USB OTG FS clock frequency = VCO frequency / PLLQ,             2 <= PLLQ <= 15
 *

 * There is no config for 100 MHz and 48 MHz for usb,
 * so we would like to have SYSYCLK=104MHz and we must have the USB clock= 48MHz.
 *
 * PLLQ = 13 PLLP = 6 PLLN=390 PLLM=10
 *
 * We will configure like this
 *
 *   PLL source is HSI
 *   PLL_VCO = (STM32_HSI_FREQUENCY / PLLM) * PLLN
 *           = (16,000,000 / 10) * 390
 *           = 624,000,000
 *   SYSCLK  = PLL_VCO / PLLP
 *           = 624,000,000 / 6 = 104,000,000
 *   USB OTG FS and SDIO Clock
 *           = PLL_VCO / PLLQ
 *           = 624,000,000 / 13 = 48,000,000
 *
 * REVISIT: Trimming of the HSI is not yet supported.
 */

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(10)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(390)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_6
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(13)

#define STM32_SYSCLK_FREQUENCY  104000000ul

/* AHB clock (HCLK) is SYSCLK (104MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK      /* HCLK  = SYSCLK / 1 */
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY      /* Same as above, to satisfy compiler */

/* APB1 clock (PCLK1) is HCLK/2 (52MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2     /* PCLK1 = HCLK / 2 */
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers driven from APB1 will be twice PCLK1 */
/* REVISIT */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (104MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK       /* PCLK2 = HCLK / 1 */
#define STM32_PCLK2_FREQUENCY   (STM32_HCLK_FREQUENCY/1)

/* Timers driven from APB2 will be twice PCLK2 */
/* REVISIT */

#define STM32_APB2_TIM1_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (2*STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM9_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB2_TIM10_CLKIN  (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB2_TIM11_CLKIN  (2*STM32_PCLK1_FREQUENCY)

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1
 */
/* REVISIT */

#define BOARD_TIM1_FREQUENCY    (2*STM32_PCLK2_FREQUENCY)
#define BOARD_TIM2_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM3_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM4_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM5_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM6_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM7_FREQUENCY    (2*STM32_PCLK1_FREQUENCY)
#define BOARD_TIM8_FREQUENCY    (2*STM32_PCLK2_FREQUENCY)

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * HCLK=72MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(178+2)=400 KHz
 */
/* REVISIT */

#define SDIO_INIT_CLKDIV        (178 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(2+2)=18 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */
/* REVISIT */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(1+2)=24 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */
/* REVISIT */

#ifdef CONFIG_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

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
#endif /* __BOARDS_ARM_STM32_NUCLEO_F401RE_INCLUDE_NUCLEO_F411RE_H */
