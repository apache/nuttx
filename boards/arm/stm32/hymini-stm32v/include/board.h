/****************************************************************************
 * boards/arm/stm32/hymini-stm32v/include/board.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Copyright (C) 2011 Laurent Latil. All rights reserved.
 *   Author: Laurent Latil <laurent@latil.nom.fr>
 *           Gregory Nutt <gnutt@nuttx.org>
 *
 * Derives, in part, from boards/stm3210e-eval/include/board.h
 *
 *   Copyright (C) 2009 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_STM32_HYMINI_STM32V_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_HYMINI_STM32V_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *************************************************************************/

/* On-board crystal frequency is 8MHz (HSE) */

#define STM32_BOARD_XTAL        8000000ul

/* PLL source is HSE/1, PLL multipler is 9: PLL frequency is 8MHz (XTAL) x 9 = 72MHz */

#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx9
#define STM32_PLL_FREQUENCY     (9*STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (72MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY
#define STM32_BOARD_HCLK        STM32_HCLK_FREQUENCY    /* same as above, to satisfy compiler */

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY

/* APB2 timers 1 and 8 will receive PCLK2. */

#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (36MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* APB1 timers 2-7 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (2*STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (2*STM32_PCLK1_FREQUENCY)

/* USB divider -- Divide PLL clock by 1.5 */

#define STM32_CFGR_USBPRE       0

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,8 are on APB2, others on APB1 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* SDIO dividers.  Note that slower clocking is required when DMA is disabled
 * in order to avoid RX overrun/TX underrun errors due to delayed responses
 * to service FIFOs in interrupt driven mode.  These values have not been
 * tuned!!!
 *
 * HCLK=72MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(178+2)=400 KHz
 */

#define SDIO_INIT_CLKDIV        (178 << SDIO_CLKCR_CLKDIV_SHIFT)

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(2+2)=18 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

#ifdef CONFIG_STM32_SDIO_DMA
#  define SDIO_MMCXFR_CLKDIV    (2 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_MMCXFR_CLKDIV    (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* DMA ON:  HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(1+2)=24 MHz
 * DMA OFF: HCLK=72 MHz, SDIOCLK=72MHz, SDIO_CK=HCLK/(3+2)=14.4 MHz
 */

#ifdef CONFIG_STM32_SDIO_DMA
#  define SDIO_SDXFR_CLKDIV     (1 << SDIO_CLKCR_CLKDIV_SHIFT)
#else
#  define SDIO_SDXFR_CLKDIV     (3 << SDIO_CLKCR_CLKDIV_SHIFT)
#endif

/* LED definitions ******************************************************************/

/* The board has 2 LEDs that we will encode as: */
#define LED_STARTED       0  /* No LEDs */
#define LED_HEAPALLOCATE  1  /* LED1 on */
#define LED_IRQSENABLED   2  /* LED2 on */
#define LED_STACKCREATED  3  /* LED1 on */
#define LED_INIRQ         4  /* LED1 off */
#define LED_SIGNAL        5  /* LED2 on */
#define LED_ASSERTION     6  /* LED1 + LED2 */
#define LED_PANIC         7  /* LED1 / LED2 blinking */

/* The board supports two user buttons
 *
 * KeyA            -- Connected to PC.13
 * KeyB            -- Connected to PB.2
 */

#define BUTTON_KEYA      0
#define BUTTON_KEYB      1

#define NUM_BUTTONS      2

#define BUTTON_KEYA_BIT  (1 << BUTTON_KEYA)
#define BUTTON_KEYB_BIT  (1 << BUTTON_KEYB)

#endif /* __BOARDS_ARM_STM32_HYMINI_STM32V_INCLUDE_BOARD_H */
