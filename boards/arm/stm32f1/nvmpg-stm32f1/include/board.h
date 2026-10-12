/****************************************************************************
 * boards/arm/stm32f1/nvmpg-stm32f1/include/board.h
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

#ifndef __BOARDS_ARM_STM32F1_NVMPG_STM32F1_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F1_NVMPG_STM32F1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The original NVMPG firmware runs the PLL from HSI/2 even though it starts
 * the HSE, so the crystal value is not known for sure.  By default we do the
 * same thing (HSI/2 x 16 = 64MHz), which works whatever crystal is fitted.
 * Select CONFIG_NVMPG_STM32F1_HSE_8MHZ if the board has an 8MHz crystal to
 * run at 72MHz.
 */

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000            /* Between 30kHz and 60kHz */
#define STM32_LSE_FREQUENCY     32768

#ifdef CONFIG_NVMPG_STM32F1_HSE_8MHZ
#  define STM32_BOARD_XTAL      8000000ul
#  define STM32_HSE_FREQUENCY   STM32_BOARD_XTAL

/* PLL source is HSE/1, PLL multiplier is 9: 8MHz x 9 = 72MHz */

#  define STM32_CFGR_PLLSRC     RCC_CFGR_PLLSRC
#  define STM32_CFGR_PLLXTPRE   0
#  define STM32_CFGR_PLLMUL     RCC_CFGR_PLLMUL_CLKx9
#  define STM32_PLL_FREQUENCY   (9*STM32_BOARD_XTAL)
#else
#  define STM32_HSE_FREQUENCY   0

/* PLL source is HSI/2, PLL multiplier is 16: 4MHz x 16 = 64MHz */

#  define STM32_CFGR_PLLSRC     0
#  define STM32_CFGR_PLLXTPRE   0
#  define STM32_CFGR_PLLMUL     RCC_CFGR_PLLMUL_CLKx16
#  define STM32_PLL_FREQUENCY   (16*STM32_HSI_FREQUENCY/2)
#endif

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers */

#define STM32_TIM1_CLKIN        (STM32_PCLK2_FREQUENCY)
#define STM32_TIM8_CLKIN        (STM32_PCLK2_FREQUENCY)
#define STM32_TIM2_CLKIN        (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM3_CLKIN        (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM4_CLKIN        (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM5_CLKIN        (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM6_CLKIN        (2*STM32_PCLK1_FREQUENCY)
#define STM32_TIM7_CLKIN        (2*STM32_PCLK1_FREQUENCY)

/* LCD **********************************************************************/

/* ILI9225 176x220 TFT used in landscape: 220 columns x 176 rows */

#define NVMPG_LCD_XRES          220
#define NVMPG_LCD_YRES          176

/* Keys *********************************************************************/

/* Ten push buttons, active low.  The original firmware sends key index + 2
 * to the controller when a key is pressed, and (index + 2) | 0x80 when it is
 * released.  They are exported as /dev/gpio0 .. /dev/gpio9.
 */

#define NVMPG_NKEYS             10

/* Alternate function pin selections ****************************************/

/* USART1: link to the NVUM/NVEM controller (PA9 = TX, PA10 = RX) */

#define GPIO_USART1_TX          GPIO_ADJUST_MODE(GPIO_USART1_TX_0, GPIO_MODE_50MHz)
#define GPIO_USART1_RX          GPIO_USART1_RX_0

#endif /* __BOARDS_ARM_STM32F1_NVMPG_STM32F1_INCLUDE_BOARD_H */
