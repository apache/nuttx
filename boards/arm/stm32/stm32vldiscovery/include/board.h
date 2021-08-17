/****************************************************************************
 * boards/arm/stm32/stm32vldiscovery/include/board.h
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

#ifndef __BOARDS_ARM_STM32_STM32VLDISCOVERY_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_STM32VLDISCOVERY_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#ifndef __ASSEMBLY__
# include <stdint.h>
#endif
#include "stm32_rcc.h"
#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* On-board crystal frequency is 8MHz (HSE) */

#define STM32_BOARD_XTAL        8000000ul

/* PLL source is HSE / 1,
 * PLL multiplier is 3: PLL output frequency is 8MHz (XTAL) x 3 = 24MHz
 */

#define STM32_CFGR2_PREDIV1     RCC_CFGR2_PREDIV1d1
#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx3
#define STM32_PLL_FREQUENCY     (3 * STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (24MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (24MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY

/* APB2 timers (1, 15-17) will receive PCLK2. */

#define STM32_APB2_TIM1_CLKIN   STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM15_CLKIN  STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM16_CLKIN  STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM17_CLKIN  STM32_PCLK2_FREQUENCY

/* APB1 clock (PCLK1) is HCLK (24MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY   STM32_HCLK_FREQUENCY

/* APB1 timers (2-7, 12-14) will receive PCLK1. */

#define STM32_APB1_TIM2_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM3_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM4_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM5_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM6_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM7_CLKIN   STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM12_CLKIN  STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM13_CLKIN  STM32_PCLK1_FREQUENCY
#define STM32_APB1_TIM14_CLKIN  STM32_PCLK1_FREQUENCY

/* Timer Frequencies, if APBx is set to 1, frequency is same to APBx
 * otherwise frequency is 2xAPBx.
 * Note: TIM1,15-17 are on APB2, others on APB1
 */

#define BOARD_TIM1_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY    STM32_HCLK_FREQUENCY
#define BOARD_TIM8_FREQUENCY    STM32_HCLK_FREQUENCY

/* LED definitions **********************************************************/

/* It is assumed that a generic board has 1 LED. Thus only two different
 * states can be shown. Statuses defined as "1" will light the LED, the
 * ones defined as "0" will turn the LED off.
 */

#define LED_STARTED       1
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   1
#define LED_STACKCREATED  1
#define LED_INIRQ         1
#define LED_SIGNAL        1
#define LED_ASSERTION     0
#define LED_PANIC         0

/* Button definitions *******************************************************/

/* It is assumed that a generic board has 1 button. */

#define BUTTON_0           0

#define NUM_BUTTONS        1

#define BUTTON_0_BIT       (1 << BUTTON_0)

#endif /* __BOARDS_ARM_STM32_STM32VLDISCOVERY_INCLUDE_BOARD_H */
