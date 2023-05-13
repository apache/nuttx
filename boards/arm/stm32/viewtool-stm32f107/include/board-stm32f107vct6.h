/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/include/board-stm32f107vct6.h
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

#ifndef __BOARDS_ARM_STM32_VIEWTOOL_STM32F107_INCLUDE_BOARD_STM32F107VCT6_H
#define __BOARDS_ARM_STM32_VIEWTOOL_STM32F107_INCLUDE_BOARD_STM32F107VCT6_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

#include "stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - 8 MHz RC factory-trimmed
 * LSI - 40 KHz RC (30-60KHz, uncalibrated)
 * HSE - On-board crystal frequency is 25MHz
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        25000000ul

#define STM32_HSI_FREQUENCY     8000000ul
#define STM32_LSI_FREQUENCY     40000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

/* PLL output is 72MHz */

#define STM32_PLL_PREDIV2       RCC_CFGR2_PREDIV2d5   /* 25MHz / 5 => 5MHz */
#define STM32_PLL_PLL2MUL       RCC_CFGR2_PLL2MULx8   /* 5MHz * 8  => 40MHz */
#define STM32_PLL_PREDIV1       RCC_CFGR2_PREDIV1d5   /* 40MHz / 5 => 8MHz */
#define STM32_PLL_PLLMUL        RCC_CFGR_PLLMUL_CLKx9 /* 8MHz * 9  => 72Mhz */
#define STM32_PLL_FREQUENCY     (72000000)

/* SYCLLK and HCLK are the PLL frequency */

#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY
#define STM32_HCLK_FREQUENCY    STM32_PLL_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (72MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)   /* Timers 2-7, 12-14 */

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

/* USB divider -- Divide PLL clock by 1.5
 *
 * USB clock = PLLOUT / 1.5 = 72MHz / 1.5 = 48MHz
 */

#define STM32_CFGR_OTGFSPRE     0

/* MCO output driven by PLL3.
 * From above, we already have PLL3 input frequency as:
 *
 *  STM32_PLL_PREDIV2 = 5, 25MHz / 5 => 5MHz
 *
 * NOTE: The Viewtool DP83848C module has its own, on-board 50MHz clock.  No
 * MCO clock need be provided on that board.
 */

#if defined(CONFIG_STM32_MII_MCO) || defined(CONFIG_STM32_RMII_MCO)
#  define BOARD_CFGR_MCO_SOURCE RCC_CFGR_PLL3CLK      /* Source: PLL3 */
#  define STM32_PLL_PLL3MUL     RCC_CFGR2_PLL3MULx10  /* MCO 5MHz * 10 = 50MHz */
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
#endif /* __BOARDS_ARM_STM32_VIEWTOOL_STM32F107_INCLUDE_BOARD_STM32F107VCT6_H */
