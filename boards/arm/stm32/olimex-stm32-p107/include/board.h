/****************************************************************************
 * boards/arm/stm32/olimex-stm32-p107/include/board.h
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

#ifndef __BOARDS_ARM_STM32_OLIMEX_STM32_P107_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_OLIMEX_STM32_P107_INCLUDE_BOARD_H

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

/* MCO output driven by PLL3. From above, we already have PLL3 input
 * frequency as:
 *
 *  STM32_PLL_PREDIV2 = 5, 25MHz / 5 => 5MHz
 */

#if defined(CONFIG_STM32_MII_MCO) || defined(CONFIG_STM32_RMII_MCO)
#  define BOARD_CFGR_MCO_SOURCE RCC_CFGR_PLL3CLK      /* Source: PLL3 */
#  define STM32_PLL_PLL3MUL     RCC_CFGR2_PLL3MULx10  /* MCO 5MHz * 10 = 50MHz */
#endif

/* Alternate function pin selections (auto-aliased for new pinmap) */

/* USART2 */

#define GPIO_USART2_TX     GPIO_ADJUST_MODE(GPIO_USART2_TX_0, GPIO_MODE_50MHz)
#define GPIO_USART2_RX     GPIO_USART2_RX_0
#define GPIO_USART2_CTS    GPIO_USART2_CTS_0
#define GPIO_USART2_RTS    GPIO_ADJUST_MODE(GPIO_USART2_RTS_0, GPIO_MODE_50MHz)
#define GPIO_USART2_CK     GPIO_ADJUST_MODE(GPIO_USART2_CK_0, GPIO_MODE_50MHz)

/* MCO */

#define GPIO_MCO           GPIO_ADJUST_MODE(GPIO_MCO_0, GPIO_MODE_50MHz)

/* Ethernet (MII/RMII) */

#define GPIO_ETH_MDC          GPIO_ADJUST_MODE(GPIO_ETH_MDC_0, GPIO_MODE_50MHz)
#define GPIO_ETH_MDIO         GPIO_ADJUST_MODE(GPIO_ETH_MDIO_0, GPIO_MODE_50MHz)
#define GPIO_ETH_MII_COL      GPIO_ETH_MII_COL_0
#define GPIO_ETH_MII_CRS      GPIO_ETH_MII_CRS_0
#define GPIO_ETH_MII_RX_CLK   GPIO_ETH_MII_RX_CLK_0
#define GPIO_ETH_MII_RXD0     GPIO_ETH_MII_RXD0_0
#define GPIO_ETH_MII_RXD1     GPIO_ETH_MII_RXD1_0
#define GPIO_ETH_MII_RXD2     GPIO_ETH_MII_RXD2_0
#define GPIO_ETH_MII_RXD3     GPIO_ETH_MII_RXD3_0
#define GPIO_ETH_MII_RX_DV    GPIO_ETH_MII_RX_DV_0
#define GPIO_ETH_MII_RX_ER    GPIO_ETH_MII_RX_ER_0
#define GPIO_ETH_MII_TX_CLK   GPIO_ETH_MII_TX_CLK_0
#define GPIO_ETH_MII_TXD0     GPIO_ADJUST_MODE(GPIO_ETH_MII_TXD0_0, GPIO_MODE_50MHz)
#define GPIO_ETH_MII_TXD1     GPIO_ADJUST_MODE(GPIO_ETH_MII_TXD1_0, GPIO_MODE_50MHz)
#define GPIO_ETH_MII_TXD2     GPIO_ADJUST_MODE(GPIO_ETH_MII_TXD2_0, GPIO_MODE_50MHz)
#define GPIO_ETH_MII_TXD3     GPIO_ADJUST_MODE(GPIO_ETH_MII_TXD3_0, GPIO_MODE_50MHz)
#define GPIO_ETH_MII_TX_EN    GPIO_ADJUST_MODE(GPIO_ETH_MII_TX_EN_0, GPIO_MODE_50MHz)
#define GPIO_ETH_RMII_CRS_DV  GPIO_ETH_RMII_CRS_DV_0
#define GPIO_ETH_RMII_REF_CLK GPIO_ETH_RMII_REF_CLK_0
#define GPIO_ETH_RMII_RXD0    GPIO_ETH_RMII_RXD0_0
#define GPIO_ETH_RMII_RXD1    GPIO_ETH_RMII_RXD1_0
#define GPIO_ETH_RMII_TXD0    GPIO_ADJUST_MODE(GPIO_ETH_RMII_TXD0_0, GPIO_MODE_50MHz)
#define GPIO_ETH_RMII_TXD1    GPIO_ADJUST_MODE(GPIO_ETH_RMII_TXD1_0, GPIO_MODE_50MHz)
#define GPIO_ETH_RMII_TX_EN   GPIO_ADJUST_MODE(GPIO_ETH_RMII_TX_EN_0, GPIO_MODE_50MHz)

#endif /* __BOARDS_ARM_STM32_OLIMEX_STM32_P107_INCLUDE_BOARD_H */
