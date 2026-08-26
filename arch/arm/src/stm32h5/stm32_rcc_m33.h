/****************************************************************************
 * arch/arm/src/stm32h5/stm32_rcc_m33.h
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

#ifndef __ARCH_ARM_SRC_STM32H5_STM32_RCC_M33_H
#define __ARCH_ARM_SRC_STM32H5_STM32_RCC_M33_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "chip.h"

#if defined(CONFIG_STM32_STM32H5XXXX)
#  include "hardware/stm32_rcc.h"
#else
#  error "Unsupported STM32H5 chip"
#endif

/****************************************************************************
 * Pre-processor Definitions
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
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mco1config
 *
 * Description:
 *
 * Input Parameters:
 *   source - One of the definitions for the RCC_CFGR_MCO definitions from
 *     chip/stm32h5_rcc.h {RCC_CFGR_SYSCLK, RCC_CFGR_INTCLK,
 *     RCC_CFGR_EXTCLK, RCC_CFGR_PLLCLKd2, RCC_CFGR_PLL2CLK,
 *     RCC_CFGR_PLL3CLKd2, RCC_CFGR_XT1, RCC_CFGR_PLL3CLK}
 *   div - Clock divider passed through the RCC_CFGR_MCO1PRE macro from
 *     chip/stm32h5_rcc.h {RCC_CFGR_MCO1PRE(x) where x is 0..15})}
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_mco1config(uint32_t source, uint32_t div)
{
  uint32_t regval;

  /* Set MCO source */

  regval = getreg32(STM32_RCC_CFGR1);
  regval &= ~(RCC_CFGR1_MCO1SEL_MASK | RCC_CFGR1_MCO1PRE_MASK);
  regval |= (source | div);
  putreg32(regval, STM32_RCC_CFGR1);
}

/****************************************************************************
 * Name: stm32_mco2config
 *
 * Description:
 *
 * Input Parameters:
 *   source - One of the definitions for the RCC_CFGR_MCO definitions from
 *     chip/stm32h5_rcc.h {RCC_CFGR_SYSCLK, RCC_CFGR_INTCLK,
 *     RCC_CFGR_EXTCLK, RCC_CFGR_PLLCLKd2, RCC_CFGR_PLL2CLK,
 *     RCC_CFGR_PLL3CLKd2, RCC_CFGR_XT1, RCC_CFGR_PLL3CLK}
 *   div - Clock divider passed through the RCC_CFGR_MCO2PRE macro from
 *     chip/stm32h5_rcc.h {RCC_CFGR_MCO2PRE(x) where x is 0..15})}
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void stm32_mco2config(uint32_t source, uint32_t div)
{
  uint32_t regval;

  /* Set MCO source */

  regval = getreg32(STM32_RCC_CFGR1);
  regval &= ~(RCC_CFGR1_MCO2SEL_MASK | RCC_CFGR1_MCO2PRE_MASK);
  regval |= (source | div);
  putreg32(regval, STM32_RCC_CFGR1);
}

/* USART clock and RCC definitions */

#define STM32_LPUART1_FREQUENCY  STM32_PCLK3_FREQUENCY
#define STM32_LPUART1_RCC_REG    STM32_RCC_APB3ENR
#define STM32_LPUART1_RCC_EN     RCC_APB3ENR_LPUART1EN

#if !defined(STM32_RCC_CCIPR1_USART1SEL) || \
    STM32_RCC_CCIPR1_USART1SEL == RCC_CCIPR1_USART1SEL_RCCPCLK2
#  define STM32_USART1_FREQUENCY STM32_PCLK2_FREQUENCY
#elif STM32_RCC_CCIPR1_USART1SEL == RCC_CCIPR1_USART1SEL_PLL2QCK
#  define STM32_USART1_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART1SEL == RCC_CCIPR1_USART1SEL_PLL3QCK
#  define STM32_USART1_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART1SEL == RCC_CCIPR1_USART1SEL_HSIKERCK
#  define STM32_USART1_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART1SEL == RCC_CCIPR1_USART1SEL_CSIKERCK
#  define STM32_USART1_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART1SEL == RCC_CCIPR1_USART1SEL_LSECK
#  define STM32_USART1_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported USART1 Clock Selection"
#endif

#define STM32_USART1_RCC_REG     STM32_RCC_APB2ENR
#define STM32_USART1_RCC_EN      RCC_APB2ENR_USART1EN

#if !defined(STM32_RCC_CCIPR1_USART2SEL) || \
    STM32_RCC_CCIPR1_USART2SEL == RCC_CCIPR1_USART2SEL_RCCPCLK1
#  define STM32_USART2_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR1_USART2SEL == RCC_CCIPR1_USART2SEL_PLL2QCK
#  define STM32_USART2_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART2SEL == RCC_CCIPR1_USART2SEL_PLL3QCK
#  define STM32_USART2_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART2SEL == RCC_CCIPR1_USART2SEL_HSIKERCK
#  define STM32_USART2_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART2SEL == RCC_CCIPR1_USART2SEL_CSIKERCK
#  define STM32_USART2_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART2SEL == RCC_CCIPR1_USART2SEL_LSECK
#  define STM32_USART2_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported USART2 Clock Selection"
#endif

#define STM32_USART2_RCC_REG     STM32_RCC_APB1LENR
#define STM32_USART2_RCC_EN      RCC_APB1LENR_USART2EN

#if !defined(STM32_RCC_CCIPR1_USART3SEL) || \
    STM32_RCC_CCIPR1_USART3SEL == RCC_CCIPR1_USART3SEL_RCCPCLK1
#  define STM32_USART3_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR1_USART3SEL == RCC_CCIPR1_USART3SEL_PLL2QCK
#  define STM32_USART3_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART3SEL == RCC_CCIPR1_USART3SEL_PLL3QCK
#  define STM32_USART3_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART3SEL == RCC_CCIPR1_USART3SEL_HSIKERCK
#  define STM32_USART3_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART3SEL == RCC_CCIPR1_USART3SEL_CSIKERCK
#  define STM32_USART3_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART3SEL == RCC_CCIPR1_USART3SEL_LSECK
#  define STM32_USART3_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported USART3 Clock Selection"
#endif

#define STM32_USART3_RCC_REG     STM32_RCC_APB1LENR
#define STM32_USART3_RCC_EN      RCC_APB1LENR_USART3EN

#if !defined(STM32_RCC_CCIPR1_UART4SEL) || \
    STM32_RCC_CCIPR1_UART4SEL == RCC_CCIPR1_UART4SEL_RCCPCLK1
#  define STM32_UART4_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR1_UART4SEL == RCC_CCIPR1_UART4SEL_PLL2QCK
#  define STM32_UART4_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART4SEL == RCC_CCIPR1_UART4SEL_PLL3QCK
#  define STM32_UART4_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART4SEL == RCC_CCIPR1_UART4SEL_HSIKERCK
#  define STM32_UART4_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART4SEL == RCC_CCIPR1_UART4SEL_CSIKERCK
#  define STM32_UART4_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART4SEL == RCC_CCIPR1_UART4SEL_LSECK
#  define STM32_UART4_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported UART4 Clock Selection"
#endif

#define STM32_UART4_RCC_REG      STM32_RCC_APB1LENR
#define STM32_UART4_RCC_EN       RCC_APB1LENR_UART4EN

#if !defined(STM32_RCC_CCIPR1_UART5SEL) || \
    STM32_RCC_CCIPR1_UART5SEL == RCC_CCIPR1_UART5SEL_RCCPCLK1
#  define STM32_UART5_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR1_UART5SEL == RCC_CCIPR1_UART5SEL_PLL2QCK
#  define STM32_UART5_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART5SEL == RCC_CCIPR1_UART5SEL_PLL3QCK
#  define STM32_UART5_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART5SEL == RCC_CCIPR1_UART5SEL_HSIKERCK
#  define STM32_UART5_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART5SEL == RCC_CCIPR1_UART5SEL_CSIKERCK
#  define STM32_UART5_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART5SEL == RCC_CCIPR1_UART5SEL_LSECK
#  define STM32_UART5_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported UART5 Clock Selection"
#endif

#define STM32_UART5_RCC_REG      STM32_RCC_APB1LENR
#define STM32_UART5_RCC_EN       RCC_APB1LENR_UART5EN

#if !defined(STM32_RCC_CCIPR1_USART6SEL) || \
    STM32_RCC_CCIPR1_USART6SEL == RCC_CCIPR1_USART6SEL_RCCPCLK1
#  define STM32_USART6_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR1_USART6SEL == RCC_CCIPR1_USART6SEL_PLL2QCK
#  define STM32_USART6_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART6SEL == RCC_CCIPR1_USART6SEL_PLL3QCK
#  define STM32_USART6_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART6SEL == RCC_CCIPR1_USART6SEL_HSIKERCK
#  define STM32_USART6_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART6SEL == RCC_CCIPR1_USART6SEL_CSIKERCK
#  define STM32_USART6_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART6SEL == RCC_CCIPR1_USART6SEL_LSECK
#  define STM32_USART6_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported USART6 Clock Selection"
#endif

#define STM32_USART6_RCC_REG     STM32_RCC_APB1LENR
#define STM32_USART6_RCC_EN      RCC_APB1LENR_USART6EN

#if !defined(STM32_RCC_CCIPR1_UART7SEL) || \
    STM32_RCC_CCIPR1_UART7SEL == RCC_CCIPR1_UART7SEL_RCCPCLK1
#  define STM32_UART7_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR1_UART7SEL == RCC_CCIPR1_UART7SEL_PLL2QCK
#  define STM32_UART7_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART7SEL == RCC_CCIPR1_UART7SEL_PLL3QCK
#  define STM32_UART7_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART7SEL == RCC_CCIPR1_UART7SEL_HSIKERCK
#  define STM32_UART7_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART7SEL == RCC_CCIPR1_UART7SEL_CSIKERCK
#  define STM32_UART7_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART7SEL == RCC_CCIPR1_UART7SEL_LSECK
#  define STM32_UART7_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported UART7 Clock Selection"
#endif

#define STM32_UART7_RCC_REG      STM32_RCC_APB1LENR
#define STM32_UART7_RCC_EN       RCC_APB1LENR_UART7EN

#if !defined(STM32_RCC_CCIPR1_UART8SEL) || \
    STM32_RCC_CCIPR1_UART8SEL == RCC_CCIPR1_UART8SEL_RCCPCLK1
#  define STM32_UART8_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR1_UART8SEL == RCC_CCIPR1_UART8SEL_PLL2QCK
#  define STM32_UART8_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART8SEL == RCC_CCIPR1_UART8SEL_PLL3QCK
#  define STM32_UART8_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART8SEL == RCC_CCIPR1_UART8SEL_HSIKERCK
#  define STM32_UART8_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART8SEL == RCC_CCIPR1_UART8SEL_CSIKERCK
#  define STM32_UART8_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART8SEL == RCC_CCIPR1_UART8SEL_LSECK
#  define STM32_UART8_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported UART8 Clock Selection"
#endif

#define STM32_UART8_RCC_REG      STM32_RCC_APB1LENR
#define STM32_UART8_RCC_EN       RCC_APB1LENR_UART8EN

#if !defined(STM32_RCC_CCIPR1_UART9SEL) || \
    STM32_RCC_CCIPR1_UART9SEL == RCC_CCIPR1_UART9SEL_RCCPCLK1
#  define STM32_UART9_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR1_UART9SEL == RCC_CCIPR1_UART9SEL_PLL2QCK
#  define STM32_UART9_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART9SEL == RCC_CCIPR1_UART9SEL_PLL3QCK
#  define STM32_UART9_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_UART9SEL == RCC_CCIPR1_UART9SEL_HSIKERCK
#  define STM32_UART9_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART9SEL == RCC_CCIPR1_UART9SEL_CSIKERCK
#  define STM32_UART9_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_UART9SEL == RCC_CCIPR1_UART9SEL_LSECK
#  define STM32_UART9_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported UART9 Clock Selection"
#endif

#define STM32_UART9_RCC_REG      STM32_RCC_APB1HENR
#define STM32_UART9_RCC_EN       RCC_APB1HENR_UART9EN

#if !defined(STM32_RCC_CCIPR1_USART10SEL) || \
    STM32_RCC_CCIPR1_USART10SEL == RCC_CCIPR1_USART10SEL_RCCPCLK1
#  define STM32_USART10_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR1_USART10SEL == RCC_CCIPR1_USART10SEL_PLL2QCK
#  define STM32_USART10_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART10SEL == RCC_CCIPR1_USART10SEL_PLL3QCK
#  define STM32_USART10_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR1_USART10SEL == RCC_CCIPR1_USART10SEL_HSIKERCK
#  define STM32_USART10_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART10SEL == RCC_CCIPR1_USART10SEL_CSIKERCK
#  define STM32_USART10_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR1_USART10SEL == RCC_CCIPR1_USART10SEL_LSECK
#  define STM32_USART10_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported USART10 Clock Selection"
#endif

#define STM32_USART10_RCC_REG    STM32_RCC_APB1LENR
#define STM32_USART10_RCC_EN     RCC_APB1LENR_USART10EN

#if !defined(STM32_RCC_CCIPR2_USART11SEL) || \
    STM32_RCC_CCIPR2_USART11SEL == RCC_CCIPR2_USART11SEL_RCCPCLK1
#  define STM32_USART11_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR2_USART11SEL == RCC_CCIPR2_USART11SEL_PLL2QCK
#  define STM32_USART11_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR2_USART11SEL == RCC_CCIPR2_USART11SEL_PLL3QCK
#  define STM32_USART11_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR2_USART11SEL == RCC_CCIPR2_USART11SEL_HSIKERCK
#  define STM32_USART11_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR2_USART11SEL == RCC_CCIPR2_USART11SEL_CSIKERCK
#  define STM32_USART11_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR2_USART11SEL == RCC_CCIPR2_USART11SEL_LSECK
#  define STM32_USART11_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported USART11 Clock Selection"
#endif

#define STM32_USART11_RCC_REG    STM32_RCC_APB1LENR
#define STM32_USART11_RCC_EN     RCC_APB1LENR_USART11EN

#if !defined(STM32_RCC_CCIPR2_UART12SEL) || \
    STM32_RCC_CCIPR2_UART12SEL == RCC_CCIPR2_UART12SEL_RCCPCLK1
#  define STM32_UART12_FREQUENCY STM32_PCLK1_FREQUENCY
#elif STM32_RCC_CCIPR2_UART12SEL == RCC_CCIPR2_UART12SEL_PLL2QCK
#  define STM32_UART12_FREQUENCY STM32_PLL2Q_FREQUENCY
#elif STM32_RCC_CCIPR2_UART12SEL == RCC_CCIPR2_UART12SEL_PLL3QCK
#  define STM32_UART12_FREQUENCY STM32_PLL3Q_FREQUENCY
#elif STM32_RCC_CCIPR2_UART12SEL == RCC_CCIPR2_UART12SEL_HSIKERCK
#  define STM32_UART12_FREQUENCY STM32_HSI_FREQUENCY
#elif STM32_RCC_CCIPR2_UART12SEL == RCC_CCIPR2_UART12SEL_CSIKERCK
#  define STM32_UART12_FREQUENCY STM32_CSI_FREQUENCY
#elif STM32_RCC_CCIPR2_UART12SEL == RCC_CCIPR2_UART12SEL_LSECK
#  define STM32_UART12_FREQUENCY STM32_LSE_FREQUENCY
#else
#  error "Unsupported UART12 Clock Selection"
#endif

#define STM32_UART12_RCC_REG     STM32_RCC_APB1HENR
#define STM32_UART12_RCC_EN      RCC_APB1HENR_UART12EN

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
 *   This function (by default) will reset most everything, enable the PLL,
 *   and enable peripheral clocking for all periperipherals enabled in the
 *   NuttX configuration file.
 *
 *   If CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_clockconfig(void);

/****************************************************************************
 * Name: stm32_board_clockconfig
 *
 * Description:
 *   Any STM32H5 board may replace the "standard" board clock configuration
 *   logic with its own, custom clock configuration logic.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
void stm32_board_clockconfig(void);
#endif

/****************************************************************************
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   The standard logic to configure the clocks based on settings in board.h.
 *   Applicable if no custom clock config is provided.  This function is
 *   chip type specific and implemented in corresponding modules such as e.g.
 *   stm32h562xx_rcc.c
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
void stm32_stdclockconfig(void);
#endif

/****************************************************************************
 * Name: stm32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.  This function is only available to support low-power modes of
 *   operation:  When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
 *
 *   This function performs a subset of the operations performed by
 *   stm32_clockconfig():  It does not reset any devices, and it does not
 *   reset the currently enabled peripheral clocks.
 *
 *   If CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG is defined, then
 *   clocking will be enabled by an externally provided, board-specific
 *   function called stm32_board_clockconfig().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_PM
void stm32_clockenable(void);
#endif

/****************************************************************************
 * Name: stm32_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) Oscillator.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void stm32_rcc_enablelse(void);

/****************************************************************************
 * Name: stm32_rcc_enablelsi
 *
 * Description:
 *   Enable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32_rcc_enablelsi(void);

/****************************************************************************
 * Name: stm32_rcc_disablelsi
 *
 * Description:
 *   Disable the Internal Low-Speed (LSI) RC Oscillator.
 *
 ****************************************************************************/

void stm32_rcc_disablelsi(void);

/****************************************************************************
 * Name: stm32_rcc_enableperipherals
 *
 * Description:
 *   Enable all the chip peripherals according to configuration.  This is
 *   chip type specific and thus implemented in corresponding modules such as
 *   e.g. stm32h562xx_rcc.c
 *
 ****************************************************************************/

void stm32_rcc_enableperipherals(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32H5_STM32_RCC_M33_H */
