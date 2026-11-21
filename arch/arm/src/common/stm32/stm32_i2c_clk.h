/****************************************************************************
 * arch/arm/src/common/stm32/stm32_i2c_clk.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_I2C_CLK_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_I2C_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/irq.h>

#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map each I2C instance to its RCC enable and reset register and bit.  The
 * bus differs between families: I2C1/2 sit on APB1 everywhere, I2C3 on
 * APB1 or APB3 (H5), I2C4 on APB1 (F7, G4), APB3 (H5) or APB4 (H7, N6).
 */

#if defined(RCC_APB1LENR_I2C1EN)
#  define STM32_RCC_I2C1_EN_REG    STM32_RCC_APB1LENR
#  define STM32_RCC_I2C1_EN        RCC_APB1LENR_I2C1EN
#  define STM32_RCC_I2C1_RST_REG   STM32_RCC_APB1LRSTR
#  define STM32_RCC_I2C1_RST       RCC_APB1LRSTR_I2C1RST
#elif defined(RCC_APB1ENR1_I2C1EN)
#  define STM32_RCC_I2C1_EN_REG    STM32_RCC_APB1ENR1
#  define STM32_RCC_I2C1_EN        RCC_APB1ENR1_I2C1EN
#  define STM32_RCC_I2C1_RST_REG   STM32_RCC_APB1RSTR1
#  define STM32_RCC_I2C1_RST       RCC_APB1RSTR1_I2C1RST
#elif defined(RCC_APB1ENR_I2C1EN)
#  define STM32_RCC_I2C1_EN_REG    STM32_RCC_APB1ENR
#  define STM32_RCC_I2C1_EN        RCC_APB1ENR_I2C1EN
#  define STM32_RCC_I2C1_RST_REG   STM32_RCC_APB1RSTR
#  define STM32_RCC_I2C1_RST       RCC_APB1RSTR_I2C1RST
#endif

#if defined(RCC_APB1LENR_I2C2EN)
#  define STM32_RCC_I2C2_EN_REG    STM32_RCC_APB1LENR
#  define STM32_RCC_I2C2_EN        RCC_APB1LENR_I2C2EN
#  define STM32_RCC_I2C2_RST_REG   STM32_RCC_APB1LRSTR
#  define STM32_RCC_I2C2_RST       RCC_APB1LRSTR_I2C2RST
#elif defined(RCC_APB1ENR1_I2C2EN)
#  define STM32_RCC_I2C2_EN_REG    STM32_RCC_APB1ENR1
#  define STM32_RCC_I2C2_EN        RCC_APB1ENR1_I2C2EN
#  define STM32_RCC_I2C2_RST_REG   STM32_RCC_APB1RSTR1
#  define STM32_RCC_I2C2_RST       RCC_APB1RSTR1_I2C2RST
#elif defined(RCC_APB1ENR_I2C2EN)
#  define STM32_RCC_I2C2_EN_REG    STM32_RCC_APB1ENR
#  define STM32_RCC_I2C2_EN        RCC_APB1ENR_I2C2EN
#  define STM32_RCC_I2C2_RST_REG   STM32_RCC_APB1RSTR
#  define STM32_RCC_I2C2_RST       RCC_APB1RSTR_I2C2RST
#endif

#if defined(RCC_APB1LENR_I2C3EN)
#  define STM32_RCC_I2C3_EN_REG    STM32_RCC_APB1LENR
#  define STM32_RCC_I2C3_EN        RCC_APB1LENR_I2C3EN
#  define STM32_RCC_I2C3_RST_REG   STM32_RCC_APB1LRSTR
#  define STM32_RCC_I2C3_RST       RCC_APB1LRSTR_I2C3RST
#elif defined(RCC_APB3ENR_I2C3EN)
#  define STM32_RCC_I2C3_EN_REG    STM32_RCC_APB3ENR
#  define STM32_RCC_I2C3_EN        RCC_APB3ENR_I2C3EN
#  define STM32_RCC_I2C3_RST_REG   STM32_RCC_APB3RSTR
#  define STM32_RCC_I2C3_RST       RCC_APB3RSTR_I2C3RST
#elif defined(RCC_APB1ENR1_I2C3EN)
#  define STM32_RCC_I2C3_EN_REG    STM32_RCC_APB1ENR1
#  define STM32_RCC_I2C3_EN        RCC_APB1ENR1_I2C3EN
#  define STM32_RCC_I2C3_RST_REG   STM32_RCC_APB1RSTR1
#  define STM32_RCC_I2C3_RST       RCC_APB1RSTR1_I2C3RST
#elif defined(RCC_APB1ENR_I2C3EN)
#  define STM32_RCC_I2C3_EN_REG    STM32_RCC_APB1ENR
#  define STM32_RCC_I2C3_EN        RCC_APB1ENR_I2C3EN
#  define STM32_RCC_I2C3_RST_REG   STM32_RCC_APB1RSTR
#  define STM32_RCC_I2C3_RST       RCC_APB1RSTR_I2C3RST
#endif

#if defined(RCC_APB4LENR_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB4LENR
#  define STM32_RCC_I2C4_EN        RCC_APB4LENR_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB4LRSTR
#  define STM32_RCC_I2C4_RST       RCC_APB4LRSTR_I2C4RST
#elif defined(RCC_APB3ENR_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB3ENR
#  define STM32_RCC_I2C4_EN        RCC_APB3ENR_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB3RSTR
#  define STM32_RCC_I2C4_RST       RCC_APB3RSTR_I2C4RST
#elif defined(RCC_APB4ENR_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB4ENR
#  define STM32_RCC_I2C4_EN        RCC_APB4ENR_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB4RSTR
#  define STM32_RCC_I2C4_RST       RCC_APB4RSTR_I2C4RST
#elif defined(RCC_APB1ENR1_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB1ENR1
#  define STM32_RCC_I2C4_EN        RCC_APB1ENR1_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB1RSTR1
#  define STM32_RCC_I2C4_RST       RCC_APB1RSTR1_I2C4RST
#elif defined(RCC_APB1ENR2_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB1ENR2
#  define STM32_RCC_I2C4_EN        RCC_APB1ENR2_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB1RSTR2
#  define STM32_RCC_I2C4_RST       RCC_APB1RSTR2_I2C4RST
#elif defined(RCC_APB1ENR_I2C4EN)
#  define STM32_RCC_I2C4_EN_REG    STM32_RCC_APB1ENR
#  define STM32_RCC_I2C4_EN        RCC_APB1ENR_I2C4EN
#  define STM32_RCC_I2C4_RST_REG   STM32_RCC_APB1RSTR
#  define STM32_RCC_I2C4_RST       RCC_APB1RSTR_I2C4RST
#endif

/* Kernel clock frequency of each instance, unless board.h gives
 * STM32_I2Cn_FREQUENCY: the clock that the family's RCC code selects.
 * Other clock sources need STM32_I2Cn_FREQUENCY in board.h.
 */

#if defined(CONFIG_STM32_STM32F30XX) || defined(CONFIG_STM32_STM32F33XX) || \
    defined(CONFIG_STM32_STM32F37XX)

/* I2CnSW stays HSI (reset value) */

#  ifndef STM32_I2C1_FREQUENCY
#    define STM32_I2C1_FREQUENCY  STM32_HSI_FREQUENCY
#  endif

#  ifndef STM32_I2C2_FREQUENCY
#    define STM32_I2C2_FREQUENCY  STM32_HSI_FREQUENCY
#  endif

#  ifndef STM32_I2C3_FREQUENCY
#    define STM32_I2C3_FREQUENCY  STM32_HSI_FREQUENCY
#  endif

#elif defined(CONFIG_STM32_STM32G4XXX)

/* I2C1-3 are set to HSI16 by stm32_rcc.c, I2C4 stays PCLK1 (reset) */

#  ifndef STM32_I2C1_FREQUENCY
#    define STM32_I2C1_FREQUENCY  STM32_HSI_FREQUENCY
#  endif

#  ifndef STM32_I2C2_FREQUENCY
#    define STM32_I2C2_FREQUENCY  STM32_HSI_FREQUENCY
#  endif

#  ifndef STM32_I2C3_FREQUENCY
#    define STM32_I2C3_FREQUENCY  STM32_HSI_FREQUENCY
#  endif

#  ifndef STM32_I2C4_FREQUENCY
#    define STM32_I2C4_FREQUENCY  STM32_PCLK1_FREQUENCY
#  endif

#elif defined(CONFIG_ARCH_CHIP_STM32F7)

/* Source from STM32_RCC_DCKCFGR2_I2CnSRC in board.h */

#  if !defined(STM32_I2C1_FREQUENCY) && defined(STM32_RCC_DCKCFGR2_I2C1SRC)
#    if STM32_RCC_DCKCFGR2_I2C1SRC == RCC_DCKCFGR2_I2C1SEL_HSI
#      define STM32_I2C1_FREQUENCY  STM32_HSI_FREQUENCY
#    elif STM32_RCC_DCKCFGR2_I2C1SRC == RCC_DCKCFGR2_I2C1SEL_SYSCLK
#      define STM32_I2C1_FREQUENCY  STM32_SYSCLK_FREQUENCY
#    elif STM32_RCC_DCKCFGR2_I2C1SRC == RCC_DCKCFGR2_I2C1SEL_APB
#      define STM32_I2C1_FREQUENCY  STM32_PCLK1_FREQUENCY
#    endif
#  endif

#  if !defined(STM32_I2C2_FREQUENCY) && defined(STM32_RCC_DCKCFGR2_I2C2SRC)
#    if STM32_RCC_DCKCFGR2_I2C2SRC == RCC_DCKCFGR2_I2C2SEL_HSI
#      define STM32_I2C2_FREQUENCY  STM32_HSI_FREQUENCY
#    elif STM32_RCC_DCKCFGR2_I2C2SRC == RCC_DCKCFGR2_I2C2SEL_SYSCLK
#      define STM32_I2C2_FREQUENCY  STM32_SYSCLK_FREQUENCY
#    elif STM32_RCC_DCKCFGR2_I2C2SRC == RCC_DCKCFGR2_I2C2SEL_APB
#      define STM32_I2C2_FREQUENCY  STM32_PCLK1_FREQUENCY
#    endif
#  endif

#  if !defined(STM32_I2C3_FREQUENCY) && defined(STM32_RCC_DCKCFGR2_I2C3SRC)
#    if STM32_RCC_DCKCFGR2_I2C3SRC == RCC_DCKCFGR2_I2C3SEL_HSI
#      define STM32_I2C3_FREQUENCY  STM32_HSI_FREQUENCY
#    elif STM32_RCC_DCKCFGR2_I2C3SRC == RCC_DCKCFGR2_I2C3SEL_SYSCLK
#      define STM32_I2C3_FREQUENCY  STM32_SYSCLK_FREQUENCY
#    elif STM32_RCC_DCKCFGR2_I2C3SRC == RCC_DCKCFGR2_I2C3SEL_APB
#      define STM32_I2C3_FREQUENCY  STM32_PCLK1_FREQUENCY
#    endif
#  endif

#  if defined(CONFIG_STM32_I2C4) && !defined(STM32_I2C4_FREQUENCY) && \
      defined(STM32_RCC_DCKCFGR2_I2C4SRC)
#    if STM32_RCC_DCKCFGR2_I2C4SRC == RCC_DCKCFGR2_I2C4SEL_HSI
#      define STM32_I2C4_FREQUENCY  STM32_HSI_FREQUENCY
#    elif STM32_RCC_DCKCFGR2_I2C4SRC == RCC_DCKCFGR2_I2C4SEL_SYSCLK
#      define STM32_I2C4_FREQUENCY  STM32_SYSCLK_FREQUENCY
#    elif STM32_RCC_DCKCFGR2_I2C4SRC == RCC_DCKCFGR2_I2C4SEL_APB
#      define STM32_I2C4_FREQUENCY  STM32_PCLK1_FREQUENCY
#    endif
#  endif

#elif defined(CONFIG_ARCH_CHIP_STM32H7)

/* Sources from STM32_RCC_D2CCIP2R_I2C123SRC and STM32_RCC_D3CCIPR_I2C4SRC
 * in board.h, else the reset value (PCLK1, PCLK4)
 */

#  if !defined(STM32_RCC_D2CCIP2R_I2C123SRC) || \
      STM32_RCC_D2CCIP2R_I2C123SRC == RCC_D2CCIP2R_I2C123SEL_PCLK1
#    define STM32_I2C123_KERCLK  STM32_PCLK1_FREQUENCY
#  elif STM32_RCC_D2CCIP2R_I2C123SRC == RCC_D2CCIP2R_I2C123SEL_HSI
#    define STM32_I2C123_KERCLK  STM32_HSI_FREQUENCY
#  endif

#  if !defined(STM32_I2C1_FREQUENCY) && defined(STM32_I2C123_KERCLK)
#    define STM32_I2C1_FREQUENCY  STM32_I2C123_KERCLK
#  endif

#  if !defined(STM32_I2C2_FREQUENCY) && defined(STM32_I2C123_KERCLK)
#    define STM32_I2C2_FREQUENCY  STM32_I2C123_KERCLK
#  endif

#  if !defined(STM32_I2C3_FREQUENCY) && defined(STM32_I2C123_KERCLK)
#    define STM32_I2C3_FREQUENCY  STM32_I2C123_KERCLK
#  endif

#  ifndef STM32_I2C4_FREQUENCY
#    if !defined(STM32_RCC_D3CCIPR_I2C4SRC) || \
        STM32_RCC_D3CCIPR_I2C4SRC == RCC_D3CCIPR_I2C4SEL_PCLK4
#      define STM32_I2C4_FREQUENCY  STM32_PCLK4_FREQUENCY
#    elif STM32_RCC_D3CCIPR_I2C4SRC == RCC_D3CCIPR_I2C4SEL_HSI
#      define STM32_I2C4_FREQUENCY  STM32_HSI_FREQUENCY
#    endif
#  endif

#endif

/* Interrupt names of the families that spell them STM32_IRQ_I2Cn_EV/ER */

#if !defined(STM32_IRQ_I2C1EV) && defined(STM32_IRQ_I2C1_EV)
#  define STM32_IRQ_I2C1EV         STM32_IRQ_I2C1_EV
#  define STM32_IRQ_I2C1ER         STM32_IRQ_I2C1_ER
#endif

#if !defined(STM32_IRQ_I2C2EV) && defined(STM32_IRQ_I2C2_EV)
#  define STM32_IRQ_I2C2EV         STM32_IRQ_I2C2_EV
#  define STM32_IRQ_I2C2ER         STM32_IRQ_I2C2_ER
#endif

#if !defined(STM32_IRQ_I2C3EV) && defined(STM32_IRQ_I2C3_EV)
#  define STM32_IRQ_I2C3EV         STM32_IRQ_I2C3_EV
#  define STM32_IRQ_I2C3ER         STM32_IRQ_I2C3_ER
#endif

#if !defined(STM32_IRQ_I2C4EV) && defined(STM32_IRQ_I2C4_EV)
#  define STM32_IRQ_I2C4EV         STM32_IRQ_I2C4_EV
#  define STM32_IRQ_I2C4ER         STM32_IRQ_I2C4_ER
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_I2C_CLK_H */
