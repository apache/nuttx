/****************************************************************************
 * arch/arm/src/common/stm32/stm32_tim_clk.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_TIM_CLK_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_TIM_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "stm32.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RCC clock-enable and reset aliases for the common timer drivers.
 *
 * TIM1, TIM8, TIM9 and TIM15-17 are on APB2, TIM2-7 and TIM13 are on
 * APB1.  TIM10-12 and TIM14 are on APB2 or APB1 depending on the family,
 * which is detected from the RCC bit definitions.  The APB1 register
 * names differ between families and are selected from the RCC registers
 * the family defines.
 * The timer input clock is provided by each board as STM32_TIMn_CLKIN.
 */

#if defined(STM32_RCC_APB1LENR)
#  define STM32_RCC_TIM_APB1EN_REG   STM32_RCC_APB1LENR
#  define STM32_RCC_TIM_APB1RST_REG  STM32_RCC_APB1LRSTR
#  define STM32_RCC_TIM_APB1EN(n)    RCC_APB1LENR_TIM##n##EN
#  define STM32_RCC_TIM_APB1RST(n)   RCC_APB1LRSTR_TIM##n##RST
#elif defined(STM32_RCC_APB1ENR1)
#  define STM32_RCC_TIM_APB1EN_REG   STM32_RCC_APB1ENR1
#  define STM32_RCC_TIM_APB1RST_REG  STM32_RCC_APB1RSTR1
#  define STM32_RCC_TIM_APB1EN(n)    RCC_APB1ENR1_TIM##n##EN
#  define STM32_RCC_TIM_APB1RST(n)   RCC_APB1RSTR1_TIM##n##RST
#else
#  define STM32_RCC_TIM_APB1EN_REG   STM32_RCC_APB1ENR
#  define STM32_RCC_TIM_APB1RST_REG  STM32_RCC_APB1RSTR
#  define STM32_RCC_TIM_APB1EN(n)    RCC_APB1ENR_TIM##n##EN
#  define STM32_RCC_TIM_APB1RST(n)   RCC_APB1RSTR_TIM##n##RST
#endif

#define STM32_RCC_TIM1_EN_REG   STM32_RCC_APB2ENR
#define STM32_RCC_TIM1_EN       RCC_APB2ENR_TIM1EN
#define STM32_RCC_TIM1_RST_REG  STM32_RCC_APB2RSTR
#define STM32_RCC_TIM1_RST      RCC_APB2RSTR_TIM1RST
#define STM32_RCC_TIM8_EN_REG   STM32_RCC_APB2ENR
#define STM32_RCC_TIM8_EN       RCC_APB2ENR_TIM8EN
#define STM32_RCC_TIM8_RST_REG  STM32_RCC_APB2RSTR
#define STM32_RCC_TIM8_RST      RCC_APB2RSTR_TIM8RST
#define STM32_RCC_TIM9_EN_REG   STM32_RCC_APB2ENR
#define STM32_RCC_TIM9_EN       RCC_APB2ENR_TIM9EN
#define STM32_RCC_TIM9_RST_REG  STM32_RCC_APB2RSTR
#define STM32_RCC_TIM9_RST      RCC_APB2RSTR_TIM9RST
#define STM32_RCC_TIM15_EN_REG  STM32_RCC_APB2ENR
#define STM32_RCC_TIM15_EN      RCC_APB2ENR_TIM15EN
#define STM32_RCC_TIM15_RST_REG STM32_RCC_APB2RSTR
#define STM32_RCC_TIM15_RST     RCC_APB2RSTR_TIM15RST
#define STM32_RCC_TIM16_EN_REG  STM32_RCC_APB2ENR
#define STM32_RCC_TIM16_EN      RCC_APB2ENR_TIM16EN
#define STM32_RCC_TIM16_RST_REG STM32_RCC_APB2RSTR
#define STM32_RCC_TIM16_RST     RCC_APB2RSTR_TIM16RST
#define STM32_RCC_TIM17_EN_REG  STM32_RCC_APB2ENR
#define STM32_RCC_TIM17_EN      RCC_APB2ENR_TIM17EN
#define STM32_RCC_TIM17_RST_REG STM32_RCC_APB2RSTR
#define STM32_RCC_TIM17_RST     RCC_APB2RSTR_TIM17RST

#define STM32_RCC_TIM2_EN_REG   STM32_RCC_TIM_APB1EN_REG
#define STM32_RCC_TIM2_EN       STM32_RCC_TIM_APB1EN(2)
#define STM32_RCC_TIM2_RST_REG  STM32_RCC_TIM_APB1RST_REG
#define STM32_RCC_TIM2_RST      STM32_RCC_TIM_APB1RST(2)
#define STM32_RCC_TIM3_EN_REG   STM32_RCC_TIM_APB1EN_REG
#define STM32_RCC_TIM3_EN       STM32_RCC_TIM_APB1EN(3)
#define STM32_RCC_TIM3_RST_REG  STM32_RCC_TIM_APB1RST_REG
#define STM32_RCC_TIM3_RST      STM32_RCC_TIM_APB1RST(3)
#define STM32_RCC_TIM4_EN_REG   STM32_RCC_TIM_APB1EN_REG
#define STM32_RCC_TIM4_EN       STM32_RCC_TIM_APB1EN(4)
#define STM32_RCC_TIM4_RST_REG  STM32_RCC_TIM_APB1RST_REG
#define STM32_RCC_TIM4_RST      STM32_RCC_TIM_APB1RST(4)
#define STM32_RCC_TIM5_EN_REG   STM32_RCC_TIM_APB1EN_REG
#define STM32_RCC_TIM5_EN       STM32_RCC_TIM_APB1EN(5)
#define STM32_RCC_TIM5_RST_REG  STM32_RCC_TIM_APB1RST_REG
#define STM32_RCC_TIM5_RST      STM32_RCC_TIM_APB1RST(5)
#define STM32_RCC_TIM6_EN_REG   STM32_RCC_TIM_APB1EN_REG
#define STM32_RCC_TIM6_EN       STM32_RCC_TIM_APB1EN(6)
#define STM32_RCC_TIM6_RST_REG  STM32_RCC_TIM_APB1RST_REG
#define STM32_RCC_TIM6_RST      STM32_RCC_TIM_APB1RST(6)
#define STM32_RCC_TIM7_EN_REG   STM32_RCC_TIM_APB1EN_REG
#define STM32_RCC_TIM7_EN       STM32_RCC_TIM_APB1EN(7)
#define STM32_RCC_TIM7_RST_REG  STM32_RCC_TIM_APB1RST_REG
#define STM32_RCC_TIM7_RST      STM32_RCC_TIM_APB1RST(7)
#define STM32_RCC_TIM13_EN_REG  STM32_RCC_TIM_APB1EN_REG
#define STM32_RCC_TIM13_EN      STM32_RCC_TIM_APB1EN(13)
#define STM32_RCC_TIM13_RST_REG STM32_RCC_TIM_APB1RST_REG
#define STM32_RCC_TIM13_RST     STM32_RCC_TIM_APB1RST(13)

#ifdef RCC_APB2ENR_TIM10EN
#  define STM32_RCC_TIM10_EN_REG  STM32_RCC_APB2ENR
#  define STM32_RCC_TIM10_EN      RCC_APB2ENR_TIM10EN
#  define STM32_RCC_TIM10_RST_REG STM32_RCC_APB2RSTR
#  define STM32_RCC_TIM10_RST     RCC_APB2RSTR_TIM10RST
#else
#  define STM32_RCC_TIM10_EN_REG  STM32_RCC_TIM_APB1EN_REG
#  define STM32_RCC_TIM10_EN      STM32_RCC_TIM_APB1EN(10)
#  define STM32_RCC_TIM10_RST_REG STM32_RCC_TIM_APB1RST_REG
#  define STM32_RCC_TIM10_RST     STM32_RCC_TIM_APB1RST(10)
#endif

#ifdef RCC_APB2ENR_TIM11EN
#  define STM32_RCC_TIM11_EN_REG  STM32_RCC_APB2ENR
#  define STM32_RCC_TIM11_EN      RCC_APB2ENR_TIM11EN
#  define STM32_RCC_TIM11_RST_REG STM32_RCC_APB2RSTR
#  define STM32_RCC_TIM11_RST     RCC_APB2RSTR_TIM11RST
#else
#  define STM32_RCC_TIM11_EN_REG  STM32_RCC_TIM_APB1EN_REG
#  define STM32_RCC_TIM11_EN      STM32_RCC_TIM_APB1EN(11)
#  define STM32_RCC_TIM11_RST_REG STM32_RCC_TIM_APB1RST_REG
#  define STM32_RCC_TIM11_RST     STM32_RCC_TIM_APB1RST(11)
#endif

#ifdef RCC_APB2ENR_TIM12EN
#  define STM32_RCC_TIM12_EN_REG  STM32_RCC_APB2ENR
#  define STM32_RCC_TIM12_EN      RCC_APB2ENR_TIM12EN
#  define STM32_RCC_TIM12_RST_REG STM32_RCC_APB2RSTR
#  define STM32_RCC_TIM12_RST     RCC_APB2RSTR_TIM12RST
#else
#  define STM32_RCC_TIM12_EN_REG  STM32_RCC_TIM_APB1EN_REG
#  define STM32_RCC_TIM12_EN      STM32_RCC_TIM_APB1EN(12)
#  define STM32_RCC_TIM12_RST_REG STM32_RCC_TIM_APB1RST_REG
#  define STM32_RCC_TIM12_RST     STM32_RCC_TIM_APB1RST(12)
#endif

#ifdef RCC_APB2ENR_TIM14EN
#  define STM32_RCC_TIM14_EN_REG  STM32_RCC_APB2ENR
#  define STM32_RCC_TIM14_EN      RCC_APB2ENR_TIM14EN
#  define STM32_RCC_TIM14_RST_REG STM32_RCC_APB2RSTR
#  define STM32_RCC_TIM14_RST     RCC_APB2RSTR_TIM14RST
#else
#  define STM32_RCC_TIM14_EN_REG  STM32_RCC_TIM_APB1EN_REG
#  define STM32_RCC_TIM14_EN      STM32_RCC_TIM_APB1EN(14)
#  define STM32_RCC_TIM14_RST_REG STM32_RCC_TIM_APB1RST_REG
#  define STM32_RCC_TIM14_RST     STM32_RCC_TIM_APB1RST(14)
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_TIM_CLK_H */
