/****************************************************************************
 * arch/arm/src/common/stm32/stm32_systick.h
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

#ifndef __ARCH_ARM_SRC_COMMON_STM32_STM32_SYSTICK_H
#define __ARCH_ARM_SRC_COMMON_STM32_STM32_SYSTICK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include <arch/board/board.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SysTick clock: the CPU clock (CLKSOURCE = 1) or the HCLK/8 external
 * reference (CLKSOURCE = 0).  The CPU clock differs from HCLK on STM32H7
 * and STM32N6, where the board must define STM32_CPUCLK_FREQUENCY.
 */

#if defined(CONFIG_STM32_SYSTICK_HCLK_DIV8)
#  define STM32_SYSTICK_CORECLK false
#  define STM32_SYSTICK_CLOCK   (STM32_HCLK_FREQUENCY / 8)
#elif defined(STM32_CPUCLK_FREQUENCY)
#  define STM32_SYSTICK_CORECLK true
#  define STM32_SYSTICK_CLOCK   STM32_CPUCLK_FREQUENCY
#elif defined(CONFIG_ARCH_CHIP_STM32H7) || defined(CONFIG_ARCH_CHIP_STM32N6)
#  error "board.h must define STM32_CPUCLK_FREQUENCY"
#else
#  define STM32_SYSTICK_CORECLK true
#  define STM32_SYSTICK_CLOCK   STM32_HCLK_FREQUENCY
#endif

#endif /* __ARCH_ARM_SRC_COMMON_STM32_STM32_SYSTICK_H */
