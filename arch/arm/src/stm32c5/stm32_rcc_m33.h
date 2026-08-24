/****************************************************************************
 * arch/arm/src/stm32c5/stm32_rcc_m33.h
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

#ifndef __ARCH_ARM_SRC_STM32C5_STM32_RCC_M33_H
#define __ARCH_ARM_SRC_STM32C5_STM32_RCC_M33_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/stm32_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USART clock and RCC definitions */

#define STM32_LPUART1_FREQUENCY  STM32_PCLK3_FREQUENCY
#define STM32_LPUART1_RCC_REG    STM32_RCC_APB3ENR
#define STM32_LPUART1_RCC_EN     RCC_APB3ENR_LPUART1EN

#define STM32_USART1_FREQUENCY   STM32_PCLK2_FREQUENCY
#define STM32_USART1_RCC_REG     STM32_RCC_APB2ENR
#define STM32_USART1_RCC_EN      RCC_APB2ENR_USART1EN

#define STM32_USART2_FREQUENCY   STM32_PCLK1_FREQUENCY
#define STM32_USART2_RCC_REG     STM32_RCC_APB1LENR
#define STM32_USART2_RCC_EN      RCC_APB1LENR_USART2EN

#define STM32_USART3_FREQUENCY   STM32_PCLK1_FREQUENCY
#define STM32_USART3_RCC_REG     STM32_RCC_APB1LENR
#define STM32_USART3_RCC_EN      RCC_APB1LENR_USART3EN

#define STM32_UART4_FREQUENCY    STM32_PCLK1_FREQUENCY
#define STM32_UART4_RCC_REG      STM32_RCC_APB1LENR
#define STM32_UART4_RCC_EN       RCC_APB1LENR_UART4EN

#define STM32_UART5_FREQUENCY    STM32_PCLK1_FREQUENCY
#define STM32_UART5_RCC_REG      STM32_RCC_APB1LENR
#define STM32_UART5_RCC_EN       RCC_APB1LENR_UART5EN

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

void stm32_clockconfig(void);

#ifdef CONFIG_ARCH_BOARD_STM32_CUSTOM_CLOCKCONFIG
void stm32_board_clockconfig(void);
#else
void stm32_stdclockconfig(void);
#endif

#ifdef CONFIG_PM
void stm32_clockenable(void);
#endif

void stm32_rcc_enableperipherals(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32C5_STM32_RCC_M33_H */
