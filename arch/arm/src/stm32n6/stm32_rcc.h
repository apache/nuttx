/****************************************************************************
 * arch/arm/src/stm32n6/stm32_rcc.h
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

#ifndef __ARCH_ARM_SRC_STM32N6_STM32_RCC_H
#define __ARCH_ARM_SRC_STM32N6_STM32_RCC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "arm_internal.h"
#include "chip.h"

#include "hardware/stm32n6xxx_rcc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* U[S]ART clock enables used by the common U[S]ART driver */

#define STM32_LPUART1_RCC_REG     STM32_RCC_APB4LENR
#define STM32_LPUART1_RCC_EN      RCC_APB4LENR_LPUART1EN
#define STM32_USART1_RCC_REG      STM32_RCC_APB2ENR
#define STM32_USART1_RCC_EN       RCC_APB2ENR_USART1EN
#define STM32_USART2_RCC_REG      STM32_RCC_APB1LENR
#define STM32_USART2_RCC_EN       RCC_APB1LENR_USART2EN
#define STM32_USART3_RCC_REG      STM32_RCC_APB1LENR
#define STM32_USART3_RCC_EN       RCC_APB1LENR_USART3EN
#define STM32_UART4_RCC_REG       STM32_RCC_APB1LENR
#define STM32_UART4_RCC_EN        RCC_APB1LENR_UART4EN
#define STM32_UART5_RCC_REG       STM32_RCC_APB1LENR
#define STM32_UART5_RCC_EN        RCC_APB1LENR_UART5EN
#define STM32_USART6_RCC_REG      STM32_RCC_APB2ENR
#define STM32_USART6_RCC_EN       RCC_APB2ENR_USART6EN
#define STM32_UART7_RCC_REG       STM32_RCC_APB1LENR
#define STM32_UART7_RCC_EN        RCC_APB1LENR_UART7EN
#define STM32_UART8_RCC_REG       STM32_RCC_APB1LENR
#define STM32_UART8_RCC_EN        RCC_APB1LENR_UART8EN
#define STM32_UART9_RCC_REG       STM32_RCC_APB2ENR
#define STM32_UART9_RCC_EN        RCC_APB2ENR_UART9EN
#define STM32_USART10_RCC_REG     STM32_RCC_APB2ENR
#define STM32_USART10_RCC_EN      RCC_APB2ENR_USART10EN

#ifndef __ASSEMBLY__

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_clockconfig
 *
 * Description:
 *   Called to establish the clock settings based on the values in board.h.
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
 * Name: stm32_stdclockconfig
 *
 * Description:
 *   The standard logic to configure the clocks based on settings in board.h.
 *   This function is chip type specific and implemented in stm32n6xx_rcc.c.
 *
 ****************************************************************************/

void stm32_stdclockconfig(void);

/****************************************************************************
 * Name: stm32_clockenable
 *
 * Description:
 *   Re-enable the clock and restore the clock settings based on settings in
 *   board.h.  This function is only available to support low-power modes of
 *   operation:  When re-awakening from deep-sleep modes, it is necessary to
 *   re-enable/re-start the PLL
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
 * Name: stm32_rcc_enableperipherals
 *
 * Description:
 *   Enable all the chip peripherals according to configuration.  This is
 *   chip type specific and thus implemented in stm32n6xx_rcc.c.
 *
 ****************************************************************************/

void stm32_rcc_enableperipherals(void);

/****************************************************************************
 * Name: stm32_rcc_enablelse
 *
 * Description:
 *   Enable the External Low-Speed (LSE) oscillator.
 *
 ****************************************************************************/

void stm32_rcc_enablelse(void);

/****************************************************************************
 * Name: stm32_rcc_enablelsi
 *
 * Description:
 *   Enable the Internal Low-Speed (LSI) oscillator.
 *
 ****************************************************************************/

void stm32_rcc_enablelsi(void);

#if defined(__cplusplus)
}
#endif
#endif /* __ASSEMBLY__ */
#endif /* __ARCH_ARM_SRC_STM32N6_STM32_RCC_H */
