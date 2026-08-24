/****************************************************************************
 * boards/arm/stm32c5/nucleo-c562re/include/board.h
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

#ifndef __BOARDS_ARM_STM32C5_NUCLEO_C562RE_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32C5_NUCLEO_C562RE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The NUCLEO-C562RE has a 24 MHz HSE crystal.
 *
 *   System clock source : PSI (HSE)
 *   SYSCLK              : 144 MHz
 *   HCLK                : 144 MHz (SYSCLK / 1)
 *   PCLK1               : 144 MHz (HCLK / 1)
 *   PCLK2               : 144 MHz (HCLK / 1)
 *   PCLK3               : 144 MHz (HCLK / 1)
 *   FLASH latency       : 4 wait states
 *   FLASH write delay   : 2
 */

#define STM32_BOARD_XTAL             24000000ul
#define STM32_HSE_FREQUENCY          STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY          32768ul
#define STM32_LSI_FREQUENCY          32000ul

#define STM32_BOARD_USEHSE           1
#define STM32_BOARD_USEPSI           1

#define STM32_RCC_CR2_PSIREFSRC      RCC_CR2_PSIREFSRC_HSE
#define STM32_RCC_CR2_PSIREF         RCC_CR2_PSIREF_24MHZ
#define STM32_RCC_CR2_PSIFREQ        RCC_CR2_PSIFREQ_144MHZ

#define STM32_PSI_FREQUENCY          144000000ul
#define STM32_SYSCLK_FREQUENCY       STM32_PSI_FREQUENCY

#define STM32_RCC_CFGR2_HPRE         RCC_CFGR2_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY         STM32_SYSCLK_FREQUENCY

#define STM32_RCC_CFGR2_PPRE1        RCC_CFGR2_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY        STM32_HCLK_FREQUENCY

#define STM32_RCC_CFGR2_PPRE2        RCC_CFGR2_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY        STM32_HCLK_FREQUENCY

#define STM32_RCC_CFGR2_PPRE3        RCC_CFGR2_PPRE3_HCLK
#define STM32_PCLK3_FREQUENCY        STM32_HCLK_FREQUENCY

#define STM32_FLASH_ACR_LATENCY      FLASH_ACR_LATENCY_4
#define STM32_FLASH_ACR_WRHIGHFREQ   FLASH_ACR_WRHIGHFREQ_2

#define STM32_TIM1_CLKIN             STM32_PCLK2_FREQUENCY
#define STM32_TIM2_CLKIN             STM32_PCLK1_FREQUENCY
#define STM32_TIM5_CLKIN             STM32_PCLK1_FREQUENCY
#define STM32_TIM6_CLKIN             STM32_PCLK1_FREQUENCY
#define STM32_TIM7_CLKIN             STM32_PCLK1_FREQUENCY
#define STM32_TIM8_CLKIN             STM32_PCLK2_FREQUENCY
#define STM32_TIM12_CLKIN            STM32_PCLK1_FREQUENCY
#define STM32_TIM15_CLKIN            STM32_PCLK2_FREQUENCY
#define STM32_TIM16_CLKIN            STM32_PCLK2_FREQUENCY
#define STM32_TIM17_CLKIN            STM32_PCLK2_FREQUENCY

/* USART2 is connected to the ST-LINK virtual COM port. */

#define GPIO_USART2_RX               GPIO_USART2_RX_1
#define GPIO_USART2_TX               GPIO_USART2_TX_1

/* User LEDs */

#define BOARD_LED_GREEN              0
#define BOARD_NLEDS                  1

#define BOARD_LED_GREEN_BIT          (1 << BOARD_LED_GREEN)

/* NuttX LED state mapping.  LD1 is active high. */

#define LED_STARTED                  0
#define LED_HEAPALLOCATE             0
#define LED_IRQSENABLED              0
#define LED_STACKCREATED             1
#define LED_INIRQ                    2
#define LED_SIGNAL                   2
#define LED_ASSERTION                2
#define LED_PANIC                    1
#define LED_IDLE                     0

/* User button B1 is active high. */

#define BUTTON_USER                  0
#define NUM_BUTTONS                  1
#define BUTTON_USER_BIT              (1 << BUTTON_USER)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
extern "C"
{
#endif

void stm32_board_initialize(void);

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32C5_NUCLEO_C562RE_INCLUDE_BOARD_H */
