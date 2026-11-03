/****************************************************************************
 * boards/arm/stm32g0/weact-stm32g0b1/include/board.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_WEACT_STM32G0B1RE_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F0L0G0_WEACT_STM32G0B1RE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 16 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - 8 MHz from MCO output of ST-LINK (disabled by default)
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000            /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768            /* X2 on board */

/* Main PLL Configuration.
 *
 * PLL source is HSE = 8,000,000
 *
 * PLL_VCOx = (STM32_HSE_FREQUENCY / PLLM) * PLLN
 * Subject to:
 *
 *     1 <= PLLM <= 8
 *     8 <= PLLN <= 86
 *   4 MHz <= PLL_IN <= 16MHz
 *   64 MHz <= PLL_VCO <= 344MHz
 * SYSCLK  = PLLRCLK = PLL_VCO / PLLR
 *
 */

/* PLL source is HSE, PLLN=16, PLLM=1
 * PLLP enable, PLLQ enable, PLLR enable
 *
 *   2 <= PLLP <= 32
 *   2 <= PLLQ <= 8
 *   2 <= PLLR <= 8
 *
 *   PLLR <= 64MHz
 *   PLLQ <= 64MHz
 *   PLLP <= 64MHz
 *
 *   PLL_VCO = (8,000,000 / 2) * 32 = 128 MHz
 *
 *   PLLP = PLL_VCO/2 = 128 MHz / 2 = 64 MHz
 *   PLLQ = PLL_VCO/2 = 128 MHz / 2 = 64 MHz
 *   PLLR = PLL_VCO/2 = 128 MHz / 2 = 64 MHz
 */

#define STM32_PLLCFG_PLLSRC     RCC_PLLCFG_PLLSRC_HSE
#define STM32_PLLCFG_PLLCFG     (RCC_PLLCFG_PLLPEN | \
                                 RCC_PLLCFG_PLLQEN | \
                                 RCC_PLLCFG_PLLREN)

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(1)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(16)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP(2)
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(2)
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)

#define STM32_VCO_FREQUENCY     ((STM32_HSE_FREQUENCY / 2) * 32)
#define STM32_PLLP_FREQUENCY    (STM32_VCO_FREQUENCY / 2)
#define STM32_PLLQ_FREQUENCY    (STM32_VCO_FREQUENCY / 2)
#define STM32_PLLR_FREQUENCY    (STM32_VCO_FREQUENCY / 2)

/* Use the PLL and set the SYSCLK source to be the PLLR (64MHz) */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  (STM32_PLLR_FREQUENCY)

/* AHB clock (HCLK) is SYSCLK (64MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 (32MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* Timers will receive PCLK1 * 2 */

#define STM32_TIM1_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM2_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM3_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM4_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM6_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM7_CLKIN   (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM14_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM15_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM16_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM17_CLKIN  (2 * STM32_PCLK1_FREQUENCY)

/* LED definitions **********************************************************/

/* The board has 1 user LED that could be used this diagnostic LED too.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.
 * The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1        0
#define BOARD_NLEDS       1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT   (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the WeAct-STM32G0B1. The following definitions describe how NuttX controls
 * the LED:
 *
 *   SYMBOL              Meaning                  LED1 state
 *   ------------------  -----------------------  ----------
 *   LED_STARTED         NuttX has been started   OFF
 *   LED_HEAPALLOCATE    Heap has been allocated  OFF
 *   LED_IRQSENABLED     Interrupts enabled       OFF
 *   LED_STACKCREATED    Idle stack created       ON
 *   LED_INIRQ           In an interrupt          No change
 *   LED_SIGNAL          In a signal handler      No change
 *   LED_ASSERTION       An assertion failed      No change
 *   LED_PANIC           The system has crashed   Blinking
 *   LED_IDLE            STM32 is in sleep mode   Not used
 */

#define LED_STARTED      0
#define LED_HEAPALLOCATE 0
#define LED_IRQSENABLED  0
#define LED_STACKCREATED 1
#define LED_INIRQ        2
#define LED_SIGNAL       2
#define LED_ASSERTION    2
#define LED_PANIC        1

/* Button definitions *******************************************************/

/* The WeAct-STM32G0B1 has one button.
 */

#define BUTTON_USER      0
#define NUM_BUTTONS      1

#define BUTTON_USER_BIT  (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* USART */

#define GPIO_USART3_RX (GPIO_USART3_RX_3|GPIO_SPEED_HIGH) /* PB9 */
#define GPIO_USART3_TX (GPIO_USART3_TX_3|GPIO_SPEED_HIGH) /* PB8 */

#endif /* __BOARDS_ARM_STM32F0L0G0_WEACT_STM32G0B1_INCLUDE_BOARD_H */
