/****************************************************************************
 * boards/arm/stm32g4/nucleo-g491re/include/board.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed to the Apache Software Foundation (ASF) under one or more
 *  contributor license agreements.  See the NOTICE file distributed with
 *  this work for additional information regarding copyright ownership.  The
 *  ASF licenses this file to you under the Apache License, Version 2.0 (the
 *  "License"); you may not use this file except in compliance with the
 *  License.  You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 *  License for the specific language governing permissions and limitations
 *  under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32G4_NUCLEO_G491RE_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32G4_NUCLEO_G491RE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#undef STM32_BOARD_XTAL                        /* Not installed */

#define STM32_HSI_FREQUENCY            16000000ul          /* 16MHz */
#define STM32_LSI_FREQUENCY            32000               /* 32kHz */
#undef STM32_HSE_FREQUENCY                     /* Not installed */
#undef STM32_LSE_FREQUENCY                     /* Not installed */

/* Main PLL Configuration.
 *
 * PLL source is HSI = 16MHz
 * PLLN = 85, PLLM = 4, PLLP = 10, PLLQ = 2, PLLR = 2
 *
 * f(VCO Clock) = f(PLL Clock Input) x (PLLN / PLLM)
 * f(PLL_P) = f(VCO Clock) / PLLP
 * f(PLL_Q) = f(VCO Clock) / PLLQ
 * f(PLL_R) = f(VCO Clock) / PLLR
 *
 * Where:
 * 8 <= PLLN <= 127
 * 1 <= PLLM <= 16
 * PLLP = 2 through 31
 * PLLQ = 2, 4, 6, or 8
 * PLLR = 2, 4, 6, or 8
 *
 * Do not exceed 170MHz on f(PLL_P), f(PLL_Q), or f(PLL_R).
 * 64MHz <= f(VCO Clock) <= 344MHz.
 *
 * Given the above:
 *
 * f(VCO Clock) = HSI   x PLLN / PLLM
 *              = 16MHz x 85   / 4
 *              = 340MHz
 *
 * PLLPCLK      = f(VCO Clock) / PLLP
 *              = 340MHz       / 10
 *              = 34MHz
 *                (May be used for ADC)
 *
 * PLLQCLK      = f(VCO Clock) / PLLQ
 *              = 340MHz       / 2
 *              = 170MHz
 *                (May be used for QUADSPI, FDCAN, SAI1, I2S3. If set to
 *                48MHz, may be used for USB, RNG.)
 *
 * PLLRCLK      = f(VCO Clock) / PLLR
 *              = 340MHz       / 2
 *              = 170MHz
 *                (May be used for SYSCLK and most peripherals.)
 */

#define STM32_PLLCFGR_PLLSRC           RCC_PLLCFGR_PLLSRC_HSI
#define STM32_PLLCFGR_PLLCFG           (RCC_PLLCFGR_PLLPEN | \
                                       RCC_PLLCFGR_PLLQEN | \
                                       RCC_PLLCFGR_PLLREN)

#define STM32_PLLCFGR_PLLN             RCC_PLLCFGR_PLLN(85)
#define STM32_PLLCFGR_PLLM             RCC_PLLCFGR_PLLM(4)
#define STM32_PLLCFGR_PLLP             RCC_PLLCFGR_PLLPDIV(10)
#define STM32_PLLCFGR_PLLQ             RCC_PLLCFGR_PLLQ_2
#define STM32_PLLCFGR_PLLR             RCC_PLLCFGR_PLLR_2

#define STM32_VCO_FREQUENCY            ((STM32_HSI_FREQUENCY / 4) * 85)
#define STM32_PLLP_FREQUENCY           (STM32_VCO_FREQUENCY / 10)
#define STM32_PLLQ_FREQUENCY           (STM32_VCO_FREQUENCY / 2)
#define STM32_PLLR_FREQUENCY           (STM32_VCO_FREQUENCY / 2)

/* Use the PLL and set the SYSCLK source to be PLLR (170MHz) */

#define STM32_SYSCLK_SW                RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS               RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY         STM32_PLLR_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (170MHz) */

#define STM32_RCC_CFGR_HPRE            RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY           STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK (170MHz) */

#define STM32_RCC_CFGR_PPRE1           RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY          STM32_HCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (170MHz) */

#define STM32_RCC_CFGR_PPRE2           RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY          STM32_HCLK_FREQUENCY

/* Timers driven from APB2 will be PCLK2 */

#define STM32_TIM1_CLKIN               STM32_PCLK2_FREQUENCY
#define STM32_TIM8_CLKIN               STM32_PCLK2_FREQUENCY
#define STM32_TIM15_CLKIN              STM32_PCLK2_FREQUENCY
#define STM32_TIM16_CLKIN              STM32_PCLK2_FREQUENCY
#define STM32_TIM17_CLKIN              STM32_PCLK2_FREQUENCY
#define STM32_TIM20_CLKIN              STM32_PCLK2_FREQUENCY

/* Timers driven from APB1 will be PCLK1 */

#define STM32_TIM2_CLKIN               STM32_PCLK1_FREQUENCY
#define STM32_TIM3_CLKIN               STM32_PCLK1_FREQUENCY
#define STM32_TIM4_CLKIN               STM32_PCLK1_FREQUENCY
#define STM32_TIM6_CLKIN               STM32_PCLK1_FREQUENCY
#define STM32_TIM7_CLKIN               STM32_PCLK1_FREQUENCY

/* LPTIM1 kernel clock is PCLK1 (RCC_CCIPR LPTIM1SEL reset value) */

#define STM32_LPTIM1_CLKIN             STM32_PCLK1_FREQUENCY

/* LPUART1 is connected to the ST-LINK virtual COM port. */

#define GPIO_LPUART1_TX                GPIO_LPUART1_TX_1   /* PA2 */
#define GPIO_LPUART1_RX                GPIO_LPUART1_RX_1   /* PA3 */

/* USART3 on CN7 pins 1 and 2. */

#define GPIO_USART3_TX                 GPIO_USART3_TX_3    /* PC10 */
#define GPIO_USART3_RX                 GPIO_USART3_RX_3    /* PC11 */

/* User LEDs */

#define BOARD_LED1                     0 /* User LD2 */
#define BOARD_NLEDS                    1

#define BOARD_LED1_BIT                 (1 << BOARD_LED1)

/* NuttX LED state mapping.  LD2 is active high. */

#define LED_STARTED                    0
#define LED_HEAPALLOCATE               0
#define LED_IRQSENABLED                0
#define LED_STACKCREATED               1
#define LED_INIRQ                      2
#define LED_SIGNAL                     2
#define LED_ASSERTION                  2
#define LED_PANIC                      1
#define LED_IDLE                       0

/* User button B1 is active high. */

#define BUTTON_USER                    0
#define NUM_BUTTONS                    1

#define BUTTON_USER_BIT                (1 << BUTTON_USER)

#endif /* __BOARDS_ARM_STM32G4_NUCLEO_G491RE_INCLUDE_BOARD_H */
