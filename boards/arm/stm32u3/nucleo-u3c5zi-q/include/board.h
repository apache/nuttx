/****************************************************************************
 * boards/arm/stm32u3/nucleo-u3c5zi-q/include/board.h
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

#ifndef __BOARDS_ARM_STM32U3_NUCLEO_U3C5ZI_Q_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32U3_NUCLEO_U3C5ZI_Q_INCLUDE_BOARD_H

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

/* The NUCLEO-U3C5ZI-Q uses the 96 MHz MSIRC0 as its system clock.
 *
 *   System clock source : MSIS (MSIRC0)
 *   SYSCLK              : 96 MHz
 *   HCLK                : 96 MHz (SYSCLK / 1)
 *   PCLK1               : 96 MHz (HCLK / 1)
 *   PCLK2               : 96 MHz (HCLK / 1)
 *   PCLK3               : 96 MHz (HCLK / 1)
 *   Core supply         : SMPS, voltage scale 1, EPOD booster enabled
 *   FLASH latency       : 3 wait states
 */

#define STM32_BOARD_USEMSIS          1
#define STM32_RCC_ICSCR1_MSISSEL     RCC_ICSCR1_MSISSEL_MSIRC0
#define STM32_RCC_ICSCR1_MSISDIV     RCC_ICSCR1_MSISDIV_DIV1

#define STM32_RCC_CFGR4_BOOSTSEL     RCC_CFGR4_BOOSTSEL_MSIS
#define STM32_RCC_CFGR4_BOOSTDIV     RCC_CFGR4_BOOSTDIV_DIV1

#define STM32_PWR_CR3_REGSEL         PWR_CR3_REGSEL_SMPS
#define STM32_PWR_VOSR_RANGE         PWR_VOSR_R1EN
#define STM32_PWR_VOSR_RANGERDY      PWR_VOSR_R1RDY
#define STM32_PWR_VOSR_BOOST         PWR_VOSR_BOOSTEN

#define STM32_MSIS_FREQUENCY         96000000ul
#define STM32_SYSCLK_FREQUENCY       STM32_MSIS_FREQUENCY

#define STM32_RCC_CFGR2_HPRE         RCC_CFGR2_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY         STM32_SYSCLK_FREQUENCY

#define STM32_RCC_CFGR2_PPRE1        RCC_CFGR2_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY        STM32_HCLK_FREQUENCY

#define STM32_RCC_CFGR2_PPRE2        RCC_CFGR2_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY        STM32_HCLK_FREQUENCY

#define STM32_RCC_CFGR3_PPRE3        RCC_CFGR3_PPRE3_HCLK
#define STM32_PCLK3_FREQUENCY        STM32_HCLK_FREQUENCY

#define STM32_FLASH_ACR_LATENCY      FLASH_ACR_LATENCY_3

#define STM32_TIM1_CLKIN             STM32_PCLK2_FREQUENCY
#define STM32_TIM2_CLKIN             STM32_PCLK1_FREQUENCY
#define STM32_TIM3_CLKIN             STM32_PCLK1_FREQUENCY
#define STM32_TIM4_CLKIN             STM32_PCLK1_FREQUENCY
#define STM32_TIM6_CLKIN             STM32_PCLK1_FREQUENCY
#define STM32_TIM7_CLKIN             STM32_PCLK1_FREQUENCY
#define STM32_TIM8_CLKIN             STM32_PCLK2_FREQUENCY
#define STM32_TIM12_CLKIN            STM32_PCLK2_FREQUENCY
#define STM32_TIM15_CLKIN            STM32_PCLK2_FREQUENCY
#define STM32_TIM16_CLKIN            STM32_PCLK2_FREQUENCY
#define STM32_TIM17_CLKIN            STM32_PCLK2_FREQUENCY

/* USART1 is connected to the ST-LINK virtual COM port. */

#define GPIO_USART1_RX               GPIO_USART1_RX_1
#define GPIO_USART1_TX               GPIO_USART1_TX_1

/* User LEDs */

#define GPIO_LED_GREEN               (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                      GPIO_SPEED_2MHZ | GPIO_OUTPUT_CLEAR | \
                                      GPIO_PORTA | GPIO_PIN5)
#define GPIO_LED_RED                 (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                      GPIO_SPEED_2MHZ | GPIO_OUTPUT_CLEAR | \
                                      GPIO_PORTE | GPIO_PIN14)
#define GPIO_LED_BLUE                (GPIO_OUTPUT | GPIO_PUSHPULL | \
                                      GPIO_SPEED_2MHZ | GPIO_OUTPUT_CLEAR | \
                                      GPIO_PORTF | GPIO_PIN8)

#define BOARD_LED_GREEN              0
#define BOARD_LED_RED                1
#define BOARD_LED_BLUE               2
#define BOARD_NLEDS                  3

#define BOARD_LED_GREEN_BIT          (1 << BOARD_LED_GREEN)
#define BOARD_LED_RED_BIT            (1 << BOARD_LED_RED)
#define BOARD_LED_BLUE_BIT           (1 << BOARD_LED_BLUE)

/* User button B1 is active high. */

#define GPIO_BTN_USER                (GPIO_INPUT | GPIO_FLOAT | \
                                      GPIO_PORTC | GPIO_PIN13)
#define BUTTON_USER                  0
#define NUM_BUTTONS                  1
#define BUTTON_USER_BIT              (1 << BUTTON_USER)

/****************************************************************************
 * Public Function Prototypes
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

void stm32_board_initialize(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_ARM_STM32U3_NUCLEO_U3C5ZI_Q_INCLUDE_BOARD_H */
