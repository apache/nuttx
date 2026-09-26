/****************************************************************************
 * boards/arm/stm32h5/stm32h573i-dk/include/board.h
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

#ifndef __BOARDS_ARM_STM32H5_STM32H573I_DK_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H5_STM32H573I_DK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The board supplies a 25 MHz oscillator to HSE (bypass mode) and a
 * 32.768 kHz crystal to LSE.  PLL1 produces the 250 MHz system clock:
 * 25 MHz / 5 * 100 / 2 = 250 MHz.
 */

#define STM32_BOARD_USEHSE
#define STM32_HSEBYP_ENABLE
#define STM32_HSEEXT_ENABLE
#define STM32_HSE_FREQUENCY       25000000ul
#define STM32_HSI_FREQUENCY       64000000ul
#define STM32_CSI_FREQUENCY       4000000ul
#define STM32_LSI_FREQUENCY       32000
#define STM32_LSE_FREQUENCY       32768

#define STM32_PLLCFG_PLL1CFG      (RCC_PLL1CFGR_PLL1SRC_HSE | \
                                  RCC_PLL1CFGR_PLL1RGE_4_8M | \
                                  RCC_PLL1CFGR_PLL1M(5) | \
                                  RCC_PLL1CFGR_PLL1PEN | \
                                  RCC_PLL1CFGR_PLL1QEN | \
                                  RCC_PLL1CFGR_PLL1REN)
#define STM32_PLLCFG_PLL1N        RCC_PLL1DIVR_PLL1N(100)
#define STM32_PLLCFG_PLL1P        RCC_PLL1DIVR_PLL1P(2)
#define STM32_PLLCFG_PLL1Q        RCC_PLL1DIVR_PLL1Q(4)
#define STM32_PLLCFG_PLL1R        RCC_PLL1DIVR_PLL1R(2)
#define STM32_PLLCFG_PLL1DIVR     (STM32_PLLCFG_PLL1N | \
                                  STM32_PLLCFG_PLL1P | \
                                  STM32_PLLCFG_PLL1Q | \
                                  STM32_PLLCFG_PLL1R)

#define STM32_VCO1_FREQUENCY      ((STM32_HSE_FREQUENCY / 5) * 100)
#define STM32_PLL1P_FREQUENCY     (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1Q_FREQUENCY     (STM32_VCO1_FREQUENCY / 4)
#define STM32_PLL1R_FREQUENCY     (STM32_VCO1_FREQUENCY / 2)
#define STM32_SYSCLK_FREQUENCY    STM32_PLL1P_FREQUENCY

/* AHB and all APB buses run at 250 MHz. */

#define STM32_RCC_CFGR2_HPRE      RCC_CFGR2_HPRE_SYSCLK
#define STM32_RCC_CFGR2_PPRE1     RCC_CFGR2_PPRE1_HCLK1
#define STM32_RCC_CFGR2_PPRE2     RCC_CFGR2_PPRE2_HCLK1
#define STM32_RCC_CFGR2_PPRE3     RCC_CFGR2_PPRE3_HCLK1
#define STM32_HCLK_FREQUENCY      STM32_SYSCLK_FREQUENCY
#define STM32_PCLK1_FREQUENCY     STM32_HCLK_FREQUENCY
#define STM32_PCLK2_FREQUENCY     STM32_HCLK_FREQUENCY
#define STM32_PCLK3_FREQUENCY     STM32_HCLK_FREQUENCY

/* Timer input clocks with APB prescalers set to one. */

#define STM32_TIM1_CLKIN          STM32_PCLK2_FREQUENCY
#define STM32_TIM2_CLKIN          STM32_PCLK1_FREQUENCY
#define STM32_TIM3_CLKIN          STM32_PCLK1_FREQUENCY
#define STM32_TIM4_CLKIN          STM32_PCLK1_FREQUENCY
#define STM32_TIM5_CLKIN          STM32_PCLK1_FREQUENCY
#define STM32_TIM6_CLKIN          STM32_PCLK1_FREQUENCY
#define STM32_TIM7_CLKIN          STM32_PCLK1_FREQUENCY
#define STM32_TIM8_CLKIN          STM32_PCLK2_FREQUENCY
#define STM32_TIM12_CLKIN         STM32_PCLK1_FREQUENCY
#define STM32_TIM13_CLKIN         STM32_PCLK1_FREQUENCY
#define STM32_TIM14_CLKIN         STM32_PCLK1_FREQUENCY
#define STM32_TIM15_CLKIN         STM32_PCLK2_FREQUENCY
#define STM32_TIM16_CLKIN         STM32_PCLK2_FREQUENCY
#define STM32_TIM17_CLKIN         STM32_PCLK2_FREQUENCY
#define STM32_LPTIM1_CLKIN        STM32_PCLK3_FREQUENCY
#define STM32_LPTIM2_CLKIN        STM32_PCLK1_FREQUENCY
#define STM32_LPTIM3_CLKIN        STM32_PCLK3_FREQUENCY
#define STM32_LPTIM4_CLKIN        STM32_PCLK3_FREQUENCY
#define STM32_LPTIM5_CLKIN        STM32_PCLK3_FREQUENCY
#define STM32_LPTIM6_CLKIN        STM32_PCLK3_FREQUENCY

/* USART1 is connected to the ST-LINK virtual COM port. */

#define GPIO_USART1_TX           GPIO_USART1_TX_1  /* PA9 */
#define GPIO_USART1_RX           GPIO_USART1_RX_1  /* PA10 */

/* LED definitions **********************************************************/

/* Four active-low LEDs: LD1 green, LD2 orange, LD3 red, LD4 blue. */

#define BOARD_LED1               0
#define BOARD_LED2               1
#define BOARD_LED3               2
#define BOARD_LED4               3
#define BOARD_NLEDS              4

#define BOARD_LED_GREEN          BOARD_LED1
#define BOARD_LED_ORANGE         BOARD_LED2
#define BOARD_LED_RED            BOARD_LED3
#define BOARD_LED_BLUE           BOARD_LED4

#define BOARD_LED1_BIT           (1 << BOARD_LED1)
#define BOARD_LED2_BIT           (1 << BOARD_LED2)
#define BOARD_LED3_BIT           (1 << BOARD_LED3)
#define BOARD_LED4_BIT           (1 << BOARD_LED4)

/* With CONFIG_ARCH_LEDS, green indicates successful boot and red blinks
 * on a panic.  Interrupts, signals and assertions leave the LEDs unchanged.
 */

#define LED_STARTED              0
#define LED_HEAPALLOCATE         0
#define LED_IRQSENABLED          0
#define LED_STACKCREATED         1
#define LED_INIRQ                2
#define LED_SIGNAL               2
#define LED_ASSERTION            2
#define LED_PANIC                3

/* Button definitions *******************************************************/

/* B1 USER is connected to PC13 and reads high when pressed. */

#define BUTTON_USER              0
#define NUM_BUTTONS              1
#define BUTTON_USER_BIT          (1 << BUTTON_USER)

#endif /* __BOARDS_ARM_STM32H5_STM32H573I_DK_INCLUDE_BOARD_H */
