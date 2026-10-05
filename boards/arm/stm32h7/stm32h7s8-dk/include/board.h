/****************************************************************************
 * boards/arm/stm32h7/stm32h7s8-dk/include/board.h
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

#ifndef __BOARDS_ARM_STM32H7_STM32H7S8_DK_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32H7_STM32H7S8_DK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Do not include STM32 H7 header files here */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The STM32H7S8-DK board provides the following clock sources:
 *
 *   MCO: 24 MHz from the ST-LINK is available as the HSE input
 *   X2:  32.768 kHz crystal for LSE
 *
 * HSI is selected by default so that the board can run when the ST-LINK is
 * not supplying its MCO clock.  CONFIG_STM32H7S8_DK_USE_HSE selects the
 * 24 MHz bypass clock instead.
 */

#define STM32_BOARD_XTAL         24000000ul
#define STM32_HSI_FREQUENCY      64000000ul
#define STM32_HSE_FREQUENCY      STM32_BOARD_XTAL
#define STM32_LSI_FREQUENCY      32000ul
#define STM32_LSE_FREQUENCY      32768ul

/* Main PLL configuration
 *
 * The HSI and HSE configurations both produce a 600 MHz PLL1 VCO and use
 * PLL1P as the system clock:
 *
 *   HSI: PLL1_VCO = (64 MHz / 32) * 300 = 600 MHz
 *   HSE: PLL1_VCO = (24 MHz / 2)  * 50  = 600 MHz
 *
 *   PLL1P = 600 MHz
 *   PLL1Q = 300 MHz
 *   PLL1R = 300 MHz
 */

#ifdef CONFIG_STM32H7S8_DK_USE_HSE
#  define STM32_BOARD_USEHSE
#  define STM32_HSEBYP_ENABLE
#  define STM32_PLLCFG_PLLSRC    RCC_PLLCKSELR_PLLSRC_HSE
#  define STM32_PLLCFG_PLL1M     RCC_PLLCKSELR_DIVM1(2)
#  define STM32_PLLCFG_PLL1N     RCC_PLL1DIVR1_N(50)
#  define STM32_PLLCFG_PLL1CFG   (RCC_PLLCFGR_PLL1RGE_8_16 | \
                                  RCC_PLLCFGR_PLL1PEN | \
                                  RCC_PLLCFGR_PLL1QEN | \
                                  RCC_PLLCFGR_PLL1REN)
#else
#  define STM32_BOARD_USEHSI
#  define STM32_PLLCFG_PLLSRC    RCC_PLLCKSELR_PLLSRC_HSI
#  define STM32_PLLCFG_PLL1M     RCC_PLLCKSELR_DIVM1(32)
#  define STM32_PLLCFG_PLL1N     RCC_PLL1DIVR1_N(300)
#  define STM32_PLLCFG_PLL1CFG   (RCC_PLLCFGR_PLL1RGE_2_4 | \
                                  RCC_PLLCFGR_PLL1PEN | \
                                  RCC_PLLCFGR_PLL1QEN | \
                                  RCC_PLLCFGR_PLL1REN)
#endif

#define STM32_PLLCFG_PLL1P       RCC_PLL1DIVR1_P(1)
#define STM32_PLLCFG_PLL1Q       RCC_PLL1DIVR1_Q(2)
#define STM32_PLLCFG_PLL1R       RCC_PLL1DIVR1_R(2)

#define STM32_VCO1_FREQUENCY     600000000ul
#define STM32_PLL1P_FREQUENCY    STM32_VCO1_FREQUENCY
#define STM32_PLL1Q_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)
#define STM32_PLL1R_FREQUENCY    (STM32_VCO1_FREQUENCY / 2)

/* SYSCLK = PLL1P = 600 MHz
 * CPUCLK = SYSCLK
 */

#define STM32_RCC_CDCFGR_CPRE    RCC_CDCFGR_CPRE_SYSCLK
#define STM32_SYSCLK_FREQUENCY   STM32_PLL1P_FREQUENCY
#define STM32_CPUCLK_FREQUENCY   STM32_SYSCLK_FREQUENCY

/* AHB clock (HCLK) is SYSCLK/2 (300 MHz). */

#define STM32_RCC_BMCFGR_HPRE    RCC_BMCFGR_HPRE_SYSCLKd2
#define STM32_HCLK_FREQUENCY     (STM32_SYSCLK_FREQUENCY / 2)

/* APB1, APB2, APB4 and APB5 clocks are HCLK/2 (150 MHz). */

#define STM32_RCC_APBCFGR_PPRE1  RCC_APBCFGR_PPRE1_HCLKd2
#define STM32_RCC_APBCFGR_PPRE2  RCC_APBCFGR_PPRE2_HCLKd2
#define STM32_RCC_APBCFGR_PPRE4  RCC_APBCFGR_PPRE4_HCLKd2
#define STM32_RCC_APBCFGR_PPRE5  RCC_APBCFGR_PPRE5_HCLKd2
#define STM32_PCLK1_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define STM32_PCLK2_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define STM32_PCLK4_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)
#define STM32_PCLK5_FREQUENCY    (STM32_HCLK_FREQUENCY / 2)

/* Timers on APB1 and APB2 are clocked at twice the peripheral clock. */

#define STM32_TIM2_CLKIN         (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM3_CLKIN         (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM4_CLKIN         (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM5_CLKIN         (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM6_CLKIN         (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM7_CLKIN         (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM12_CLKIN        (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM13_CLKIN        (2 * STM32_PCLK1_FREQUENCY)
#define STM32_TIM14_CLKIN        (2 * STM32_PCLK1_FREQUENCY)

#define STM32_TIM1_CLKIN         (2 * STM32_PCLK2_FREQUENCY)
#define STM32_TIM8_CLKIN         (2 * STM32_PCLK2_FREQUENCY)
#define STM32_TIM15_CLKIN        (2 * STM32_PCLK2_FREQUENCY)
#define STM32_TIM16_CLKIN        (2 * STM32_PCLK2_FREQUENCY)
#define STM32_TIM17_CLKIN        (2 * STM32_PCLK2_FREQUENCY)

/* FLASH wait states */

#define BOARD_FLASH_WAITSTATES   7

/* LED definitions **********************************************************/

/* The STM32H7S8-DK has three user-controllable LEDs:
 *
 *   LD1: Green
 *   LD2: Yellow
 *   LD3: Red
 *   LD4: Blue
 *
 * If CONFIG_ARCH_LEDS is defined, NuttX controls the LEDs as follows:
 *
 *   SYMBOL                Meaning                     LED state
 *   LED_STARTED           NuttX has been started      All LEDs off
 *   LED_HEAPALLOCATE      Heap has been allocated     No change
 *   LED_IRQSENABLED       Interrupts enabled          No change
 *   LED_STACKCREATED      Idle stack created          Green on
 *   LED_INIRQ             In an interrupt             No change
 *   LED_SIGNAL            In a signal handler         No change
 *   LED_ASSERTION         An assertion failed         Red on
 *   LED_PANIC             The system has crashed      Red blinking
 */

#define BOARD_LED1              0
#define BOARD_LED2              1
#define BOARD_LED3              2
#define BOARD_LED4              3
#define BOARD_NLEDS             4
#define BOARD_LED_GREEN         BOARD_LED1
#define BOARD_LED_YELLOW        BOARD_LED2
#define BOARD_LED_RED           BOARD_LED3
#define BOARD_LED_BLUE          BOARD_LED4

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT          (1 << BOARD_LED1)
#define BOARD_LED2_BIT          (1 << BOARD_LED2)
#define BOARD_LED3_BIT          (1 << BOARD_LED3)
#define BOARD_LED4_BIT          (1 << BOARD_LED4)

#define LED_STARTED             0
#define LED_HEAPALLOCATE        1
#define LED_IRQSENABLED         2
#define LED_STACKCREATED        3
#define LED_INIRQ               4
#define LED_SIGNAL              5
#define LED_ASSERTION           6
#define LED_PANIC               7
#define LED_IDLE                8

/* Button definitions *******************************************************/

/* The blue B2 user button is connected to PC13 and is active low.  The black
 * B1 button drives NRST and is not a GPIO button.
 */

#define BUTTON_USER             0
#define NUM_BUTTONS             1
#define BUTTON_USER_BIT         (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* UART4: ST-LINK virtual COM port on PD1 and PD0. */

#define GPIO_UART4_TX           (GPIO_UART4_TX_5 | GPIO_SPEED_100MHz) /* PD1 */
#define GPIO_UART4_RX           (GPIO_UART4_RX_5 | GPIO_SPEED_100MHz) /* PD0 */

#endif /* __BOARDS_ARM_STM32H7_STM32H7S8_DK_INCLUDE_BOARD_H */
