/****************************************************************************
 * boards/arm/stm32l0/npzero-g1s-dk/include/board.h
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

#ifndef __BOARDS_ARM_STM32L0_NPZERO_G1S_DK_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32L0_NPZERO_G1S_DK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The STM32 MCU host of the nPZero G1S Development Kit has no crystal:
 *
 * HSI - Internal 16 MHz RC Oscillator
 * LSI - 37 KHz RC
 */

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     37000            /* Between 26kHz and 56kHz */
#define STM32_HSE_FREQUENCY     0
#define STM32_LSE_FREQUENCY     0

/* The PLL is not used */

#define STM32_CFGR_PLLSRC       0
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       0
#define STM32_CFGR_PLLDIV       0

/* SYSCLK is HSI (16MHz).  It keeps the consumption low and it is the
 * clock expected by the I2C driver for its timings.
 */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_HSI
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_HSI
#define STM32_SYSCLK_FREQUENCY  STM32_HSI_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (16MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (16MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK (16MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY   STM32_HCLK_FREQUENCY

/* APB1 timers 2, 3, 6 and 7 will receive PCLK1 */

#define STM32_TIM2_CLKIN        (STM32_PCLK1_FREQUENCY)
#define STM32_TIM3_CLKIN        (STM32_PCLK1_FREQUENCY)
#define STM32_TIM6_CLKIN        (STM32_PCLK1_FREQUENCY)
#define STM32_TIM7_CLKIN        (STM32_PCLK1_FREQUENCY)

/* LED definitions **********************************************************/

/* The STM32 MCU host has one LED, connected to PA12 (high to light it).
 * The LEDs D1-D6 of the DevBoard show the state of the power switches of
 * the nPZero and they are not controlled by the MCU.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1       0
#define BOARD_NLEDS      1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT   (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDS is defined, then NuttX will control the LED.  The
 * following definitions describe how NuttX controls the LED:
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

/* Alternate function pin selections ****************************************/

/* USART2 is the console, on the MCU UART connector J19 of the DevBoard:
 *   USART2_RX - PA3
 *   USART2_TX - PA2
 */

#define GPIO_USART2_RX   (GPIO_USART2_RX_1|GPIO_SPEED_HIGH)   /* PA3 */
#define GPIO_USART2_TX   (GPIO_USART2_TX_1|GPIO_SPEED_HIGH)   /* PA2 */

/* I2C1 is connected to the nPZero G1S (host I2C of the DevBoard):
 *   I2C1_SCL - PB6
 *   I2C1_SDA - PB7
 */

#define GPIO_I2C1_SCL    (GPIO_I2C1_SCL_1|GPIO_PULLUP|GPIO_SPEED_LOW) /* PB6 */
#define GPIO_I2C1_SDA    (GPIO_I2C1_SDA_1|GPIO_PULLUP|GPIO_SPEED_LOW) /* PB7 */

#endif /* __BOARDS_ARM_STM32L0_NPZERO_G1S_DK_INCLUDE_BOARD_H */
