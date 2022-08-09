/****************************************************************************
 * boards/arm/stm32f0l0g0/stm32l0538-disco/include/board.h
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

#ifndef __BOARDS_ARM_STM32F0L0G0_STM32L0538_DISCO_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32F0L0G0_STM32L0538_DISCO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* HSI - Internal 16 MHz RC Oscillator
 * LSI - 32 KHz RC
 * HSE - 8 MHz from MCO output of ST-LINK
 * LSE - 32.768 kHz
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSEBYP_ENABLE
#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000            /* Between 30kHz and 60kHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768            /* X2 on board */

/* PLL source is HSE/1, PLL multipler is 8:
 *   PLL frequency is 8MHz (XTAL) x 8 = 64MHz
 */

#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC
#define STM32_CFGR_PLLXTPRE     0
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx8
#define STM32_PLL_FREQUENCY     (8*STM32_BOARD_XTAL)

/* Use the PLL and set the SYSCLK source to be the PLL/2 (32MHz) */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_CFGR_PLLDIV       RCC_CFGR_PLLDIV_2
#define STM32_SYSCLK_FREQUENCY  STM32_PLL_FREQUENCY/2

/* AHB clock (HCLK) is SYSCLK (32MHz) */

#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY    STM32_SYSCLK_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (32MHz) */

#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY   STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN        (STM32_PCLK2_FREQUENCY)

/* APB1 clock (PCLK1) is HCLK/2 (16MHz) */

#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY   (STM32_HCLK_FREQUENCY/2)

/* 48MHz clock configuration */

#if defined(CONFIG_STM32F0L0G0_USB) || defined(CONFIG_STM32F0L0G0_RNG)
#  define STM32_USE_CLK48       1
#  define STM32_CLK48_SEL       RCC_CCIPR_CLK48SEL_HSI48
#  define STM32_HSI48_SYNCSRC   SYNCSRC_NONE
#endif

/* TODO: timers */

/* LED definitions **********************************************************/

/* The STM32L0538-DISCO board has three LEDs.  Two of these are controlled by
 * logic on the board and are not available for software control:
 *
 * LD1 COM:  LD1 default status is red.  LD1 turns to green to indicate that
 *           communications are in progress between the PC and the
 *           ST-LINK/V2-1.
 * LD3 PWR:  red LED indicates that the board is powered.
 *
 * And one can be controlled by software:
 *
 * User LD2: green LED is a user LED connected to the I/O PA5 of the
 *           STM32L053C8T6.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LED in
 * any way.  The following definition is used to access the LED.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED1       0 /* User LD2 */
#define BOARD_NLEDS      1

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT   (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LED on board
 * the STM32L0538-DISCO.  The following definitions describe how NuttX
 * controls the LED:
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
 *   LED_IDLE            STM32 is is sleep mode   Not used
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

/* The STM32L0538-DISCO supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PA0 of the STM32L053C8T6.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32L053C8T6.
 */

#define BUTTON_USER      0
#define NUM_BUTTONS      1

#define BUTTON_USER_BIT  (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* USART */

/* By default the USART1 is connected to STLINK Virtual COM Port:
 *   USART1_RX - PA10
 *   USART1_TX - PA9
 */

#define GPIO_USART1_RX GPIO_USART1_RX_1 /* PA10 */
#define GPIO_USART1_TX GPIO_USART1_TX_1 /* PA9 */

/* SPI1 - E-papper display:
 *   SPI1_MISO - not used
 *   SPI1_MOSI - PB5
 *   SPI1_SCK  - PB3
 */

#undef  GPIO_SPI1_MISO                  /* Not used */
#define GPIO_SPI1_MOSI GPIO_SPI1_MOSI_3 /* PB5 */
#define GPIO_SPI1_SCK  GPIO_SPI1_SCK_2  /* PB3 */

/* SPI2 - NFC connector:
 *   SPI2_MISO - PB14
 *   SPI2_MOSI - PB15
 *   SPI2_SCK  - PB13
 */

#define GPIO_SPI2_MISO GPIO_SPI2_MISO_1 /* PB14 */
#define GPIO_SPI2_MOSI GPIO_SPI2_MOSI_1 /* PB15 */
#define GPIO_SPI2_SCK  GPIO_SPI2_SCK_3  /* PB13 */

#endif /* __BOARDS_ARM_STM32F0L0G0_STM32L0538_DISCO_INCLUDE_BOARD_H */
