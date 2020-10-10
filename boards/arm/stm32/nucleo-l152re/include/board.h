/****************************************************************************
 * boards/arm/stm32/nucleo-l152re/include/board.h
 * include/arch/board/board.h
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

#ifndef __BOARDS_ARM_STM32_NUCLEOL152RE_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_NUCLEOL152RE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

#ifdef __KERNEL__
#  include "stm32.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Four different clock sources can be used to drive the system clock
 * (SYSCLK):
 *
 * - HSI high-speed internal oscillator clock
 *   Generated from an internal 16 MHz RC oscillator
 * - HSE high-speed external oscillator clock. 8 MHz from MCO output of
 *   ST-LINK.
 * - PLL clock
 * - MSI multispeed internal oscillator clock
 *   The MSI clock signal is generated from an internal RC oscillator.
 *   Seven frequency ranges are available: 65.536 kHz, 131.072 kHz,
 *   262.144 kHz, 524.288 kHz, 1.048 MHz, 2.097 MHz (default value)
 *   and 4.194 MHz.
 *
 * The devices have the following two secondary clock sources
 * - LSI low-speed internal RC clock
 *   Drives the watchdog and RTC.  Approximately 37KHz
 * - LSE low-speed external oscillator clock
 *   Driven by 32.768KHz crystal (X2) on the OSC32_IN and OSC32_OUT pins.
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     37000            /* Approximately 37KHz */
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768            /* X2 on board */

/* PLL Configuration
 *
 *   - PLL source is HSE      -> 8MHz
 *   - PLL multipler is 12    -> 96MHz PLL VCO clock output
 *   - PLL output divider 3   -> 32MHz divided down PLL VCO clock output
 *
 * Resulting SYSCLK frequency is 8MHz x 12 / 3 = 32MHz
 *
 * USB/SDIO:
 *   If the USB or SDIO interface is used in the application, the PLL VCO
 *   clock (defined by STM32_CFGR_PLLMUL) must be programmed to output a 96
 *   MHz frequency. This is required to provide a 48 MHz clock to the USB or
 *   SDIO (SDIOCLK or USBCLK = PLLVCO/2).
 * SYSCLK
 *   The system clock is derived from the PLL VCO divided by the output
 *   division factor.
 * Limitations:
 *   96 MHz as PLLVCO when the product is in range 1 (1.8V),
 *   48 MHz as PLLVCO when the product is in range 2 (1.5V),
 *   24 MHz when the product is in range 3 (1.2V).
 *   Output division to avoid exceeding 32 MHz as SYSCLK.
 *   The minimum input clock frequency for PLL is 2 MHz (when using HSE as
 *   PLL source).
 */

#if 1
#define STM32_CFGR_PLLSRC       RCC_CFGR_PLLSRC         /* PLL clocked by the HSE */
#define STM32_HSEBYP_ENABLE     1
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx12  /* PLLMUL = 12 */
#define STM32_CFGR_PLLDIV       RCC_CFGR_PLLDIV_3       /* PLLDIV = 3 */
#define STM32_PLL_FREQUENCY     (12*STM32_BOARD_XTAL)   /* PLL VCO Frequency is 96MHz */
#else
#define STM32_CFGR_PLLSRC       0                       /* PLL clocked by the HSI RC  */
#define STM32_CFGR_PLLMUL       RCC_CFGR_PLLMUL_CLKx6   /* PLLMUL = 6 */
#define STM32_CFGR_PLLDIV       RCC_CFGR_PLLDIV_3       /* PLLDIV = 3 */
#define STM32_PLL_FREQUENCY     (6*STM32_HSI_FREQUENCY) /* PLL VCO Frequency is 96MHz */
#endif

/* Use the PLL and set the SYSCLK source to be the divided down PLL VCO
 * output frequency (STM32_PLL_FREQUENCY divided by the PLLDIV value).
 */

#define STM32_SYSCLK_SW         RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS        RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY  (STM32_PLL_FREQUENCY/3)

/* AHB clock (HCLK) is SYSCLK (32MHz) */

#define STM32_RCC_CFGR_HPRE      RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY     STM32_SYSCLK_FREQUENCY
#define STM32_BOARD_HCLK         STM32_HCLK_FREQUENCY    /* Same as above, to satisfy compiler */

/* APB2 clock (PCLK2) is HCLK (32MHz) */

#define STM32_RCC_CFGR_PPRE2     RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY    STM32_HCLK_FREQUENCY
#define STM32_APB2_CLKIN         STM32_PCLK2_FREQUENCY

/* APB1 clock (PCLK1) is HCLK (32MHz) */

#define STM32_RCC_CFGR_PPRE1     RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY    STM32_HCLK_FREQUENCY

/* TODO: Timers */

/* LED definitions **********************************************************/

/* The Nucleo L152RE board has three LEDs.  Two of these are controlled by
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
 *           STM32L152RET6.
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
 * the Nucleo L152RE.  The following definitions describe how NuttX controls
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

/* The Nucleo L152RE supports two buttons; only one button is controllable
 * by software:
 *
 *   B1 USER:  user button connected to the I/O PC13 of the STM32L152RET6.
 *   B2 RESET: push button connected to NRST is used to RESET the
 *             STM32L152RET6.
 */

#define BUTTON_USER      0
#define NUM_BUTTONS      1

#define BUTTON_USER_BIT  (1 << BUTTON_USER)

/* Alternate function pin selections ****************************************/

/* USART */

/* By default the USART2 is connected to STLINK Virtual COM Port:
 * USART2_RX - PA3
 * USART2_TX - PA2
 */

#define GPIO_USART2_RX GPIO_USART2_RX_1 /* PA3 */
#define GPIO_USART2_TX GPIO_USART2_TX_1 /* PA2 */

#endif /* __BOARDS_ARM_STM32_NUCLEO_L152RE_INCLUDE_BOARD_H */
