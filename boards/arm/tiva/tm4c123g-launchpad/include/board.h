/****************************************************************************
 * boards/arm/tiva/tm4c123g-launchpad/include/board.h
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

#ifndef __BOARDS_ARM_TIVA_TMC4C123G_LAUNCHPAD_INCLUDE_BOARD_H
#define __BOARDS_ARM_TIVA_TMC4C123G_LAUNCHPAD_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* RCC settings.  Crystals on-board the TMC4C123G LaunchPad include:
 *
 *   16MHz connected to OSC0/1 (pins 40/41)
 *   32.768kHz connected to XOSC0/1 (pins 34/36)
 */

#define SYSCON_RCC_XTAL      SYSCON_RCC_XTAL16000KHZ /* On-board crystal is 16 MHz */
#define XTAL_FREQUENCY       16000000

/* Oscillator source is the main oscillator */

#define SYSCON_RCC_OSCSRC    SYSCON_RCC_OSCSRC_MOSC
#define SYSCON_RCC2_OSCSRC   SYSCON_RCC2_OSCSRC2_MOSC
#define OSCSRC_FREQUENCY     XTAL_FREQUENCY

/* Use system divider = 4; this corresponds to a system clock frequency
 * of (400 / 1) / 5 = 80MHz (Using RCC2 and DIV400).
 */

#define TIVA_SYSDIV          5
#define SYSCLK_FREQUENCY     80000000  /* 80MHz */

/* Peripheral Clock (PCLK)
 *
 * Same frequency as the SYSCLK
 */

#define PCLK_FREQUENCY       SYSCLK_FREQUENCY

/* Other RCC settings:
 *
 * - Main and internal oscillators enabled.
 * - PLL and sys dividers not bypassed
 * - PLL not powered down
 * - No auto-clock gating reset
 */

#define TIVA_RCC_VALUE (SYSCON_RCC_OSCSRC | SYSCON_RCC_XTAL | \
                      SYSCON_RCC_USESYSDIV | SYSCON_RCC_SYSDIV(TIVA_SYSDIV))

/* RCC2 settings
 *
 * - PLL and sys dividers not bypassed.
 * - PLL not powered down
 * - Not using RCC2
 *
 * When SYSCON_RCC2_DIV400 is not selected, SYSDIV2 is the divisor-1.
 * When SYSCON_RCC2_DIV400 is selected, SYSDIV2 is the divisor-1)/2, plus
 * the LSB:
 *
 * SYSDIV2 SYSDIV2LSB DIVISOR
 *   0       N/A         2
 *   1       0           3
 *   "       1           4
 *   2       0           5
 *   "       1           6
 *   etc.
 */

#if (TIVA_SYSDIV & 1) == 0
#  define TIVA_RCC2_VALUE (SYSCON_RCC2_OSCSRC | SYSCON_RCC2_SYSDIV2LSB | \
                           SYSCON_RCC2_SYSDIV_DIV400(TIVA_SYSDIV) | \
                           SYSCON_RCC2_DIV400 | SYSCON_RCC2_USERCC2)
#else
#  define TIVA_RCC2_VALUE (SYSCON_RCC2_OSCSRC | SYSCON_RCC2_SYSDIV_DIV400(TIVA_SYSDIV) | \
                           SYSCON_RCC2_DIV400 | SYSCON_RCC2_USERCC2)
#endif

/* LED definitions **********************************************************/

/* The TMC4C123G LaunchPad has a single RGB LED.
 * There is only one visible LED which will vary in color.
 * But, from the standpoint of the firmware, this appears as three LEDs:
 *
 *   BOARD_LED_R    -- Connected to PF1
 *   BOARD_LED_G    -- Connected to PF3
 *   BOARD_LED_B    -- Connected to PF2
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R               0
#define BOARD_LED_G               1
#define BOARD_LED_B               2
#define BOARD_NLEDS               3

/* LED bits for use with board_userled_all() */

#define BOARD_LED1_BIT            (1 << BOARD_LED1)
#define BOARD_LED2_BIT            (1 << BOARD_LED2)

/* If CONFIG_ARCH_LEDS is defined, then automated support for the LaunchPad
 * LEDs will be included in the build:
 *
 * OFF:
 * - OFF means that the OS is still initializing. Initialization is very fast
 *   so if you see this at all, it probably means that the system is hanging
 *   up somewhere in the initialization phases.
 *
 * GREEN or GREEN-ish
 * - This means that the OS completed initialization.
 *
 * Bluish:
 * - Whenever and interrupt or signal handler is entered, the BLUE LED is
 *   illuminated and extinguished when the interrupt or signal handler exits.
 *   This will add a BLUE-ish tinge to the LED.
 *
 * Redish:
 * - If a recovered assertion occurs, the RED component will be illuminated
 *   briefly while the assertion is handled.
 *   You will probably never see this.
 *
 * Flashing RED:
 * - In the event of a fatal crash, the BLUE and GREEN components will be
 *   extinguished and the RED component will FLASH at a 2Hz rate.
 *                                 RED  GREEN BLUE
 */

#define LED_STARTED       0     /* OFF  OFF   OFF                */
#define LED_HEAPALLOCATE  0     /* OFF  OFF   OFF                */
#define LED_IRQSENABLED   0     /* OFF  OFF   OFF                */
#define LED_STACKCREATED  1     /* OFF  ON    OFF                */
#define LED_INIRQ         2     /* NC   NC    ON  (momentary)    */
#define LED_SIGNAL        2     /* NC   NC    ON  (momentary)    */
#define LED_ASSERTION     3     /* ON   NC    NC  (momentary)    */
#define LED_PANIC         4     /* ON   OFF   OFF (flashing 2Hz) */

/* LED definitions **********************************************************/

/* The TMC4C123G LaunchPad has a two buttons:
 *
 *   BOARD_SW1    -- Connected to PF4
 *   BOARD_SW2    -- Connected to PF0
 */

#define BUTTON_SW1        0
#define BUTTON_SW2        1
#define NUM_BUTTONS       2

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)

/* Pin Multiplexing Disambiguation ******************************************/

#define GPIO_UART1_CTS    GPIO_UART1_CTS_1
#define GPIO_UART1_RTS    GPIO_UART1_RTS_1
#define GPIO_UART1_RX     GPIO_UART1_RX_1
#define GPIO_UART1_TX     GPIO_UART1_TX_1

/* CAN0 pin mux selection - defaults to port E in Kconfig */
#ifdef CONFIG_TIVA_CAN0
#  ifdef CONFIG_TM4C123G_LAUNCHPAD_CAN0RX_PF0
#    define GPIO_CAN0_RX GPIO_CAN0_RX_1
#  endif
#  ifdef CONFIG_TM4C123G_LAUNCHPAD_CAN0RX_PB4
#    define GPIO_CAN0_RX GPIO_CAN0_RX_2
#  endif
#  ifdef CONFIG_TM4C123G_LAUNCHPAD_CAN0RX_PE4
#    define GPIO_CAN0_RX GPIO_CAN0_RX_3
#  endif
#  ifdef CONFIG_TM4C123G_LAUNCHPAD_CAN0TX_PF3
#    define GPIO_CAN0_TX GPIO_CAN0_TX_1
#  endif
#  ifdef CONFIG_TM4C123G_LAUNCHPAD_CAN0TX_PB5
#    define GPIO_CAN0_TX GPIO_CAN0_TX_2
#  endif
#  ifdef CONFIG_TM4C123G_LAUNCHPAD_CAN0TX_PE5
#    define GPIO_CAN0_TX GPIO_CAN0_TX_3
#  endif
#endif

#endif /* __BOARDS_ARM_TMC4C123G_LAUNCHPAD_INCLUDE_BOARD_H */
