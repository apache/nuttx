/****************************************************************************
 * boards/arm/efr32/xg24-ek2703a/include/board.h
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

#ifndef __BOARDS_ARM_EFR32_XG24_EK2703A_INCLUDE_BOARD_H
#define __BOARDS_ARM_EFR32_XG24_EK2703A_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The kit has a 39 MHz HFXO crystal and a 32.768 kHz LFXO crystal.  The
 * DPLL locks the HFRCODPLL to the HFXO:
 *
 *   SYSCLK = 39 MHz * (3839 + 1) / (1919 + 1) = 78 MHz
 *   HCLK   = SYSCLK                           = 78 MHz
 *   PCLK   = HCLK / 2                         = 39 MHz (USART0)
 */

#define BOARD_HFXO_FREQUENCY      39000000
#define BOARD_LFXO_FREQUENCY      32768

#define BOARD_DPLL_N              3839
#define BOARD_DPLL_M              1919

#define BOARD_SYSCLK_FREQUENCY    78000000
#define BOARD_HCLK_FREQUENCY      BOARD_SYSCLK_FREQUENCY
#define BOARD_PCLK_FREQUENCY      (BOARD_HCLK_FREQUENCY / 2)

/* LED definitions **********************************************************/

/* The kit has two yellow user LEDs, LED0 (PA4) and LED1 (PA7), both
 * active high.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED0                0
#define BOARD_LED1                1
#define BOARD_NLEDS               2

/* LED bits for use with board_userled_all() */

#define BOARD_LED0_BIT            (1 << BOARD_LED0)
#define BOARD_LED1_BIT            (1 << BOARD_LED1)

/* If CONFIG_ARCH_LEDS is defined, the LEDs show the state of NuttX:
 *
 *   SYMBOL                Meaning                     LED0   LED1
 *   -------------------   -------------------------   ----   -------
 *   LED_STARTED           NuttX has been started      OFF    OFF
 *   LED_HEAPALLOCATE      Heap has been allocated     OFF    OFF
 *   LED_IRQSENABLED       Interrupts enabled          OFF    OFF
 *   LED_STACKCREATED      Idle stack created          ON     OFF
 *   LED_INIRQ             In an interrupt             (no change)
 *   LED_SIGNAL            In a signal handler         (no change)
 *   LED_ASSERTION         An assertion failed         (no change)
 *   LED_PANIC             The system has crashed      N/C    Blinking
 *   LED_IDLE              MCU is in sleep mode        (not used)
 */

#define LED_STARTED               0
#define LED_HEAPALLOCATE          0
#define LED_IRQSENABLED           0
#define LED_STACKCREATED          1
#define LED_INIRQ                 2
#define LED_SIGNAL                2
#define LED_ASSERTION             2
#define LED_PANIC                 3

/* Button definitions *******************************************************/

/* The kit has two user buttons, BTN0 (PB2) and BTN1 (PB3), low when
 * pressed and debounced by an RC filter.
 */

#define BUTTON_BTN0               0
#define BUTTON_BTN1               1
#define NUM_BUTTONS               2

#define BUTTON_BTN0_BIT           (1 << BUTTON_BTN0)
#define BUTTON_BTN1_BIT           (1 << BUTTON_BTN1)

/* Pin routing **************************************************************/

/* USART0 is connected to the virtual COM port of the on-board debugger:
 * TX on PA5 and RX on PA6 (RTS PA8 and CTS PA9 are not used).
 */

#define GPIO_USART0_TX  (GPIO_PORTA | GPIO_PIN(5) | GPIO_PUSHPULL | \
                         GPIO_OUTPUT_SET)
#define GPIO_USART0_RX  (GPIO_PORTA | GPIO_PIN(6) | GPIO_INPUT_PULLUP)

#endif /* __BOARDS_ARM_EFR32_XG24_EK2703A_INCLUDE_BOARD_H */
