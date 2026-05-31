/****************************************************************************
 * boards/arm64/am62x/pocketbeagle2/include/board.h
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

/* PocketBeagle 2 board definitions */

#ifndef __BOARDS_ARM64_AM62X_POCKETBEAGLE2_INCLUDE_BOARD_H
#define __BOARDS_ARM64_AM62X_POCKETBEAGLE2_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* UART functional clock, sourced from HSDIV4_CLKOUT1 as configured by the
 * R5 SYSFW.  Matches AM62X_UART_SCLK in hardware/am62x_uart.h.
 */

#define BOARD_UART_SCLK         48000000ul  /* 48 MHz */

/* ARM Generic Timer (CNTPCT_EL0) reference clock.
 * On AM62x this is driven by the 200 MHz SYSCLK0 / 4 path.
 * Configured in defconfig via CONFIG_ARCH_TIMER_FREQ.
 */

#define BOARD_ARCH_TIMER_FREQ   200000000ul /* 200 MHz */

/* LED definitions *********************************************************
 *
 * The PocketBeagle 2 user LEDs are on main GPIO0 lines 3-6.
 */

#define BOARD_LED_USR0      0
#define BOARD_LED_USR1      1
#define BOARD_LED_USR2      2
#define BOARD_LED_USR3      3
#define BOARD_NLEDS         4

#define BOARD_LED_USR0_BIT  (1 << BOARD_LED_USR0)
#define BOARD_LED_USR1_BIT  (1 << BOARD_LED_USR1)
#define BOARD_LED_USR2_BIT  (1 << BOARD_LED_USR2)
#define BOARD_LED_USR3_BIT  (1 << BOARD_LED_USR3)

/* GPIO test devices ********************************************************/

#define BOARD_NGPIOOUT      4
#define BOARD_NGPIOIN       0
#define BOARD_NGPIOINT      0

/* LED encoded OS states (used when CONFIG_ARCH_LEDS is set) ***************
 *
 * SYMBOL                     Meaning                   USR0  USR1  USR2
 * ----------------------  --------------------------   ----  ----  ----
 * LED_STARTED              NuttX has been started       ON    OFF   OFF
 * LED_HEAPALLOCATE         Heap has been allocated      ON    ON    OFF
 * LED_IRQSENABLED          Interrupts enabled           ON    ON    ON
 * LED_STACKCREATED         Idle stack created           OFF   ON    OFF
 * LED_INIRQ                In an interrupt              (no change)
 * LED_SIGNAL               In a signal handler          (no change)
 * LED_ASSERTION            Assertion failed             OFF   OFF   ON
 * LED_PANIC                Board panicked               Blinking USR2
 */

#define LED_STARTED          0
#define LED_HEAPALLOCATE     1
#define LED_IRQSENABLED      2
#define LED_STACKCREATED     3
#define LED_INIRQ            4
#define LED_SIGNAL           5
#define LED_ASSERTION        6
#define LED_PANIC            7

#endif /* __BOARDS_ARM64_AM62X_POCKETBEAGLE2_INCLUDE_BOARD_H */
