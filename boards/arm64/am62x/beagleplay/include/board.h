/****************************************************************************
 * boards/arm64/am62x/beagleplay/include/board.h
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

/* BeaglePlay board definitions */

#ifndef __BOARDS_ARM64_AM62X_BEAGLEPLAY_INCLUDE_BOARD_H
#define __BOARDS_ARM64_AM62X_BEAGLEPLAY_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

#define BOARD_UART_SCLK         48000000ul   /* 48 MHz UART functional clock */
#define BOARD_ARCH_TIMER_FREQ   200000000ul  /* 200 MHz ARM generic timer    */

/* LED definitions *********************************************************
 *
 * BeaglePlay user LEDs
 *
 * LED index   Net name   GPIO
 *   0          USR0       GPIO1_22 (green)
 *   1          USR1       GPIO1_23 (yellow)
 *   2          USR2       GPIO1_24 (red)
 *   3          USR3       GPIO1_25 (blue)
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

/* LED encoded OS states */

#define LED_STARTED          0
#define LED_HEAPALLOCATE     1
#define LED_IRQSENABLED      2
#define LED_STACKCREATED     3
#define LED_INIRQ            4
#define LED_SIGNAL           5
#define LED_ASSERTION        6
#define LED_PANIC            7

#endif /* __BOARDS_ARM64_AM62X_BEAGLEPLAY_INCLUDE_BOARD_H */
