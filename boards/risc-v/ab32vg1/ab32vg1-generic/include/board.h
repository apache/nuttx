/****************************************************************************
 * boards/risc-v/ab32vg1/ab32vg1-generic/include/board.h
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

#ifndef __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_INCLUDE_BOARD_H
#define __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_INCLUDE_BOARD_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking: 26 MHz crystal, PLL0 at 240 MHz, system clock set by
 * CONFIG_AB32VG1_SYSCLK_MHZ.
 */

/* LED definitions **********************************************************/

/* The board has one RGB LED (schematic V02): red on PE1, green on PE4 and
 * blue on PA1, all active low.
 */

#define BOARD_LED_RED       0
#define BOARD_LED_GREEN     1
#define BOARD_LED_BLUE      2
#define BOARD_NLEDS         3

#define BOARD_LED_RED_BIT   (1 << BOARD_LED_RED)
#define BOARD_LED_GREEN_BIT (1 << BOARD_LED_GREEN)
#define BOARD_LED_BLUE_BIT  (1 << BOARD_LED_BLUE)

/* When CONFIG_ARCH_LEDS is defined, NuttX controls the LED:
 *
 *   SYMBOL              Meaning                     LED state
 *   ------------------  --------------------------  ----------------
 *   LED_STARTED         NuttX has been started      Blue
 *   LED_HEAPALLOCATE    Heap has been allocated     Blue
 *   LED_IRQSENABLED     Interrupts enabled          Blue
 *   LED_STACKCREATED    Idle stack created          Green
 *   LED_INIRQ           In an interrupt             (no change)
 *   LED_SIGNAL          In a signal handler         (no change)
 *   LED_ASSERTION       An assertion failed         Red
 *   LED_PANIC           The system has crashed      Red, blinking
 *   LED_IDLE            MCU is in sleep mode        (not used)
 */

#define LED_STARTED         0
#define LED_HEAPALLOCATE    0
#define LED_IRQSENABLED     0
#define LED_STACKCREATED    1
#define LED_INIRQ           2
#define LED_SIGNAL          2
#define LED_ASSERTION       3
#define LED_PANIC           4

/* Button definitions *******************************************************/

/* Two user buttons (schematic V02), active low: S2 (KEY1) on PF1 and
 * S3 (KEY2) on PF0.  S1 is the reset button.
 */

#define BUTTON_KEY1         0
#define BUTTON_KEY2         1
#define NUM_BUTTONS         2

#define BUTTON_KEY1_BIT     (1 << BUTTON_KEY1)
#define BUTTON_KEY2_BIT     (1 << BUTTON_KEY2)

#endif /* __BOARDS_RISCV_AB32VG1_AB32VG1_GENERIC_INCLUDE_BOARD_H */
