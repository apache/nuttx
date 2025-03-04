/****************************************************************************
 * boards/avr/at32uc3/mizar32a/include/board.h
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

#ifndef __BOARDS_AVR_AT32UC3_MIZAR32A_INCLUDE_BOARD_H
#define __BOARDS_AVR_AT32UC3_MIZAR32A_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Oscillator setup:  RCOSC, OSC32, OSC0, OSC1.  Only RCOSC, OSC0, or PLL0
 * can drive the main clock.
 */

/* The RCOSC frequency needs to be calibrated! */

#define AVR32_FRCOSC         115000     /* RCOSC frequency in Hz. 115KHz nominal */

#define AVR32_FOSC32         32768      /* OSC32 frequency in Hz */
#define AVR32_OSC32STARTUP   3          /* OSC32 startup time in RCOSC periods */

#define AVR32_FOSC0          12000000   /* OSC0 frequency in Hz */
#define AVR32_OSC0STARTUP    3          /* OSC0 startup time in RCOSC periods. */

/* #define AVR32_FOSC1       12000000     OSC1 frequency: Hz.
 * #define AVR32_OSC1STARTUP 3            OSC1 startup time in RCOSC periods.
 */

/* PLL setup
 *
 *   FOSC0 MUL DIV PLL DIV2_EN CPU_CLOCK PBA_CLOCK   COMMENT
 *   (MHz)         (MHz)        (MHz)    (MHz)
 *    12   15   1  192     1     12       12
 *    12    9   3   40     1     20       20    PLL out of spec
 *    12   15   1  192     1     24       12
 *    12    9   1  120     1     30       15
 *    12    9   3   40     0     40       20    PLL out of spec
 *    12   15   1  192     1     48       12
 *    12   15   1  192     1     48       24
 *    12    8   1  108     1     54       27
 *    12    9   1  120     1     60       15
 *    12    9   1  120     1     60       30
 *    12   10   1  132     1     66       16.5  This applies for Mizar32-A.
 */

#define AVR32_CLOCK_PLL0_OSC0 1
#undef AVR32_CLOCK_PLL0_OSC1
#define AVR32_PLL0_MUL        10
#define AVR32_PLL0_DIV        1
#define AVR32_PLL0_DIV2       1
#define AVR32_PLL0_WBWM       0
#define AVR32_PLL0_FREQ       132000000

/* Set PLL1 @ 96 MHz from OSC0: 12MHz*(7+1)/1 = 96MHz */

#define AVR32_CLOCK_PLL1_OSC0 1
#undef AVR32_CLOCK_PLL1_OSC1
#define AVR32_PLL1_MUL        7
#define AVR32_PLL1_DIV        1
#define AVR32_PLL1_DIV2       1
#define AVR32_PLL1_WBWM       0
#define AVR32_PLL1_FREQ       96000000

/* Clock divider setup */

#define AVR32_CKSEL_CPUDIV    0
#define AVR32_CKSEL_HSBDIV    0
#define AVR32_CKSEL_PBADIV    1
#define AVR32_CKSEL_PBBDIV    0

/* GCLK_USBB */

#undef  AVR32_CLOCK_USB_PLL0
#define AVR32_CLOCK_USB_PLL1  1
#undef  AVR32_CLOCK_USB_OSC0
#undef  AVR32_CLOCK_USB_OSC1
#define AVR32_CLOCK_USB_DIV   0

/* Main Clock setup: Select OSC0 as the main clock.
 *
 * - A 12MHz crystal is provided on he board for OSC0.
 * - The Mizar32-A board has no support for OSC1.
 * - There are pads for the 32KHz OSC32, but it is not populated on the
 *   board.
 */

#define AVR32_CLOCK_OSC0      1
#undef  AVR32_CLOCK_OSC1          /* Not supported */
#undef  AVR32_CLOCK_OSC32         /* Not populated */
#define AVR32_CLOCK_PLL0          /* Enable PLL0   */
#undef  AVR32_CLOCK_PLL1

#define AVR32_CPU_CLOCK       66000000
#define AVR32_PBA_CLOCK       16500000

/* Pin muliplexing selecion *************************************************/

#define PINMUX_USART1_RXD     PINMUX_USART1_RXD_0
#define PINMUX_USART1_TXD     PINMUX_USART1_TXD_0

/* LED definitions **********************************************************/

/* The Mizar32-A board has only one controllable LED */
#define LED_STARTED       0  /* No LEDs */
#define LED_HEAPALLOCATE  1  /* LED1 on */
#define LED_IRQSENABLED   2  /* LED2 on */
#define LED_STACKCREATED  3  /* LED1 on */
#define LED_INIRQ         4  /* LED1 off */
#define LED_SIGNAL        5  /* LED2 on */
#define LED_ASSERTION     6  /* LED1 + LED2 */
#define LED_PANIC         7  /* LED1 / LED2 blinking */

/* Button definitions *******************************************************/

/* The Mizar32 board has 2 BUTTONs,
 * one of which can be sensed through GPIO pins.
 */

#define BUTTON1               1 /* Bit 0: Button 1 */

#endif /* __BOARDS_AVR_AT32UC3_MIZAR32A_INCLUDE_BOARD_H */
