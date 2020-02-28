/****************************************************************************
 * boards/arm/s32k1xx/s32k118evb/include/board.h
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_S32K118EVB_INCLUDE_BOARD_H
#define __BOARDS_ARM_S32K118EVB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* The S32K118EVB is fitted with a 40MHz Crystal */

#define BOARD_XTAL_FREQUENCY 40000000

/* The S32K118 will run at 48MHz */

/* LED definitions **********************************************************/

/* The S32K118EVB has one RGB LED:
 *
 *   RedLED   PTD16 (FTM0 CH1)
 *   GreenLED PTD15 (FTM0 CH0)
 *   BlueLED  PTE8  (FTM0 CH6)
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual RGB
 * components.
 *
 * The RGB components could, alternatively be controlled through PWM using
 * the common RGB LED driver.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_R       0
#define BOARD_LED_G       1
#define BOARD_LED_B       2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_R_BIT   (1 << BOARD_LED_R)
#define BOARD_LED_G_BIT   (1 << BOARD_LED_G)
#define BOARD_LED_B_BIT   (1 << BOARD_LED_B)

/* If CONFIG_ARCH_LEDs is defined, then NuttX will control the LEDs on board
 * the S32K118EVB.  The following definitions describe how NuttX controls the
 * LEDs:
 *
 *   SYMBOL                Meaning                      LED state
 *                                                      RED   GREEN  BLUE
 *   -------------------  ----------------------------  -----------------
 */

#define LED_STARTED       1 /* NuttX has been started    OFF   OFF    OFF */
#define LED_HEAPALLOCATE  2 /* Heap has been allocated   OFF   OFF    ON  */
#define LED_IRQSENABLED   0 /* Interrupts enabled        OFF   OFF    ON  */
#define LED_STACKCREATED  3 /* Idle stack created        OFF   ON     OFF */
#define LED_INIRQ         0 /* In an interrupt          (no change)       */
#define LED_SIGNAL        0 /* In a signal handler      (no change)       */
#define LED_ASSERTION     0 /* An assertion failed      (no change)       */
#define LED_PANIC         4 /* The system has crashed    FLASH OFF    OFF */
#undef  LED_IDLE            /* S32K118EVB in sleep mode (Not used)        */

/* Button definitions *******************************************************/

/* The S32K118EVB supports two buttons:
 *
 *   SW2  PTD3
 *   SW3  PTD5
 */

#define BUTTON_SW2         0
#define BUTTON_SW3         1
#define NUM_BUTTONS        2

#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)
#define BUTTON_SW3_BIT    (1 << BUTTON_SW3)

/* Alternate function pin selections ****************************************/

/* By default, the serial console will be provided on the OpenSDA VCOM port:
 *
 *   OpenSDA UART TX  PTB1(LPUART0_TX)
 *   OpenSDA UART RX  PTB0(LPUART0_RX)
 */

#define PIN_LPUART0_RX    PIN_LPUART0_RX_1  /* PTB0 */
#define PIN_LPUART0_TX    PIN_LPUART0_TX_1  /* PTB1 */

#endif  /* __BOARDS_ARM_S32K118EVB_INCLUDE_BOARD_H */
