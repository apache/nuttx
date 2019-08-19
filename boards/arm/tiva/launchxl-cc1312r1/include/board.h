/****************************************************************************
 * boards/arm/tiva/launchxl-cc1312r1/include/board.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author:  Gregory Nutt <gnutt@nuttx.org>
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

#ifndef __BOARDS_ARM_TIVA_LAUNCHXL_CC1312R1_INCLUDE_BOARD_H
#define __BOARDS_ARM_TIVA_LAUNCHXL_CC1312R1_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Frequency of the ARM core clock */

#define SYSCLK_FREQUENCY      48000000

/* Peripheral Clock (PCLK)
 *
 * Same frequency as the SYSCLK
 */

#define PCLK_FREQUENCY       SYSCLK_FREQUENCY

/* LED definitions **********************************************************/

/* The LaunchXL-cc1312R1 and two LEDs controlled by software: DIO7_GLED (CR1)
 * and DIO6_RLED (CR2).  A high output value illuminates an LED.
 *
 * If CONFIG_ARCH_LEDS is not defined, then the user can control the LEDs in
 * any way.  The following definitions are used to access individual LEDs.
 */

/* LED index values for use with board_userled() */

#define BOARD_GLED        0
#define BOARD_RLED        1
#define BOARD_NLEDS       2

/* LED bits for use with board_userled_all() */

#define BOARD_GLED_BIT    (1 << BOARD_GLED)
#define BOARD_RLED_BIT    (1 << BOARD_RLED)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/cc1312_autoleds.c.  The LEDs are used to
 * encode OS-related events as follows:
 *
 *   ------------------- ---------------------------- ---- ----
 *   SYMBOL                  Meaning                  GLED RLED
 *   ------------------- ---------------------------- ---- ----
 */

#define LED_STARTED      0 /* NuttX has been started  OFF  OFF   */
#define LED_HEAPALLOCATE 1 /* Heap has been allocated OFF  ON    */
#define LED_IRQSENABLED  1 /* Interrupts enabled      OFF  ON    */
#define LED_STACKCREATED 2 /* Idle stack created      ON   OFF   */
#define LED_INIRQ        3 /* In an interrupt         N/C  GLOW  */
#define LED_SIGNAL       3 /* In a signal handler     N/C  GLOW  */
#define LED_ASSERTION    3 /* An assertion failed     N/C  GLOW  */
#define LED_PANIC        4 /* The system has crashed  OFF  BLINK */
#undef  LED_IDLE           /* MCU is is sleep mode    -Not used- */

/* Thus iF GLED is statically on, NuttX has successfully  booted and is,
 * apparently, running normally.  A soft glow of the RLED means that the
 * board is taking interrupts.   If GLED is off and GLED is flashing at
 * approximately 2Hz, then a fatal error has been detected and the system
 * has halted.
 */

/* Button definitions *******************************************************/

/* The LaunchXL-CC1312R1 has two push-puttons:
 *
 *   DIO13_BTN1  SW1  Low input sensed when depressed
 *   DIO14_BTN2  SW2  Low input sensed when depressed
 */

#define BUTTON_SW1        0
#define BUTTON_SW2        1
#define NUM_BUTTONS       2

#define BUTTON_SW1_BIT    (1 << BUTTON_SW1)
#define BUTTON_SW2_BIT    (1 << BUTTON_SW2)

/* Pin configuration ********************************************************/

#ifdef CONFIG_TIVA_UART0
/* UART0:
 *
 * The on-board XDS110 Debugger provide a USB virtual serial console using
 * UART0 (DIO2_RXD and DIO3_TXD).
 */

#  define GPIO_UART0_RX &g_gpio_uart0_rx
#  define GPIO_UART0_TX &g_gpio_uart0_tx
#endif

/* DMA **********************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Pin configuration ********************************************************/

struct cc13xx_pinconfig_s; /* Forward reference */

#ifdef CONFIG_TIVA_UART0
extern const struct cc13xx_pinconfig_s g_gpio_uart0_rx;
extern const struct cc13xx_pinconfig_s g_gpio_uart0_tx;
#endif

#endif /* __BOARDS_ARM_TIVA_LAUNCHXL_CC1312R1_INCLUDE_BOARD_H */
