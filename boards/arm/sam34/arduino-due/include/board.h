/****************************************************************************
 * boards/arm/sam34/arduino-due/include/board.h
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_SAM34_ARDUINO_DUE_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAM34_ARDUINO_DUE_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#  ifdef CONFIG_SAM34_GPIO_IRQ
#    include <arch/irq.h>
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the SAM3X device is running on a 4MHz internal RC.
 * These definitions will configure clocking
 *
 * MAINOSC:  Frequency = 12MHz (crystal)
 * PLLA: PLL Divider = 1, Multiplier = 14 to generate PLLACK = 168MHz
 * Master Clock (MCK): Source = PLLACK, Prescalar = 1 to generate MCK = 84MHz
 * CPU clock: 84MHz
 */

#define BOARD_32KOSC_FREQUENCY     (32768)
#define BOARD_SCLK_FREQUENCY       (BOARD_32KOSC_FREQUENCY)
#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */

/* Main oscillator register settings.
 *
 *   The start up time should be should be:
 *   Start Up Time = 8 * MOSCXTST / SLCK = 56 Slow Clock Cycles.
 */

#define BOARD_CKGR_MOR_MOSCXTST    (62 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration.
 *
 *   Divider = 1
 *   Multipler = 14
 */

#define BOARD_CKGR_PLLAR_MUL       (13 << PMC_CKGR_PLLAR_MUL_SHIFT)
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS

/* PMC master clock register settings.
 *
 *  Source = PLLA
 *  Divider = 2
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV2

/* USB UTMI PLL start-up time */

#define BOARD_CKGR_UCKR_UPLLCOUNT  (3 << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)

/* Resulting frequencies */

#define BOARD_PLLA_FREQUENCY       (168000000) /* PLLACK:  14 * 12Mhz / 1 */
#define BOARD_MCK_FREQUENCY        (84000000)  /* MCK:     PLLACK / 2 */
#define BOARD_CPU_FREQUENCY        (84000000)  /* CPU:     MCK */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock
 * (MCK) divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCCK / (2*(CLKDIV+1))
 *   CLKDIV = MCCK / MCI_SPEED / 2 - 1
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 84MHz, CLKDIV = 104,
 * MCI_SPEED = 84MHz / 2 * (104+1) = 400 KHz
 */

#define HSMCI_INIT_CLKDIV          (104 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 84MHz, CLKDIV = 2, MCI_SPEED = 84MHz / 2 * (2+1) = 14 MHz */

#define HSMCI_MMCXFR_CLKDIV        (1 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 84MHz, CLKDIV = 1, MCI_SPEED = 84MHz / 2 * (1+1) = 21 MHz */

#define HSMCI_SDXFR_CLKDIV         (1 << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_SDWIDEXFR_CLKDIV     HSMCI_SDXFR_CLKDIV

/* FLASH wait states
 *
 * FWS MAX FREQUENCY
 *     1.62V 1.8V
 * --- ----- ------
 *  0  17MHz 19MHz
 *  1  45MHz 50MHz
 *  2  58MHz 64MHz
 *  3  70MHz 80MHz
 *  4  78MHz 90MHz
 */

#define BOARD_FWS                  4

/* LED definitions **********************************************************/

/*  There are three user-controllable LEDs on board the Arduino Due board:
 *
 *     LED              GPIO
 *     ---------------- -----
 *     L   Amber LED    PB27
 *     TX  Yellow LED   PA21
 *     RX  Yellow LED   PC30
 *
 * LED L is connected to ground and can be illuminated by driving the PB27
 * output high. The TX and RX LEDs are pulled high and can be illuminated by
 * driving the corresponding
 * GPIO output to low.
 */

/* LED index values for use with board_userled() */

#define BOARD_LED_L       0
#define BOARD_LED_RX      1
#define BOARD_LED_TX      2
#define BOARD_NLEDS       3

/* LED bits for use with board_userled_all() */

#define BOARD_LED_L_BIT   (1 << BOARD_LED_L)
#define BOARD_LED_RX_BIT  (1 << BOARD_LED_RX)
#define BOARD_LED_TX_BIT  (1 << BOARD_LED_TX)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/sam_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *  SYMBOL                MEANING                         LED STATE
 *                                                         L         TX       RX
 *  -----------------------  --------------------------  -------- -------- --------
 */

#define LED_STARTED       0  /* NuttX has been started     OFF      OFF      OFF       */
#define LED_HEAPALLOCATE  0  /* Heap has been allocated    OFF      OFF      OFF       */
#define LED_IRQSENABLED   0  /* Interrupts enabled         OFF      OFF      OFF       */
#define LED_STACKCREATED  1  /* Idle stack created         ON       OFF      OFF       */
#define LED_INIRQ         2  /* In an interrupt            N/C      GLOW     OFF       */
#define LED_SIGNAL        2  /* In a signal handler        N/C      GLOW     OFF       */
#define LED_ASSERTION     2  /* An assertion failed        N/C      GLOW     OFF       */
#define LED_PANIC         3  /* The system has crashed     N/C      N/C      Blinking  */
#define LED_PANIC         3  /* MCU is is sleep mode       ------ Not used --------    */

/* Thus if LED L is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.  If LED RX is glowing, then NuttX is
 * handling interrupts (and also signals and assertions).  If TX is flashing
 * at approximately 2Hz, then a fatal error has been detected and the system
 */

/* Button definitions *******************************************************/

/* There are no buttons on the Arduino Due board. */

/* GPIO pin configurations **************************************************/

#if 1 /* #ifdef CONFIG_ARDUINO_DUE_REV3 works with REV2 as well */

/* This port was performed on the Arduino Due Rev 2 board.
 * A NuttX user reported issues with the serial port on his Aduino Due Rev 3
 * board.
 * That problem was resolved as follows:
 *
 *  "... The issue was in my hardware. I found the difference between Arduino
 *   Due shematics (revision R2) and actual PCB layout of my Arduino (revision
 *   R3). On a schematics which I download from arduino.cc was shown that 2nd
 *   pin of IC10 is connected to the ground, but on my Arduino the 2nd pin
 *   of IC10 was connected to its 1st pin instead of ground and in my case
 *   IC10 works in open-drain mode, but RX0 pin on SAM3x doesn't have pull-up
 *   and thus TX signal from AtMega16U2 can't reach the SAM3x input pin, on
 *   this pin always '0'.
 *
 *  "My solution is to enable internal pull-up in SAM3x. ...
 *   Now shell console on UART0 (via USB programming connector) of Arduino Due
 *   Due works as expected."
 */

#  undef  GPIO_UART0_RXD
#  define GPIO_UART0_RXD  (GPIO_PERIPHA | GPIO_CFG_DEFAULT | GPIO_PORT_PIOA | \
                           GPIO_PIN8 | GPIO_CFG_PULLUP)
#endif
#endif /* __BOARDS_ARM_SAM34_ARDUINO_DUE_INCLUDE_BOARD_H */
