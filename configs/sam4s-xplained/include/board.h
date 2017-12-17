/************************************************************************************
 * configs/sam4s-xplained/include/board.h
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
 ************************************************************************************/

#ifndef __CONFIGS_SAM4L_XPLAINED_INCLUDE_BOARD_H
#define __CONFIGS_SAM4L_XPLAINED_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  ifdef CONFIG_SAM34_GPIO_IRQ
#    include <arch/irq.h>
#  endif
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* After power-on reset, the sam3u device is running on a 4MHz internal RC.  These
 * definitions will configure clocking with MCK = 48MHz, PLLA = 96, and CPU=120MHz.
 */

/* Main oscillator register settings */

#define BOARD_CKGR_MOR_MOSCXTST    (63 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration:
 *
 * Source: 12MHz crystall at 12MHz
 * PLLdiv: 10
 * PLLmul: 1 (bypassed)
 * Fpll:   (12MHz * 10) / 1 = 120MHz
 */

#define BOARD_MAINOSC_FREQUENCY    (12000000)
#define BOARD_CKGR_PLLAR_MUL       (9 << PMC_CKGR_PLLAR_MUL_SHIFT)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS
#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_PLLA_FREQUENCY       (10*BOARD_MAINOSC_FREQUENCY)

/* PMC master clock register settings */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1
#define BOARD_MCK_FREQUENCY        (BOARD_PLLA_FREQUENCY/1)
#define BOARD_CPU_FREQUENCY        (BOARD_PLLA_FREQUENCY/1)

/* USB UTMI PLL start-up time */

#define BOARD_CKGR_UCKR_UPLLCOUNT  (3 << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCK / (2*(CLKDIV+1))
 *   CLKDIV = MCI / MCI_SPEED / 2 - 1
 */

/* MCK = 48MHz, CLKDIV = 59, MCI_SPEED = 48MHz / 2 * (59+1) = 400 KHz */

#define HSMCI_INIT_CLKDIV          (59 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 48MHz, CLKDIV = 1, MCI_SPEED = 48MHz / 2 * (1+1) = 12 MHz */

#define HSMCI_MMCXFR_CLKDIV        (1 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 48MHz, CLKDIV = 0, MCI_SPEED = 48MHz / 2 * (0+1) = 24 MHz */

#define HSMCI_SDXFR_CLKDIV         (0 << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_SDWIDEXFR_CLKDIV     HSMCI_SDXFR_CLKDIV

/* FLASH wait states:
 *
 * DC Characteristics
 *
 * Parameter              Min   Typ  Max
 * ---------------------- ----- ----- ----
 * Vddcore DC Supply Core 1.08V 1.2V 1.32V
 * Vvddio  DC Supply I/Os 1.62V 3.3V 3.6V
 *
 *                     Wait   Maximum
 * Vddcore   Vvddio   States Frequency (MHz)
 * ------- ---------- ------ ---------------
 * 1.08V   1.62-3.6V    0        16
 * "   "   "  "-"  "    1        33
 * "   "   "  "-"  "    2        50
 * "   "   "  "-"  "    3        67
 * "   "   "  "-"  "    4        84
 * "   "   "  "-"  "    5       100
 * 1.08V   2.7-3.6V     0        20
 * "   "   " "-"  "     1        40
 * "   "   " "-"  "     2        60
 * "   "   " "-"  "     3        80
 * "   "   " "-"  "     4       100
 * 1.2V    1.62-3.6V    0        17
 * "  "    " "-"  "     1        34
 * "  "    " "-"  "     2        52
 * "  "    " "-"  "     3        69
 * "  "    " "-"  "     4        87
 * "  "    " "-"  "     5       104
 * "  "    " "-"  "     6       121
 * 1.2V    2.7-3.6V     0        21
 * "  "    " "-"  "     1        42
 * "  "    " "-"  "     2        63
 * "  "    " "-"  "     3        84
 * "  "    " "-"  "     4       105
 * "  "    " "-"  "     5       123 << SELECTION
 */

#define BOARD_FWS                  5

/* LED definitions ******************************************************************/
/* There are four LEDs on board the SAM4S Xplained board, two of these can be
 * controlled by software in the SAM4S:
 *
 *   LED              GPIO
 *   ---------------- -----
 *   D9  Yellow LED   PC10
 *   D10 Yellow LED   PC17
 *
 * Both can be illuminated by driving the GPIO output to ground (low).
 */

/* LED index values for use with board_userled() */

#define BOARD_D9             0
#define BOARD_D10            1
#define BOARD_NLEDS          2

/* LED bits for use with board_userled_all() */

#define BOARD_D9_BIT         (1 << BOARD_D9)
#define BOARD_D10_BIT        (1 << BOARD_D10)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/up_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL                Val  Meaning                      LED state
 *                                                          D9       D10
 * ----------------------- ---  -----------------------  -------- --------    */
#define LED_STARTED          0 /* NuttX has been started    OFF      OFF      */
#define LED_HEAPALLOCATE     0 /* Heap has been allocated   OFF      OFF      */
#define LED_IRQSENABLED      0 /* Interrupts enabled        OFF      OFF      */
#define LED_STACKCREATED     0 /* Idle stack created        ON       OFF      */
#define LED_INIRQ            0 /* In an interrupt             No change       */
#define LED_SIGNAL           0 /* In a signal handler         No change       */
#define LED_ASSERTION        0 /* An assertion failed         No change       */
#define LED_PANIC            0 /* The system has crashed    OFF      Blinking */
#define LED_IDLE             0 /* MCU is is sleep mode        Not used        */

/* Thus if D9 is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.  If D10 is flashing at approximately
 * 2Hz, then a fatal error has been detected and the system has halted.
 */

/* Button definitions ***************************************************************/
/* Mechanical buttons:
 *
 * The SAM4S Xplained has two mechanical buttons. One button is the RESET button
 * connected to the SAM4S reset line and the other is a generic user configurable
 * button labeled BP2. When a button is pressed it will drive the I/O line to GND.
 *
 *   PA5 BP2
 */

#define BUTTON_BP2           0
#define NUM_BUTTONS          1

#define BUTTON_BP2_BIT       (1 << BUTTON_BP2)

#endif  /* __CONFIGS_SAM4L_XPLAINED_INCLUDE_BOARD_H */
