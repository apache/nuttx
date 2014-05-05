/************************************************************************************
 * configs/sam4s-xplained-pro/include/board.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *            Bob Doiron
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

#ifndef __CONFIGS_SAM4S_XPLAINED_PRO_INCLUDE_BOARD_H
#define __CONFIGS_SAM4S_XPLAINED_PRO_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  ifdef CONFIG_GPIO_IRQ
#    include <arch/irq.h>
#  endif
#endif

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* After power-on reset, the sam4s device is running on a 4MHz internal RC.  These
 * definitions will configure clocking with MCK = 120MHz, PLLA = 240, and CPU=120MHz.
 */

/* Main oscillator register settings */

#define BOARD_CKGR_MOR_MOSCXTST     (63 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

#define BOARD_32KOSC_FREQUENCY      (32768)
#define BOARD_SLCK_FREQUENCY        (BOARD_32KOSC_FREQUENCY)
#define BOARD_MAINOSC_FREQUENCY     (12000000)

#ifdef CONFIG_SAM34_UDP
/* PLLA configuration:
 *
 * Source: 12MHz crystall at 12MHz
 * PLLmul: 10
 * PLLdiv: 1 (bypassed)
 * Fpll:   (12MHz * 20) / 1 = 240MHz
 */

#  define BOARD_CKGR_PLLAR_MUL      (19 << PMC_CKGR_PLLAR_MUL_SHIFT)
#  define BOARD_CKGR_PLLAR_DIV      PMC_CKGR_PLLAR_DIV_BYPASS
#  define BOARD_CKGR_PLLAR_COUNT    (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#  define BOARD_PLLA_FREQUENCY      (20*BOARD_MAINOSC_FREQUENCY) /* PLLA = 240Mhz */

/* PMC master clock register settings */

#  define BOARD_PMC_MCKR_CSS        PMC_MCKR_CSS_PLLA
#  define BOARD_PMC_MCKR_PRES       PMC_MCKR_PRES_DIV2
#  define BOARD_MCK_FREQUENCY       (BOARD_PLLA_FREQUENCY/2)     /* MCK = 120Mhz */
#  define BOARD_CPU_FREQUENCY       (BOARD_PLLA_FREQUENCY/2)     /* CPU = 120Mhz */

/* USB UTMI PLL start-up time */

#  define BOARD_CKGR_UCKR_UPLLCOUNT (3 << PMC_CKGR_UCKR_UPLLCOUNT_SHIFT)

#else
/* PLLA configuration:
 *
 * Source: 12MHz crystall at 12MHz
 * PLLmul: 10
 * PLLdiv: 1 (bypassed)
 * Fpll:   (12MHz * 10) / 1 = 120MHz
 */

#  define BOARD_CKGR_PLLAR_MUL       (9 << PMC_CKGR_PLLAR_MUL_SHIFT)
#  define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS
#  define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#  define BOARD_PLLA_FREQUENCY       (10*BOARD_MAINOSC_FREQUENCY) /* PLLA = 120Mhz */

/* PMC master clock register settings */

#  define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#  define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1
#  define BOARD_MCK_FREQUENCY        (BOARD_PLLA_FREQUENCY)       /* MCK = 120Mhz */
#  define BOARD_CPU_FREQUENCY        (BOARD_PLLA_FREQUENCY)       /* CPU = 120Mhz */
#endif

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCK / (2*(CLKDIV+1))
 *   CLKDIV = MCI / MCI_SPEED / 2 - 1
 */

/* MCK = 120MHz, CLKDIV = 149, MCI_SPEED = 120MHz / 2 * (149+1) = 400 KHz */

#define HSMCI_INIT_CLKDIV            (149 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 120MHz, CLKDIV = 3, MCI_SPEED = 120MHz / 2 * (3+1) = 15 MHz */

#define HSMCI_MMCXFR_CLKDIV          (3 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 120MHz, CLKDIV = 0, MCI_SPEED = 120MHz / 2 * (2+1) = 20 MHz */

#define HSMCI_SDXFR_CLKDIV           (2 << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_SDWIDEXFR_CLKDIV       HSMCI_SDXFR_CLKDIV

#ifdef CONFIG_SAM34_UDP
/* The PLL clock (USB_48M or UDPCK) is driven from the output of the PLL,
 * PLLACK.  The PLL clock must be 48MHz.  PLLACK can be divided down via the
 * PMC USB register to provide the PLL clock.  So in order to use the USB
 * feature, the PLL output must be a multiple of 48MHz.
 *
 * PLLACK = 240MHz, USBDIV=4, USB_48M = 240 MHz / (4 + 1) = 48MHz
 * PLLACK = 192MHz, USBDIV=5, USB_48M = 192 MHz / (3 + 1) = 48MHz
 */

#  define BOARD_PMC_USBS             (0)
#  define BOARD_PMC_USBDIV           (4 << PMC_USB_USBDIV_SHIFT)
#endif

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

#define BOARD_FWS                    5

/* LED definitions ******************************************************************/
/* There are two LEDs on board the SAM4S Xplained Pro board, One of these can be
 * controlled by software in the SAM4S:
 *
 *   LED              GPIO
 *   ---------------- -----
 *   D301 Yellow LED   PC10
 *
 * Both can be illuminated by driving the GPIO output to ground (low).
 */

/* LED index values for use with sam_setled() */

#define BOARD_D301           0
#define BOARD_NLEDS          1

/* LED bits for use with sam_setleds() */

#define BOARD_D301_BIT       (1 << BOARD_D301)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.  In that case, the usage by the board port is defined in
 * include/board.h and src/up_leds.c. The LEDs are used to encode OS-related
 * events as follows:
 *
 *   SYMBOL                Val  Meaning                      LED state
 *                                                          D301
 * ----------------------- ---  -----------------------  --------------     */
#define LED_STARTED          0 /* NuttX has been started    OFF             */
#define LED_HEAPALLOCATE     0 /* Heap has been allocated   OFF             */
#define LED_IRQSENABLED      0 /* Interrupts enabled        OFF             */
#define LED_STACKCREATED     1 /* Idle stack created        ON              */
#define LED_INIRQ            2 /* In an interrupt           OFF             */
#define LED_SIGNAL           2 /* In a signal handler       OFF             */
#define LED_ASSERTION        4 /* An assertion failed       No change       */
#define LED_PANIC            3 /* The system has crashed    Flash @ 250ms   */
#define LED_IDLE             4 /* MCU is is sleep mode      Not used        */

#define LED_D301_OFF true /* GPIO high for OFF */
#define LED_D301_ON false /* GPIO low for ON */

/* Thus if D301 is statically on, NuttX has successfully booted and is,
 * apparently, running normmally.
 */

/* Button definitions ***************************************************************/
/* Mechanical buttons:
 *
 * The SAM4S Xplained Pro has two mechanical buttons. One button is the RESET button
 * connected to the SAM4S reset line and the other is a generic user configurable
 * button labeled SW0. When a button is pressed it will drive the I/O line to GND.
 *
 *   PA2 SW0
 */

#define BUTTON_SW0           0
#define NUM_BUTTONS          1

#define BUTTON_SW0_BIT       (1 << BUTTON_SW0)

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/************************************************************************************
 * Public Function Prototypes
 ************************************************************************************/
/************************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAM3U architectures must provide the following entry point.  This entry point
 *   is called early in the intitialization -- after all memory has been configured
 *   and mapped but before any devices have been initialized.
 *
 ************************************************************************************/

void sam_boardinitialize(void);

/************************************************************************************
 * Name:  sam_ledinit, sam_setled, and sam_setleds
 *
 * Description:
 *   If CONFIG_ARCH_LEDS is defined, then NuttX will control the on-board LEDs.  If
 *   CONFIG_ARCH_LEDS is not defined, then the following interfaces are available to
 *   control the LEDs from user applications.
 *
 ************************************************************************************/

#ifndef CONFIG_ARCH_LEDS
void sam_ledinit(void);
void sam_setled(int led, bool ledon);
void sam_setleds(uint8_t ledset);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIGS_SAM4S_XPLAINED_PRO_INCLUDE_BOARD_H */
