/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/include/board.h
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

#ifndef __BOARDS_ARM_SAM34_SAM4S_XPLAINED_PRO_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAM34_SAM4S_XPLAINED_PRO_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdbool.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  ifdef CONFIG_SAM34_GPIO_IRQ
#    include <arch/irq.h>
#  endif
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the sam4s device is running on a 4MHz internal RC.
 * These definitions will configure clocking with MCK = 120MHz, PLLA = 240,
 * and CPU=120MHz.
 */

#define BOARD_32KOSC_FREQUENCY      (32768)
#define BOARD_SCLK_FREQUENCY        (BOARD_32KOSC_FREQUENCY)
#define BOARD_MAINOSC_FREQUENCY     (12000000)

/* Main oscillator register settings */

#define BOARD_CKGR_MOR_MOSCXTST     (63 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

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

/* LED definitions **********************************************************/

/* There are two LEDs on board the SAM4S Xplained Pro board, One of these
 * can be controlled by software in the SAM4S:
 *
 *   LED              GPIO
 *   ---------------- -----
 *   D301 Yellow LED  PC23
 *
 * Both can be illuminated by driving the GPIO output to ground (low).
 */

/* LED index values for use with board_userled() */

#define BOARD_D301           0
#define BOARD_NLEDS          1

/* LED bits for use with board_userled_all() */

#define BOARD_D301_BIT       (1 << BOARD_D301)

/* These LEDs are not used by the board port unless CONFIG_ARCH_LEDS is
 * defined.
 * In that case, the usage by the board port is defined in
 * include/board.h and src/up_leds.c.
 * The LEDs are used to encode OS-related events as follows:
 *
 *   SYMBOL                Val  Meaning                      LED state
 *                                                          D301
 * ----------------------- ---  -----------------------  --------------
 */

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
 * apparently, running normally.
 */

/* Button definitions *******************************************************/

/* Mechanical buttons:
 *
 * The SAM4S Xplained Pro has two mechanical buttons.
 * One button is the RESET button connected to the SAM4S reset line and the
 * other is a generic user configurable button labeled SW0.
 * When a button is pressed it will drive the I/O line to GND.
 *
 *   PA2 SW0
 */

#define BUTTON_SW0           0
#define NUM_BUTTONS          1

#define BUTTON_SW0_BIT       (1 << BUTTON_SW0)

/* NAND *********************************************************************/

#define GPIO_SMC_RB   (GPIO_INPUT | GPIO_SMC_NWAIT)

/* Address for transferring command bytes to the nandflash, CLE A22 */

#define BOARD_NCS0_NAND_CMDADDR   0x60400000

/* Address for transferring address bytes to the nandflash, ALE A21 */

#define BOARD_NCS0_NAND_ADDRADDR  0x60200000

/* Address for transferring data bytes to the nandflash. */

#define BOARD_NCS0_NAND_DATAADDR  0x60000000

#endif /* __BOARDS_ARM_SAM34_SAM4S_XPLAINED_PRO_INCLUDE_BOARD_H */
