/****************************************************************************
 * boards/arm/sam34/sam3u-ek/include/board.h
 *
 *   Copyright (C) 2009-2011, 2013 Gregory Nutt. All rights reserved.
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

#ifndef __BOARDS_ARM_SAM34_SAM3U_EK_INCLUDE_BOARD_H
#define __BOARDS_ARM_SAM34_SAM3U_EK_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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

/* After power-on reset, the sam3u device is running on a 4MHz internal RC.
 * These definitions will configure clocking
 *
 *   MAINOSC:  Frequency = 12MHz (crysta)
 *   PLLA: PLL Divider = 1, Multiplier = 16 to generate PLLACK = 192MHz
 *   Master Clock (MCK): Source = PLLACK, Prescalar = 1 to generate MCK = 96MHz
 *   CPU clock: 96MHz
 */

/* Main oscillator register settings.
 *
 *   The start up time should be should be:
 *   Start Up Time = 8 * MOSCXTST / SLCK = 56 Slow Clock Cycles.
 */

#define BOARD_CKGR_MOR_MOSCXTST    (62 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration.
 *
 *   Divider = 1
 *   Multipler = 16
 */

#define BOARD_CKGR_PLLAR_MUL       (15 << PMC_CKGR_PLLAR_MUL_SHIFT)
#define BOARD_CKGR_PLLAR_STMODE    PMC_CKGR_PLLAR_STMODE_FAST
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

#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */
#define BOARD_PLLA_FREQUENCY       (192000000) /* PLLACK:  16 * 12Mhz / 1 */
#define BOARD_MCK_FREQUENCY        (96000000)  /* MCK:     PLLACK / 2 */
#define BOARD_CPU_FREQUENCY        (96000000)  /* CPU:     MCK */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV+1)).
 *
 *   MCI_SPEED = MCK / (2*(CLKDIV+1))
 *   CLKDIV = MCI / MCI_SPEED / 2 - 1
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 96MHz, CLKDIV = 119, MCI_SPEED = 96MHz / 2 * (119+1) = 400 KHz */

#define HSMCI_INIT_CLKDIV          (119 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 96MHz, CLKDIV = 3, MCI_SPEED = 96MHz / 2 * (3+1) = 12 MHz */

#define HSMCI_MMCXFR_CLKDIV        (3 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 96MHz, CLKDIV = 1, MCI_SPEED = 96MHz / 2 * (1+1) = 24 MHz */

#define HSMCI_SDXFR_CLKDIV         (1 << HSMCI_MR_CLKDIV_SHIFT)
#define HSMCI_SDWIDEXFR_CLKDIV     HSMCI_SDXFR_CLKDIV

/* FLASH wait states
 *
 * FWS Max frequency
 *     1.62V 1.8V
 * --- ----- ------
 *  0  24MHz 27MHz
 *  1  40MHz 47MHz
 *  2  72MHz 84MHz
 *  3  84MHz 96MHz
 */

#define BOARD_FWS                  3

/* LED definitions **********************************************************/

#define LED_STARTED                0 /* LED0=OFF LED1=OFF LED2=OFF */
#define LED_HEAPALLOCATE           1 /* LED0=OFF LED1=OFF LED2=ON */
#define LED_IRQSENABLED            2 /* LED0=OFF LED1=ON  LED2=OFF */
#define LED_STACKCREATED           3 /* LED0=OFF LED1=ON  LED2=ON */

#define LED_INIRQ                  4 /* LED0=XXX LED1=TOG LED2=XXX */
#define LED_SIGNAL                 5 /* LED0=XXX LED1=XXX LED2=TOG */
#define LED_ASSERTION              6 /* LED0=TOG LED1=XXX LED2=XXX */
#define LED_PANIC                  7 /* LED0=TOG LED1=XXX LED2=XXX */

/* Button definitions *******************************************************/

#define BUTTON1                    1 /* Bit 0: Button 1 */
#define BUTTON2                    2 /* Bit 1: Button 2 */

#endif /* __BOARDS_ARM_SAM34_SAM3U_EK_INCLUDE_BOARD_H */
