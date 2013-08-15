/************************************************************************************
 * configs/sama5df3x-ek/include/board_396mhz.h
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

#ifndef __CONFIGS_SAMA5D3X_EK_INCLUDE_BOARD_396MHZ_H
#define __CONFIGS_SAMA5D3X_EK_INCLUDE_BOARD_396MHZ_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* After power-on reset, the SAMA5 device is running on a 12MHz internal RC.  These
 * definitions will configure operational clocking.
 *
 * This is the configuration provided in the Atmel example code.  This results in a
 * CPU clock of 396MHz:
 *
 *   MAINOSC:  Frequency = 12MHz (crystal)
 *   PLLA: PLL Divider = 1, Multiplier = 66 to generate PLLACK = 792MHz
 *   Master Clock (MCK): Source = PLLACK/2, Prescalar = 1, MDIV = 3 to generate
 *     MCK      =  132MHz
 *     CPU clock = 396MHz
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
 *   Multipler = 66
 */

#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_OUT       (0)
#define BOARD_CKGR_PLLAR_MUL       (65 << PMC_CKGR_PLLAR_MUL_SHIFT)
#define BOARD_CKGR_PLLAR_DIV       PMC_CKGR_PLLAR_DIV_BYPASS

/* PMC master clock register settings.
 *
 *  Master/Processor Clock Source Selection = PLLA
 *  Master/Processor Clock Prescaler        = 1
 *  PLLA Divider                            = 2
 *  Master Clock Division (MDIV)            = 3
 *
 *  NOTE: Bit PLLADIV2 must always be set to 1 when MDIV is set to 3.
 *
 *  Prescaler input                         = 792MHz / 2 = 396MHz
 *  Prescaler output                        = 396MHz / 1 = 396MHz
 *  Processor Clock (PCK)                   = 396MHz
 *  Master clock (MCK)                      = 396MHz / 3 = 132MHz
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1
#define BOARD_PMC_MCKR_PLLADIV     PMC_MCKR_PLLADIV2
#define BOARD_PMC_MCKR_MDIV        PMC_MCKR_MDIV_PCKDIV3
786
/* Resulting frequencies */

#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */
#define BOARD_PLLA_FREQUENCY       (792000000) /* PLLACK:  66 * 12Mhz / 1 */
#define BOARD_PCK_FREQUENCY        (396000000) /* CPU:     PLLACK / 2 / 1  */
#define BOARD_MCK_FREQUENCY        (132000000) /* MCK:     PLLACK / 2 / 1 / 3 */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV) + CLOCKODD + 2).
 *
 *   MCI_SPEED = MCK / (2*CLKDIV + CLOCKODD + 2)
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 132MHz, CLKDIV = 164, MCI_SPEED = 132MHz / (2*164 + 0 + 2) = 400 KHz */

#define HSMCI_INIT_CLKDIV          (164 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 132MHz, CLKDIV = 2 w/CLOCKODD, MCI_SPEED = 132MHz /(2*2 + 1 + 2) = 18.9 MHz */

#define HSMCI_MMCXFR_CLKDIV        ((2 << HSMCI_MR_CLKDIV_SHIFT) | HSMCI_MR_CLKODD)

/* MCK = 132MHz, CLKDIV = 2, MCI_SPEED = 132MHz /(2*2 + 0 + 2) = 22 MHz */

#define HSMCI_SDXFR_CLKDIV         (2 << HSMCI_MR_CLKDIV_SHIFT)
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

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* !__ASSEMBLY__ */
#endif  /* __CONFIGS_SAMA5D3X_EK_INCLUDE_BOARD_396MHZ_H */
