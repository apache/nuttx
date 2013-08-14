/************************************************************************************
 * configs/sama5df3x-ek/include/board_384mhz.h
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

#ifndef __CONFIGS_SAMA5D3X_EK_INCLUDE_BOARD_384MHZ_H
#define __CONFIGS_SAMA5D3X_EK_INCLUDE_BOARD_384MHZ_H

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
 * This is an alternative slower configuration that will produce a 48MHz USB clock
 * with the required accuracy.  When used with OHCI, an additional requirement is
 * the PLLACK be a multiple of 48MHz.  This setup results in a CPU clock of 384MHz.
 *
 *   MAINOSC:  Frequency = 12MHz (crystal)
 *   PLLA: PLL Divider = 1, Multiplier = 64 to generate PLLACK = 768MHz
 *   Master Clock (MCK): Source = PLLACK/2, Prescalar = 1, MDIV = 3 to generate
 *     MCK      =  128MHz
 *     CPU clock = 384MHz
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
 *   Multipler = 64
 */

#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_OUT       (0)
#define BOARD_CKGR_PLLAR_MUL       (63 << PMC_CKGR_PLLAR_MUL_SHIFT)
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
 *  Prescaler input                         = 768MHz / 2 = 384MHz
 *  Prescaler output                        = 768MHz / 1 = 384MHz
 *  Processor Clock (PCK)                   = 384MHz
 *  Master clock (MCK)                      = 396MHz / 3 = 129MHz
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1
#define BOARD_PMC_MCKR_PLLADIV     PMC_MCKR_PLLADIV2
#define BOARD_PMC_MCKR_MDIV        PMC_MCKR_MDIV_PCKDIV3

/* For OHCI Full-speed operations, the user has to perform the following:
 *
 *   1) Enable UHP peripheral clock, bit (1 << AT91C_ID_UHPHS) in PMC_PCER
 *      register.
 *   2) Select PLLACK as Input clock of OHCI part, USBS bit in PMC_USB
 *      register.
 *   3) Program the OHCI clocks (UHP48M and UHP12M) with USBDIV field in
 *      PMC_USB register. USBDIV value is calculated regarding the PLLACK
 *      value and USB Full-speed accuracy.
 *   4) Enable the OHCI clocks, UHP bit in PMC_SCER register.
 *
 * "The USB Host controller requires 48 MHz and 12 MHz clocks for OHCI
 *  full-speed operations. These clocks must be generated by a PLL with a
 *  correct accuracy of ± 0.25% thanks to USBDIV field.
 *
 * "Thus the USB Host peripheral receives three clocks from the Power
 *  Management Controller (PMC): the Peripheral Clock (MCK domain), the
 *  UHP48M and the UHP12M (built-in UHP48M divided by four) used by the
 *  OHCI to interface with the bus USB signals (Recovered 12 MHz domain)
 *  in Full-speed operations"
 *
 *  USB Clock = PLLACK / (USBDIV + 1) = 48MHz
 *  USBDIV    = PLLACK / 48MHz - 1
 *            = 15
 *
 *  The maximum value of USBDIV is 15 corresponding to a divisor of 16.
 *  REVISIT: USBDIV = 15 gives an exact clock of 48MHz.
 */

#define BOARD_OHCI_INPUT           PMC_USB_USBS_PLLA
#define BOARD_OHCI_DIVIDER         (15)

/* Resulting frequencies */

#define BOARD_MAINOSC_FREQUENCY    (12000000)  /* MAINOSC: 12MHz crystal on-board */
#define BOARD_PLLA_FREQUENCY       (768000000) /* PLLACK:  64 * 12Mhz / 1 */
#define BOARD_PCK_FREQUENCY        (384000000) /* CPU:     PLLACK / 2 / 1  */
#define BOARD_MCK_FREQUENCY        (128000000) /* MCK:     PLLACK / 2 / 1 / 3 */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV) + CLOCKODD + 2).
 *
 *   MCI_SPEED = MCK / (2*CLKDIV + CLOCKODD + 2)
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 128MHz, CLKDIV = 159, MCI_SPEED = 128MHz / (2*159 + 0 + 2) = 400 KHz */

#define HSMCI_INIT_CLKDIV          (159 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 128MHz, CLKDIV = 2 w/CLOCKODD, MCI_SPEED = 128MHz /(2*2 + 1 + 2) = 18.3 MHz */

#define HSMCI_MMCXFR_CLKDIV        ((2 << HSMCI_MR_CLKDIV_SHIFT) | HSMCI_MR_CLKODD)

/* MCK = 128MHz, CLKDIV = 2, MCI_SPEED = 128MHz /(2*2 + 0 + 2) = 21.3 MHz */

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
#endif  /* __CONFIGS_SAMA5D3X_EK_INCLUDE_BOARD_384MHZ_H */
