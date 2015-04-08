/************************************************************************************
 * configs/sama5d3-xplained/include/board_sdram.h
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIGS_SAMA5D3_XPLAINED_INCLUDE_BOARD_SDRAM_H
#define __CONFIGS_SAMA5D3_XPLAINED_INCLUDE_BOARD_SDRAM_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "sam_pmc.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* After power-on reset, the SAMA5 device is running on a 12MHz internal RC.  When
 * booting from SDRAM, NuttX is loaded in SDRAM by an intermediate bootloader.  That
 * bootloader had to have already configured the PLL and SDRAM for proper operation.
 *
 * In this case, we do not reconfigure the clocking.  Rather, we need to query
 * the register settings to determine the clock frequencies.  We can only assume that
 * the Main clock source is the on-board 12MHz crystal.
 */

#define BOARD_MAINCK_FREQUENCY     BOARD_MAINOSC_FREQUENCY
#define BOARD_PLLA_FREQUENCY       (sam_pllack_frequency(BOARD_MAINOSC_FREQUENCY))
#define BOARD_PLLADIV2_FREQUENCY   (sam_plladiv2_frequency(BOARD_MAINOSC_FREQUENCY))
#define BOARD_PCK_FREQUENCY        (sam_pck_frequency(BOARD_MAINOSC_FREQUENCY))
#define BOARD_MCK_FREQUENCY        (sam_mck_frequency(BOARD_MAINOSC_FREQUENCY))

/* On some SAMA5's, the clocking to peripherals may be divided down from MCK,
 * but not for the SAMA5D3.
 */

#define BOARD_PIT_FREQUENCY        BOARD_MCK_FREQUENCY
#define BOARD_USART_FREQUENCY      BOARD_MCK_FREQUENCY

#if defined(CONFIG_SAMA5_EHCI) || defined(CONFIG_SAMA5_OHCI) || \
    defined(CONFIG_SAMA5_UDPHS)

/* The USB Host High Speed requires a 480 MHz clock (UPLLCK) for the embedded
 * High-speed transceivers. UPLLCK is the output of the 480 MHz UTMI PLL
 * (UPLL).  The source clock of the UTMI PLL is the Main OSC output:  Either
 * the 12MHz internal RC oscillator on a an external 12MHz crystal.  The
 * Main OSC must be 12MHz because the UPLL has a built-in 40x multiplier.
 *
 * For High-speed operations, the user has to perform the following:
 *
 *   1) Enable UHP peripheral clock, bit (1 << AT91C_ID_UHPHS) in
 *      PMC_PCER register.
 *   2) Write CKGR_PLLCOUNT field in PMC_UCKR register.
 *   3) Enable UPLL, bit AT91C_CKGR_UPLLEN in PMC_UCKR register.
 *   4) Wait until UTMI_PLL is locked. LOCKU bit in PMC_SR register
 *   5) Enable BIAS, bit AT91C_CKGR_BIASEN in PMC_UCKR register.
 *   6) Select UPLLCK as Input clock of OHCI part, USBS bit in PMC_USB
 *      register.
 *   7) Program the OHCI clocks (UHP48M and UHP12M) with USBDIV field in
 *      PMC_USB register. USBDIV must be 9 (division by 10) if UPLLCK is
 *      selected.
 *   8) Enable OHCI clocks, UHP bit in PMC_SCER register.
 *
 * Steps 2 through 7 performed here.  1 and 8 are performed in the EHCI
 * driver is initialized.
 */

#  define BOARD_USE_UPLL             1     /* Use UPLL for clock source */
#  define BOARD_CKGR_UCKR_UPLLCOUNT  (15)  /* Maximum value */
#  define BOARD_CKGR_UCKR_BIASCOUNT  (15)  /* Maximum value */

/* REVISIT:  The divisor of 10 produces a rate that is too high. Division
 * by 5, however, seems to work just fine.  No idea why?
 */

#  define BOARD_UPLL_OHCI_DIV        (5)   /* Divide by 5 */
#endif

/* ADC Configuration
 *
 * ADCClock = MCK / ((PRESCAL+1) * 2)
 * PRESCAL  = (MCK / (2 * ADCClock) - 1)
 */

#define BOARD_ADCCLK_FREQUENCY     (8000000)   /* ADCCLK:  MCK / ((7+1)*2) */
#define BOARD_ADCCLK_FREQUENCY \
  ((BOARD_PLLADIV2_FREQUENCY / (2 *BOARD_PLLADIV2_FREQUENCY)) - 1)

#define BOARD_ADC_PRESCAL          (7)
#define BOARD_TSD_STARTUP          (40)        /* 40 nanoseconds */
#define BOARD_TSD_TRACKTIM         (2000)      /* Min 1µs at 8MHz */
#define BOARD_TSD_DEBOUNCE         (10000000)  /* 10 milliseconds (units nanoseconds) */

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV) + CLOCKODD + 2).
 *
 *   CLKFULLDIV = 2*CLKDIV + CLOCKODD;
 *   MCI_SPEED  = MCK / (CLKFULLDIV + 2)
 *   CLKFULLDIV = MCK / MCI_SPEED - 2
 *
 *   CLKDIV     = CLKFULLDIV >> 1
 *   CLOCKODD   = CLKFULLDIV & 1
 *
 * Where CLKDIV has a range of 0-255.
 */

/* Initial clock: 400 KHz (target) */

#define HSMCI_INIT_CLKDIV          sam_hsmci_clkdiv(400000)

/* MMC transfer clock: 20 MHz (target) */

#define HSMCI_MMCXFR_CLKDIV        sam_hsmci_clkdiv(20000000)

/* SD transfer clock: 25 MHz (target) */

#define HSMCI_SDXFR_CLKDIV         sam_hsmci_clkdiv(25000000)

#define HSMCI_SDWIDEXFR_CLKDIV     HSMCI_SDXFR_CLKDIV

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
#endif  /* __CONFIGS_SAMA5D3_XPLAINED_INCLUDE_BOARD_SDRAM_H */
