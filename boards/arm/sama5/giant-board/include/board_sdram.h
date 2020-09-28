/****************************************************************************
 * boards/arm/sama5/giant-board/include/board_sdram.h
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

#ifndef __BOARDS_ARM_SAMA5_GIANT_BOARD_INCLUDE_BOARD_SDRAM_H
#define __BOARDS_ARM_SAMA5_GIANT_BOARD_INCLUDE_BOARD_SDRAM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "sam_pmc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the SAMA5 device is running on a 24MHz internal RC.
 * When booting from SDRAM, NuttX is loaded in SDRAM by an intermediate
 * bootloader.  That bootloader had to have already configured the PLL
 * and SDRAM for proper operation.
 *
 * In this case, we do not reconfigure the clocking.
 * Rather, we need to query the register settings to determine the clock
 * frequencies.
 * We can only assume that the Main clock source is the on-board 12MHz
 * crystal.
 */

#define BOARD_MAINCK_FREQUENCY     BOARD_MAINOSC_FREQUENCY
#define BOARD_PLLA_FREQUENCY       (sam_pllack_frequency(BOARD_MAINOSC_FREQUENCY))
#define BOARD_PLLADIV2_FREQUENCY   (sam_plladiv2_frequency(BOARD_MAINOSC_FREQUENCY))
#define BOARD_PCK_FREQUENCY        (sam_pck_frequency(BOARD_MAINOSC_FREQUENCY))
#define BOARD_MCK_FREQUENCY        (sam_mck_frequency(BOARD_MAINOSC_FREQUENCY))

/* Clocking to certain peripherals may be MCK/2.
 *
 * REVISIT:  I am not sure why this is.  Perhaps because of H32MXDIV?
 */

#define BOARD_PIT_FREQUENCY        (BOARD_MCK_FREQUENCY >> 1)
#define BOARD_USART_FREQUENCY      (BOARD_MCK_FREQUENCY >> 1)
#define BOARD_FLEXCOM_FREQUENCY    (BOARD_MCK_FREQUENCY >> 1)

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
#  define BOARD_UPLL_OHCI_DIV        (10)  /* Divide by 10 */
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
#define BOARD_TSD_TRACKTIM         (2000)      /* Min 1�s at 8MHz */
#define BOARD_TSD_DEBOUNCE         (10000000)  /* 10 milliseconds (units nanoseconds) */

/* SDMMC clocking
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

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* !__ASSEMBLY__ */
#endif /* __BOARDS_ARM_SAMA5_GIANT_BOARD_INCLUDE_BOARD_SDRAM_H */
