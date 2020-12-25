/****************************************************************************
 * boards/arm/sama5/sama5d2-xult/include/board_498mhz.h
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

#ifndef __BOARDS_ARM_SAMA5_SAMA5D2_XULT_INCLUDE_BOARD_498MHZ_H
#define __BOARDS_ARM_SAMA5_SAMA5D2_XULT_INCLUDE_BOARD_498MHZ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* After power-on reset, the SAMA5 device is running on a 12MHz internal RC.
 * These definitions will configure operational clocking.
 *
 * This is the configuration results in a CPU clock of 498MHz:
 *
 * MAINOSC:  Frequency = 12MHz (crystal)
 * PLLA: PLL Multiplier = 43+1 to generate PLLACK = 498MHz
 * Master Clock (MCK): Source = PLLACK/1, Prescalar = 1, MDIV = 4 to generate
 *     MCK      =  166MHz
 *     CPU clock = 498MHz
 */

/* Main oscillator register settings.
 *
 *   The start up time should be should be:
 *   Start Up Time = 8 * MOSCXTST / SLCK = 56 Slow Clock Cycles.
 */

#define BOARD_CKGR_MOR_MOSCXTST    (62 << PMC_CKGR_MOR_MOSCXTST_SHIFT) /* Start-up Time */

/* PLLA configuration.
 *
 *   Multipler = 43+1: PLLACK = 44 * 12MHz = 498MHz
 */

#define BOARD_CKGR_PLLAR_COUNT     (63 << PMC_CKGR_PLLAR_COUNT_SHIFT)
#define BOARD_CKGR_PLLAR_OUT       (0)
#define BOARD_CKGR_PLLAR_MUL       (43 << PMC_CKGR_PLLAR_MUL_SHIFT)

/* PMC master clock register settings.
 *
 *  Master/Processor Clock Source Selection = PLLA
 *  Master/Processor Clock Prescaler        = 1
 *  PLLA Divider                            = 1
 *  Master Clock Division (MDIV)            = 4
 *
 *  NOTE: Bit PLLADIV2 must always be set to 1 when MDIV is set to 3.
 *
 *  Prescaler input                         = 498MHz / 1 = 498MHz
 *  Prescaler output                        = 498MHz / 1 = 498MHz
 *  Processor Clock (PCK)                   = 498MHz
 *  Master clock (MCK)                      = 498MHz / 4 = 132MHz
 */

#define BOARD_PMC_MCKR_CSS         PMC_MCKR_CSS_PLLA
#define BOARD_PMC_MCKR_PRES        PMC_MCKR_PRES_DIV1
#define BOARD_PMC_MCKR_PLLADIV     PMC_MCKR_PLLADIV1
#define BOARD_PMC_MCKR_MDIV        PMC_MCKR_MDIV_PCKDIV4

/* ADC Configuration
 *
 * ADCClock = MCK / ((PRESCAL+1) * 2)
 *
 * Given:
 *   MCK      = 132MHz
 *   ADCClock = 8MHz
 * Then:
 *   PRESCAL  = 7.25
 *
 * PRESCAL=7 and MCK=132MHz yields ADC clock of 8.25MHz
 */

#define BOARD_ADC_PRESCAL          (7)
#define BOARD_TSD_STARTUP          (40)        /* 40 nanoseconds */
#define BOARD_TSD_TRACKTIM         (2000)      /* Min 1us at 8MHz */
#define BOARD_TSD_DEBOUNCE         (10000000)  /* 10 milliseconds (units nanoseconds) */

/* Resulting frequencies */

#define BOARD_MAINCK_FREQUENCY     BOARD_MAINOSC_FREQUENCY
#define BOARD_PLLA_FREQUENCY       (996000000) /* PLLACK:  83 * 12Mhz / 1 */
#define BOARD_PCK_FREQUENCY        (498000000) /* CPU:     PLLACK / 2 / 1  */
#define BOARD_MCK_FREQUENCY        (166000000) /* MCK:     PLLACK / 1 / 1 / 3 */
#define BOARD_ADCCLK_FREQUENCY     (83000000)  /* ADCCLK:  MCK / ((7+1)*2) */

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

/* HSMCI clocking
 *
 * Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
 * divided by (2*(CLKDIV) + CLOCKODD + 2).
 *
 *   MCI_SPEED = MCK / (2*CLKDIV + CLOCKODD + 2)
 *
 * Where CLKDIV has a range of 0-255.
 */

/* MCK = 132MHz, CLKDIV = 164,
 * MCI_SPEED = 132MHz / (2*164 + 0 + 2) = 400 KHz
 */

#define HSMCI_INIT_CLKDIV          (164 << HSMCI_MR_CLKDIV_SHIFT)

/* MCK = 132MHz, CLKDIV = 2 w/CLOCKODD,
 * MCI_SPEED = 132MHz /(2*2 + 1 + 2) = 18.9 MHz
 */

#define HSMCI_MMCXFR_CLKDIV        ((2 << HSMCI_MR_CLKDIV_SHIFT) | HSMCI_MR_CLKODD)

/* MCK = 132MHz, CLKDIV = 2, MCI_SPEED = 132MHz /(2*2 + 0 + 2) = 22 MHz */

#define HSMCI_SDXFR_CLKDIV         (2 << HSMCI_MR_CLKDIV_SHIFT)
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
#endif  /* __BOARDS_ARM_SAMA5_SAMA5D2_XULT_INCLUDE_BOARD_498MHZ_H */
