/************************************************************************************
 * configs/xmc4500-relax/include/board.h
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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

#ifndef __CONFIG_XMC4500_RELAX_INCLUDE_BOARD_H
#define __CONFIG_XMC4500_RELAX_INCLUDE_BOARD_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Clocking *************************************************************************/
/* Default clock initialization
 * fPLL = 288MHz => fSYS = 288MHz => fCPU = 144MHz
 *                                => fPB  = 144MHz
 *                                => fCCU = 144MHz
 *                                => fETH = 72MHz
 *               => fUSB = 48MHz
 *               => fEBU = 72MHz
 *
 * fUSBPLL Disabled, only enabled if SCU_CLK_USBCLKCR_USBSEL_USBPLL is selected
 *
 * fOFI = 24MHz => fWDT = 24MHz
 */

/* On-board crystals
 *
 *   NOTE: Only the XMC4500 Relax Kit-V1 provides the 32.768KHz RTC crystal.  It
 *   is not available on XMC4500 Relax Lite Kit-V1.
 */

#define BOARD_XTAL_FREQUENCY      12000000 /* 12MHz XTAL */
#undef  BOARD_RTC_XTAL_FRQUENCY            /* 32.768KHz RTC XTAL not available */

/* Select the external crystal as the PLL clock source */

#define BOARD_PLL_CLOCKSRC_XTAL   1        /* PLL Clock source == extnernal crystal */
#undef BOARD_PLL_CLOCKSRC_OFI              /* PLL Clock source != internal fast oscillator */

/* PLL Configuration:
 *
 * fPLL = (fPLLSRC / (pdiv * k2div) * ndiv
 *
 * fPLL = (12000000 / (2 * 1)) * 48
 *      = 288MHz
 */

#define BOARD_PLL_PDIV            2
#define BOARD_PLL_NDIV            48
#define BOARD_PLL_K2DIV           1
#define BOARD_PLL_FREQUENCY       288000000

/* System frequency is divided down from PLL output */

#define BOARD_SYSDIV              1        /* PLL Output divider to get fSYS */
#define BOARD_SYS_FREQUENCY       288000000

/* CPU frequency may be divided down from system frequency */

#define BOARD_CPUDIV_ENABLE       1        /* Enable PLL dive by 2 for fCPU */
#define BOARD_CPU_FREQUENCY       144000000

/* Standby clock source selection
 *
 * BOARD_STDBY_CLOCKSRC_OSI    - Internal slow oscillator (32768Hz)
 * BOARD_STDBY_CLOCKSRC_OSCULP - External 32.768KHz crystal
 */

#define BOARD_STDBY_CLOCKSRC_OSI   1
#undef BOARD_STDBY_CLOCKSRC_OSCULP

/* USB PLL settings.
 *
 *   fUSBPLL = 48MHz and fUSBPLLVCO = 384 MHz
 *
 * Note: Implicit divider of 2 and fUSBPLLVCO >= 260 MHz and
 * fUSBPLLVCO <= 520 MHz
 */

#define BOARD_USB_PDIV            2
#define BOARD_USB_NDIV            64

/************************************************************************************
 * Public Data
 ************************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
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

#endif /* __ASSEMBLY__ */
#endif  /* __CONFIG_XMC4500_RELAX_INCLUDE_BOARD_H */
