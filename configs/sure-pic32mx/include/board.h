/****************************************************************************
 * configs/sure-pic32mx/include/board.h
 * include/arch/board/board.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#ifndef __CONFIGS_SURE_PIC32MX_INCLUDE_BOARD_H
#define __CONFIGS_SURE_PIC32MX_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

/* Clocking *****************************************************************/
/* Crystal frequencies */

#define BOARD_POSC_FREQ        20000000  /* Primary OSC XTAL frequency (20MHz) */
#define BOARD_SOSC_FREQ        32768     /* Secondary OSC XTAL frequency (32.768KHz) */

/* PLL configuration and resulting CPU clock.
 * CPU_CLOCK = ((POSC_FREQ / IDIV) * MULT) / ODIV
 */

#define BOARD_PLL_IDIV         5         /* PLL input divider */
#define BOARD_PLL_MULT         15        /* PLL multiplier */
#define BOARD_PLL_ODIV         1         /* PLL output divider */

#define BOARD_CPU_CLOCK        60000000 /* CPU clock (60MHz = (20MHz / 5) * 15 / 1) */

/* USB PLL configuration.
 * USB_CLOCK = ((POSC_XTAL / IDIV) * 24) / 2
 */

#define BOARD_UPLL_IDIV        5         /* USB PLL divider */
#define BOARD_USB_CLOCK        48000000  /* USB clock ((20MHz / 5) * 24) */

/* Peripheral clock is *not* divided down from CPU clock.
 * PBCLOCK = CPU_CLOCK / PBDIV
 */

#define BOARD_PBDIV            1        /* Peripheral clock divisor (PBDIV) */
#define BOARD_PBCLOCK          60000000 /* Peripheral clock (PBCLK = 60MHz/1) */

/* Watchdog pre-scaler (re-visit) */

#define BOARD_WD_ENABLE        0        /* Watchdog is disabled */
#define BOARD_WD_PRESCALER     8        /* Watchdog pre-scaler */

/* LED definitions **********************************************************/
/* The Sure PIC32MX board has five LEDs.  One (D4, lablel "Power") is not
 * controllable by software.  Four are controllable by software:
 *
 * D7  "USB"    Yellow  RD7 Low illuminates
 * D8  "SD"     Yellow  RD6 Low illuminates
 * D9  "Flash"  Yellow  RF0 Low illuminates
 * D10 "Error"  Red     RF1 Low illuminates
 */
                                  /* ON                  OFF                 */
                                  /* USB SD  FLASH ERROR USB SD  FLASH ERROR */
#define LED_STARTED            0  /* OFF OFF OFF   OFF   --- --- ---   ---   */
#define LED_HEAPALLOCATE       1  /* ON  OFF N/C   N/C   --- --- ---   ---   */
#define LED_IRQSENABLED        2  /* OFF ON  N/C   N/C   --- --- ---   ---   */
#define LED_STACKCREATED       3  /* ON  ON  N/C   N/C   --- --- ---   ---   */
#define LED_INIRQ              4  /* N/C N/C ON    N/C   N/C N/C OFF   N/C   */
#define LED_SIGNAL             4  /* N/C N/C ON    N/C   N/C N/C OFF   N/C   */
#define LED_ASSERTION          4  /* N/C N/C ON    N/C   N/C N/C OFF   N/C   */
#define LED_PANIC              5  /* N/C N/C N/C   ON    N/C N/C N/C   OFF   */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __CONFIGS_SURE_PIC32MX_INCLUDE_BOARD_H */
