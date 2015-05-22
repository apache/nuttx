/************************************************************************************
 * arch/arm/src/lpc11xx/chip/lpc11_wdt.h
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#ifndef __ARCH_ARM_SRC_LPC11XX_LPC11_WDT_H
#define __ARCH_ARM_SRC_LPC11XX_LPC11_WDT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "chip/lpc11_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC11_WDT_MOD_OFFSET       0x0000  /* Watchdog mode register */
#define LPC11_WDT_TC_OFFSET        0x0004  /* Watchdog timer constant register */
#define LPC11_WDT_FEED_OFFSET      0x0008  /* Watchdog feed sequence register */
#define LPC11_WDT_TV_OFFSET        0x000c  /* Watchdog timer value register */

/* Register addresses ***************************************************************/

#define LPC11_WDT_MOD              (LPC11_WDT_BASE+LPC11_WDT_MOD_OFFSET)
#define LPC11_WDT_TC               (LPC11_WDT_BASE+LPC11_WDT_TC_OFFSET)
#define LPC11_WDT_FEED             (LPC11_WDT_BASE+LPC11_WDT_FEED_OFFSET)
#define LPC11_WDT_TV               (LPC11_WDT_BASE+LPC11_WDT_TV_OFFSET)

/* Register bit definitions *********************************************************/

/* Watchdog mode register */

#define WDT_MOD_WDEN               (1 << 0)  /* Bit 0: Watchdog enable */
#define WDT_MOD_WDRESET            (1 << 1)  /* Bit 1: Watchdog reset enable */
#define WDT_MOD_WDTOF              (1 << 2)  /* Bit 2: Watchdog time-out */
#define WDT_MOD_WDINT              (1 << 3)  /* Bit 3: Watchdog interrupt */
                                             /* Bits 4-31: Reserved */
/* Watchdog timer constant register */

#define WDT_TC                     (0x00ffffff) /* Bits 0-23: Watchdog time-out interval */
                                                /* Bits 24-31: Reserved */

/* Watchdog feed sequence register  */

#define WDT_FEED_MASK              (0xff)    /* Bits 0-7: Feed value should be 0xaa
                                              *           followed by 0x55 */
                                             /* Bits 14-31: Reserved */
/* Watchdog timer value register */

#define WDT_TV                     (0x00ffffff) /* Bits 0-23: Watchdog timer value */
                                                /* Bits 24-31: Reserved */

/************************************************************************************
 * Public Types
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC11XX_LPC11_WDT_H */
