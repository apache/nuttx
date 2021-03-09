/************************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_wwdt.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_WWDT_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_WWDT_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

/* Register offsets *****************************************************************/

#define LPC54_WWDT_MOD_OFFSET       0x0000  /* Watchdog mode register */
#define LPC54_WWDT_TC_OFFSET        0x0004  /* Watchdog timer constant register */
#define LPC54_WWDT_FEED_OFFSET      0x0008  /* Watchdog feed sequence register */
#define LPC54_WWDT_TV_OFFSET        0x000c  /* Watchdog timer value register */
#define LPC54_WWDT_WARNINT_OFFSET   0x0014  /* Watchdog warning interrupt register */
#define LPC54_WWDT_WINDOW_OFFSET    0x0018  /* Watchdog timer window register */

/* Register addresses ***************************************************************/

#define LPC54_WWDT_MOD              (LPC54_WWDT_BASE+LPC54_WWDT_MOD_OFFSET)
#define LPC54_WWDT_TC               (LPC54_WWDT_BASE+LPC54_WWDT_TC_OFFSET)
#define LPC54_WWDT_FEED             (LPC54_WWDT_BASE+LPC54_WWDT_FEED_OFFSET)
#define LPC54_WWDT_TV               (LPC54_WWDT_BASE+LPC54_WWDT_TV_OFFSET)
#define LPC54_WWDT_WDCLKSEL         (LPC54_WWDT_BASE+LPC54_WWDT_WDCLKSEL_OFFSET)
#define LPC54_WWDT_WARNINT          (LPC54_WWDT_BASE+LPC54_WWDT_WARNINT_OFFSET)
#define LPC54_WWDT_WINDOW           (LPC54_WWDT_BASE+LPC54_WWDT_WINDOW_OFFSET)

/* Register bit definitions *********************************************************/

/* Watchdog mode register */

#define WWDT_MOD_WDEN               (1 << 0)   /* Bit 0: Watchdog enable */
#define WWDT_MOD_WDRESET            (1 << 1)   /* Bit 1: Watchdog reset enable */
#define WWDT_MOD_WDTOF              (1 << 2)   /* Bit 2: Watchdog time-out */
#define WWDT_MOD_WDINT              (1 << 3)   /* Bit 3: Watchdog interrupt */
#define WWDT_MOD_WDPROTECT          (1 << 4)   /* Bit 4: Watchdog update mode */
#define WWDT_MOD_LOCK               (1 << 5)   /* Bit 5: Prevent disabling WDT */
                                               /* Bits 6-31: Reserved */
/* Watchdog timer constant register */

#define WWDT_TC_MASK                0x00ffffff /* Bits 0-23: Watchdog time-out value */
                                               /* Bits 24-31: Reserved */
/* Watchdog feed sequence register  */

#define WWDT_FEED_MASK              0xff      /* Bits 0-7: Feed value: 0xaa followed by 0x55 */
                                              /* Bits 14-31: Reserved */
/* Watchdog timer value register */

#define WWDT_TV_MASK                0x00ffffff /* Bits 0-23: Counter timer value */
                                               /* Bits 24-31: Reserved */
/* Watchdog warning interrupt register */

#define WWDT_WARNINT_MASK           0x03ff     /* Bits 0-9: Watchdog warning compare value */
                                               /* Bits 10-31: Reserved */
/* Watchdog timer window register */

#define WWDT_WINDOW_MASK            0x00ffffff /* Bits 0-23: Watchdog window value */
                                               /* Bits 24-31: Reserved */

#endif /* __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_WWDT_H */
