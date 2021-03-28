/****************************************************************************
 * arch/arm/src/lpc54xx/hardware/lpc54_wwdt.h
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

#ifndef __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_WWDT_H
#define __ARCH_ARM_SRC_LPC54XX_HARDWARE_LPC54_WWDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/lpc54_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC54_WWDT_MOD_OFFSET       0x0000  /* Watchdog mode register */
#define LPC54_WWDT_TC_OFFSET        0x0004  /* Watchdog timer constant register */
#define LPC54_WWDT_FEED_OFFSET      0x0008  /* Watchdog feed sequence register */
#define LPC54_WWDT_TV_OFFSET        0x000c  /* Watchdog timer value register */
#define LPC54_WWDT_WARNINT_OFFSET   0x0014  /* Watchdog warning interrupt register */
#define LPC54_WWDT_WINDOW_OFFSET    0x0018  /* Watchdog timer window register */

/* Register addresses *******************************************************/

#define LPC54_WWDT_MOD              (LPC54_WWDT_BASE+LPC54_WWDT_MOD_OFFSET)
#define LPC54_WWDT_TC               (LPC54_WWDT_BASE+LPC54_WWDT_TC_OFFSET)
#define LPC54_WWDT_FEED             (LPC54_WWDT_BASE+LPC54_WWDT_FEED_OFFSET)
#define LPC54_WWDT_TV               (LPC54_WWDT_BASE+LPC54_WWDT_TV_OFFSET)
#define LPC54_WWDT_WDCLKSEL         (LPC54_WWDT_BASE+LPC54_WWDT_WDCLKSEL_OFFSET)
#define LPC54_WWDT_WARNINT          (LPC54_WWDT_BASE+LPC54_WWDT_WARNINT_OFFSET)
#define LPC54_WWDT_WINDOW           (LPC54_WWDT_BASE+LPC54_WWDT_WINDOW_OFFSET)

/* Register bit definitions *************************************************/

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
