/****************************************************************************
 * arch/arm/src/lpc17xx_40xx/hardware/lpc17_40_wdt.h
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

#ifndef __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_WDT_H
#define __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "chip.h"
#include "hardware/lpc17_40_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define LPC17_40_WDT_MOD_OFFSET       0x0000  /* Watchdog mode register */
#define LPC17_40_WDT_TC_OFFSET        0x0004  /* Watchdog timer constant register */
#define LPC17_40_WDT_FEED_OFFSET      0x0008  /* Watchdog feed sequence register */
#define LPC17_40_WDT_TV_OFFSET        0x000c  /* Watchdog timer value register */

#ifdef LPC176x
#  define LPC17_40_WDT_CLKSEL_OFFSET  0x0010  /* Watchdog clock source selection register */
#endif

#ifdef LPC178x_40xx
#  define LPC17_40_WDT_WARNINT_OFFSET 0x0014  /* Watchdog warning interrupt */
#  define LPC17_40_WDT_WINDOW_OFFSET  0x0018  /* Watchdog window compare value */
#endif

/* Register addresses *******************************************************/

#define LPC17_40_WDT_MOD              (LPC17_40_WDT_BASE+LPC17_40_WDT_MOD_OFFSET)
#define LPC17_40_WDT_TC               (LPC17_40_WDT_BASE+LPC17_40_WDT_TC_OFFSET)
#define LPC17_40_WDT_FEED             (LPC17_40_WDT_BASE+LPC17_40_WDT_FEED_OFFSET)
#define LPC17_40_WDT_TV               (LPC17_40_WDT_BASE+LPC17_40_WDT_TV_OFFSET)
#define LPC17_40_WDT_CLKSEL           (LPC17_40_WDT_BASE+LPC17_40_WDT_CLKSEL_OFFSET)

#ifdef LPC178x_40xx
#  define LPC17_40_WDT_WARNINT        (LPC17_40_WDT_BASE+LPC17_40_WDT_WARNINT_OFFSET)
#  define LPC17_40_WDT_WINDOW         (LPC17_40_WDT_BASE+LPC17_40_WDT_WINDOW_OFFSET)
#endif

/* Register bit definitions *************************************************/

/* Watchdog mode register */

#define WDT_MOD_WDEN                  (1 << 0)  /* Bit 0: Watchdog enable */
#define WDT_MOD_WDRESET               (1 << 1)  /* Bit 1: Watchdog reset enable */
#define WDT_MOD_WDTOF                 (1 << 2)  /* Bit 2: Watchdog time-out */
#define WDT_MOD_WDINT                 (1 << 3)  /* Bit 3: Watchdog interrupt */
#ifdef LPC178x_40xx
#  define WDT_MOD_WDPROTECT           (1 << 4)  /* Bit 4: Watchdog interrupt */
#endif
                                                /* Bits 5-31: Reserved */

/* Watchdog timer constant register */

#ifdef LPC176x
#  define WDT_TC                      (0xffffffff) /* Bits 0-31: Watchdog time-out interval */
#endif
#ifdef LPC178x_40xx
#  define WDT_TC                      (0x00ffffff) /* Bits 0-23: Watchdog time-out interval */

                                                /* Bits 24-31: Reserved */
#endif

/* Watchdog feed sequence register  */

#define WDT_FEED_KEY_1                (0xaa)    /* Watchdog feed key 1 */
#define WDT_FEED_KEY_2                (0x55)    /* Watchdog feed key 2 */
#define WDT_FEED_MASK                 (0xff)    /* Bits 0-7: Feed value should be 0xaa
                                                 *           followed by 0x55 */
                                                /* Bits 14-31: Reserved */

/* Watchdog timer value register */

#ifdef LPC176x
#  define WDT_TVT                     (0xffffffff) /* Bits 0-31: Watchdog timer value */
#endif
#ifdef LPC178x_40xx
#  define WDT_TVT                     (0xffffff) /* Bits 0-23: Watchdog timer value */
                                                 /* Bits 24-31: Reserved */
#endif

/* Watchdog clock source selection register */

#ifdef LPC176x
#  define WDT_CLKSEL_WDSEL_SHIFT      (0)       /* Bits 0-1: Clock source for the Watchdog timer */
#  define WDT_CLKSEL_WDSEL_MASK       (3 << WDT_CLKSEL_WDSEL_SHIFT)
#    define WDT_CLKSEL_WDSEL_INTRC    (0 << WDT_CLKSEL_WDSEL_SHIFT) /* Internal RC osc */
#    define WDT_CLKSEL_WDSEL_APB      (1 << WDT_CLKSEL_WDSEL_SHIFT) /* APB peripheral clock (watchdog pclk) */
#    define WDT_CLKSEL_WDSEL_RTC      (2 << WDT_CLKSEL_WDSEL_SHIFT) /* RTC oscillator (rtc_clk) */

                                                /* Bits 2-30: Reserved */
#  define WDT_CLKSEL_WDLOCK           (1 << 31) /* Bit 31: Lock WDT register bits if set */
#endif

/* Watchdog timer warning interrupt register */

#ifdef LPC178x_40xx
#  define WDT_WARNINT                 (0x3ff)   /* Bits 0-9: Warning Interrupt compare value */
                                                /* Bits 10-31: Reserved */
#endif

/* Watchdog timer value register */

#ifdef LPC178x_40xx
#  define WDT_WINDOW                  (0xffffff) /* Bits 0-23: Watchdog window value */
                                                 /* Bits 24-31: Reserved */
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#endif /* __ARCH_ARM_SRC_LPC17XX_40XX_HARDWARE_LPC17_40_WDT_H */
