/****************************************************************************
 * arch/mips/src/pic32mx/pic32mx_wdt.h
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

#ifndef __ARCH_MIPS_SRC_PIC32MX_PIC32MX_WDT_H
#define __ARCH_MIPS_SRC_PIC32MX_PIC32MX_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MX_WDT_CON_OFFSET   0x0000 /* Watchdog timer control register */

/* See also the WDTO, SLEEP, and IDLE bits in the resets RCON register */

/* Register Addresses *******************************************************/

#define PIC32MX_WDT_CON          (PIC32MX_WDT_K1BASE+PIC32MX_WDT_CON_OFFSET)

/* Register Bit-Field Definitions *******************************************/

/* Watchdog timer control register */

#define WDT_CON_WDTCLR           (1 << 0)  /* Bit 0:  Watchdog timer reset */
#define WDT_CON_SWDTPS_SHIFT     (2)       /* Bits 2-6: WDT postscaler value from DEVCFG1 */
#define WDT_CON_SWDTPS_MASK      (31 << WDT_CON_SWDTPS_SHIFT)
#  define WDT_CON_SWDTPS_1       (0 << WDT_CON_SWDTPS_SHIFT)  /* 1:1 */
#  define WDT_CON_SWDTPS_2       (1 << WDT_CON_SWDTPS_SHIFT)  /* 1:2 */
#  define WDT_CON_SWDTPS_4       (2 << WDT_CON_SWDTPS_SHIFT)  /* 1:4 */
#  define WDT_CON_SWDTPS_8       (3 << WDT_CON_SWDTPS_SHIFT)  /* 1:8 */
#  define WDT_CON_SWDTPS_16      (4 << WDT_CON_SWDTPS_SHIFT)  /* 1:16 */
#  define WDT_CON_SWDTPS_32      (5 << WDT_CON_SWDTPS_SHIFT)  /* 1:32 */
#  define WDT_CON_SWDTPS_64      (6 << WDT_CON_SWDTPS_SHIFT)  /* 1:64 */
#  define WDT_CON_SWDTPS_128     (7 << WDT_CON_SWDTPS_SHIFT)  /* 1:128 */
#  define WDT_CON_SWDTPS_256     (8 << WDT_CON_SWDTPS_SHIFT)  /* 1:256 */
#  define WDT_CON_SWDTPS_512     (9 << WDT_CON_SWDTPS_SHIFT)  /* 1:512 */
#  define WDT_CON_SWDTPS_1024    (10 << WDT_CON_SWDTPS_SHIFT) /* 1:1024 */
#  define WDT_CON_SWDTPS_2048    (11 << WDT_CON_SWDTPS_SHIFT) /* 1:2048 */
#  define WDT_CON_SWDTPS_4096    (12 << WDT_CON_SWDTPS_SHIFT) /* 1:4096 */
#  define WDT_CON_SWDTPS_8192    (13 << WDT_CON_SWDTPS_SHIFT) /* 1:8192 */
#  define WDT_CON_SWDTPS_16384   (14 << WDT_CON_SWDTPS_SHIFT) /* 1:16384 */
#  define WDT_CON_SWDTPS_32768   (15 << WDT_CON_SWDTPS_SHIFT) /* 1:32768 */
#  define WDT_CON_SWDTPS_65536   (16 << WDT_CON_SWDTPS_SHIFT) /* 1:65536 */
#  define WDT_CON_SWDTPS_131072  (17 << WDT_CON_SWDTPS_SHIFT) /* 1:131072 */
#  define WDT_CON_SWDTPS_262144  (18 << WDT_CON_SWDTPS_SHIFT) /* 1:262144 */
#  define WDT_CON_SWDTPS_524288  (19 << WDT_CON_SWDTPS_SHIFT) /* 1:524288 */
#  define WDT_CON_SWDTPS_1048576 (20 << WDT_CON_SWDTPS_SHIFT) /* 1:1048576 */

#define WDT_CON_ON               (1 << 15  /* Bit 15: Watchdog timer enable */

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
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_MIPS_SRC_PIC32MX_PIC32MX_WDT_H */
