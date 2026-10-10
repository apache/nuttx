/****************************************************************************
 * arch/mips/src/pic32mz/hardware/pic32mz_wdt.h
 *
 * SPDX-License-Identifier: Apache-2.0
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

#ifndef __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_WDT_H
#define __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_WDT_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "pic32mz_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PIC32MZ_WDT_CON_OFFSET     0x0000 /* Watchdog timer control */
#define PIC32MZ_WDT_CONCLR_OFFSET  0x0004 /* Watchdog timer control clear */
#define PIC32MZ_WDT_CONSET_OFFSET  0x0008 /* Watchdog timer control set */
#define PIC32MZ_WDT_CONINV_OFFSET  0x000c /* Watchdog timer control invert */

/* Register Addresses *******************************************************/

#define PIC32MZ_WDT_CON            (PIC32MZ_WDT_K1BASE + PIC32MZ_WDT_CON_OFFSET)
#define PIC32MZ_WDT_CONCLR         (PIC32MZ_WDT_K1BASE + PIC32MZ_WDT_CONCLR_OFFSET)
#define PIC32MZ_WDT_CONSET         (PIC32MZ_WDT_K1BASE + PIC32MZ_WDT_CONSET_OFFSET)
#define PIC32MZ_WDT_CONINV         (PIC32MZ_WDT_K1BASE + PIC32MZ_WDT_CONINV_OFFSET)

/* The clear key is the upper half-word of WDTCON.  It must be written with
 * a single 16-bit store to WDTCON + 2.
 */

#define PIC32MZ_WDT_CLRKEY         (PIC32MZ_WDT_K1BASE + 2)

/* Watchdog Timer Control Register Bit Definitions **************************/

#define WDT_CON_WDTWINEN           (1 << 0)  /* Bit 0: Windowed mode enable */
#define WDT_CON_RUNDIV_SHIFT       (8)       /* Bits 8-12: Run-mode postscaler */
#define WDT_CON_RUNDIV_MASK        (0x1f << WDT_CON_RUNDIV_SHIFT)
#define WDT_CON_ON                 (1 << 15) /* Bit 15: Watchdog enable */
#define WDT_CON_CLRKEY_SHIFT       (16)      /* Bits 16-31: Clear key */
#define WDT_CON_CLRKEY_MASK        (0xffff << WDT_CON_CLRKEY_SHIFT)

#define WDT_CLRKEY_VALUE           0x5743

/* RUNDIV (loaded from CFGCON2.WDTPS at reset) selects a postscaler of
 * 2^RUNDIV; the valid values are 0..20 (1:1 to 1:1048576).
 */

#define WDT_RUNDIV_MAX             20

#endif /* __ARCH_MIPS_SRC_PIC32MZ_HARDWARE_PIC32MZ_WDT_H */
