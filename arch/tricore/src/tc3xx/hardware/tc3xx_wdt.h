/****************************************************************************
 * arch/tricore/src/tc3xx/hardware/tc3xx_wdt.h
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

#ifndef __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_WDT_H
#define __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_WDT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SCU watchdog timers: one per CPU and the safety watchdog */

#define TC3XX_SCU_WDTCPU_CON0(n)   (0xf003624c + (n) * 0xc)
#define TC3XX_SCU_WDTCPU_CON1(n)   (0xf0036250 + (n) * 0xc)
#define TC3XX_SCU_WDTS_CON0        0xf00362a8
#define TC3XX_SCU_WDTS_CON1        0xf00362ac

#define TC3XX_WDT_CON0_ENDINIT     (1 << 0)          /* End of init */
#define TC3XX_WDT_CON0_LCK         (1 << 1)          /* Lock */
#define TC3XX_WDT_CON0_PW_SHIFT    2                 /* Password */
#define TC3XX_WDT_CON0_PW_MASK     (0x3fff << TC3XX_WDT_CON0_PW_SHIFT)
#define TC3XX_WDT_CON0_REL_MASK    (0xffff << 16)    /* Reload value */
#define TC3XX_WDT_CON1_DR          (1 << 3)          /* Disable request */

/* The low six password bits read inverted */

#define TC3XX_WDT_PW_INVERTED      0x3f

#endif /* __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_WDT_H */
