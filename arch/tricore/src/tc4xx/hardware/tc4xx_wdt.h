/****************************************************************************
 * arch/tricore/src/tc4xx/hardware/tc4xx_wdt.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_WDT_H
#define __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_WDT_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* WTU watchdog timers: one per CPU and the system watchdog */

#define TC4XX_WTU_WDTCPU_CTRLA(n)  (0xf000003c + (n) * 0x30)
#define TC4XX_WTU_WDTCPU_CTRLB(n)  (0xf0000040 + (n) * 0x30)
#define TC4XX_WTU_WDTSYS_CTRLA     0xf00001a8
#define TC4XX_WTU_WDTSYS_CTRLB     0xf00001ac
#define TC4XX_WTU_STAT_OFFSET      8                 /* From CTRLA */

#define TC4XX_WTU_CTRLA_LCK        (1 << 0)          /* Lock */
#define TC4XX_WTU_CTRLA_PW_SHIFT   1                 /* Password */
#define TC4XX_WTU_CTRLA_PW_MASK    (0x7fff << TC4XX_WTU_CTRLA_PW_SHIFT)
#define TC4XX_WTU_CTRLA_TCVI_SHIFT 16                /* Timer check value */
#define TC4XX_WTU_CTRLB_DR         (1 << 0)          /* Disable request */
#define TC4XX_WTU_STAT_TC          (1 << 8)          /* Timer check */
#define TC4XX_WTU_STAT_TIM_SHIFT   16                /* Timer value */

/* The low seven password bits read inverted */

#define TC4XX_WTU_PW_INVERTED      0x7f

#endif /* __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_WDT_H */
