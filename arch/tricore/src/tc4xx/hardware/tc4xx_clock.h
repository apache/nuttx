/****************************************************************************
 * arch/tricore/src/tc4xx/hardware/tc4xx_clock.h
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

#ifndef __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_CLOCK_H
#define __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_CLOCK_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clock module registers */

#define TC4XX_CLOCK_OSCCON         0xf0064100
#define TC4XX_CLOCK_RAMPCON0       0xf0064200
#define TC4XX_CLOCK_RAMPSTAT       0xf006420c
#define TC4XX_CLOCK_SYSPLLCON0     0xf0064300
#define TC4XX_CLOCK_SYSPLLCON1     0xf0064304
#define TC4XX_CLOCK_SYSPLLSTAT     0xf006430c
#define TC4XX_CLOCK_PERPLLCON0     0xf0064380
#define TC4XX_CLOCK_PERPLLCON1     0xf0064384
#define TC4XX_CLOCK_PERPLLSTAT     0xf006438c
#define TC4XX_CLOCK_CCUCON         0xf0064400
#define TC4XX_CLOCK_CCUSTAT        0xf0064404
#define TC4XX_CLOCK_SYSCCUCON0     0xf0064410
#define TC4XX_CLOCK_SYSCCUCON1     0xf0064414
#define TC4XX_CLOCK_PERCCUCON0     0xf0064420
#define TC4XX_CLOCK_PERCCUCON1     0xf0064424
#define TC4XX_CLOCK_OSCMON1        0xf0064508
#define TC4XX_CLOCK_OSCSTAT        0xf006450c
#define TC4XX_CLOCK_RAMPMON        0xf0064510

#define TC4XX_OSCCON_MODE_SHIFT    8
#define TC4XX_OSCCON_MODE_MASK     (0x7 << TC4XX_OSCCON_MODE_SHIFT)
#define TC4XX_OSCCON_INSEL_SHIFT   24
#define TC4XX_OSCCON_INSEL_MASK    (0x3 << TC4XX_OSCCON_INSEL_SHIFT)

#define TC4XX_OSCMON1_OSCVAL_MASK  (0x7f << 0)
#define TC4XX_OSCMON1_SMONEN       (1 << 8)
#define TC4XX_OSCMON1_PMONEN       (1 << 24)
#define TC4XX_OSCSTAT_WDS_MASK     ((0x3 << 0) | (0x3 << 16))

#define TC4XX_RAMPCON0_UFL_MASK    (0x3ff << 0)
#define TC4XX_RAMPCON0_CMD_SHIFT   16
#define TC4XX_RAMPCON0_CMD_MASK    (0x7 << TC4XX_RAMPCON0_CMD_SHIFT)
#define TC4XX_RAMPCON0_PWR         (1 << 19)
#define TC4XX_RAMPSTAT_FSTAT_SHIFT 16
#define TC4XX_RAMPSTAT_FSTAT_MASK  (0x3 << TC4XX_RAMPSTAT_FSTAT_SHIFT)
#define TC4XX_RAMPSTAT_ACTIVE      (1 << 23)
#define TC4XX_RAMPSTAT_FLLLOCK     (1 << 24)
#define TC4XX_RAMPMON_FLLLDEN      (1 << 28)

#define TC4XX_PLLCON0_PLLPWR       (1 << 0)
#define TC4XX_PLLCON0_NDIV_SHIFT   8
#define TC4XX_PLLCON0_NDIV_MASK    (0x7f << TC4XX_PLLCON0_NDIV_SHIFT)
#define TC4XX_PLLCON0_PDIV_SHIFT   16
#define TC4XX_PLLCON0_PDIV_MASK    (0x7 << TC4XX_PLLCON0_PDIV_SHIFT)
#define TC4XX_PLLSTAT_PWRSTAT      (1 << 0)
#define TC4XX_PLLSTAT_PLLLOCK      (1 << 1)

#define TC4XX_PLLCON1_K2DIV_MASK   (0xf << 0)
#define TC4XX_PLLCON1_K2PRE_SHIFT  4
#define TC4XX_PLLCON1_K2PRE_MASK   (0x3 << TC4XX_PLLCON1_K2PRE_SHIFT)
#define TC4XX_PLLCON1_K3DIV_SHIFT  8
#define TC4XX_PLLCON1_K3DIV_MASK   (0x7 << TC4XX_PLLCON1_K3DIV_SHIFT)
#define TC4XX_PLLCON1_K3PRE_SHIFT  12
#define TC4XX_PLLCON1_K3PRE_MASK   (0xf << TC4XX_PLLCON1_K3PRE_SHIFT)
#define TC4XX_PLLCON1_K4DIV_SHIFT  16          /* PERPLLCON1 only */
#define TC4XX_PLLCON1_K4DIV_MASK   (0x7 << TC4XX_PLLCON1_K4DIV_SHIFT)
#define TC4XX_PLLCON1_K4PRE_SHIFT  20          /* PERPLLCON1 only */
#define TC4XX_PLLCON1_K4PRE_MASK   (0x3 << TC4XX_PLLCON1_K4PRE_SHIFT)

#define TC4XX_CCUSTAT_LCK          (1 << 31)
#define TC4XX_CCU_CLKSELS_MASK     (0x3 << 0)  /* CCUCON and CCUSTAT */
#define TC4XX_CCU_CLKSELS_PLL      0
#define TC4XX_CCU_CLKSELS_BACK     1
#define TC4XX_CCU_CLKSELS_RAMP     2
#define TC4XX_CCU_CLKSELP          (1 << 16)   /* Set: backup clock */
#define TC4XX_CCUCON_UP            (1 << 30)   /* SYSCCUCON0 and 1 */

#endif /* __ARCH_TRICORE_SRC_TC4XX_HARDWARE_TC4XX_CLOCK_H */
