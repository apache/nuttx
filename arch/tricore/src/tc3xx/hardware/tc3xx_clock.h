/****************************************************************************
 * arch/tricore/src/tc3xx/hardware/tc3xx_clock.h
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

#ifndef __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_CLOCK_H
#define __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_CLOCK_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SCU clock control unit (CCU) registers */

#define TC3XX_SCU_OSCCON           0xf0036010
#define TC3XX_SCU_SYSPLLSTAT       0xf0036014
#define TC3XX_SCU_SYSPLLCON0       0xf0036018
#define TC3XX_SCU_SYSPLLCON1       0xf003601c
#define TC3XX_SCU_PERPLLSTAT       0xf0036024
#define TC3XX_SCU_PERPLLCON0       0xf0036028
#define TC3XX_SCU_PERPLLCON1       0xf003602c
#define TC3XX_SCU_CCUCON0          0xf0036030
#define TC3XX_SCU_CCUCON1          0xf0036034
#define TC3XX_SCU_CCUCON2          0xf0036040
#define TC3XX_SCU_CCUCON5          0xf003604c
#define TC3XX_SCU_CCUCON6          0xf0036080  /* CCUCON6 to CCUCON11 */

#define TC3XX_OSCCON_PLLLV         (1 << 1)
#define TC3XX_OSCCON_MODE_MASK     (0x3 << 5)
#define TC3XX_OSCCON_PLLHV         (1 << 8)
#define TC3XX_OSCCON_OSCVAL_SHIFT  16
#define TC3XX_OSCCON_OSCVAL_MASK   (0x1f << TC3XX_OSCCON_OSCVAL_SHIFT)

#define TC3XX_PLLCON0_DIVBY        (1 << 0)    /* PERPLLCON0 only */
#define TC3XX_PLLCON0_NDIV_SHIFT   9
#define TC3XX_PLLCON0_NDIV_MASK    (0x7f << TC3XX_PLLCON0_NDIV_SHIFT)
#define TC3XX_PLLCON0_PLLPWD       (1 << 16)
#define TC3XX_PLLCON0_RESLD        (1 << 18)
#define TC3XX_PLLCON0_PDIV_SHIFT   24
#define TC3XX_PLLCON0_PDIV_MASK    (0x7 << TC3XX_PLLCON0_PDIV_SHIFT)
#define TC3XX_PLLCON0_INSEL_SHIFT  30          /* SYSPLLCON0 only */
#define TC3XX_PLLCON0_INSEL_MASK   (0x3 << TC3XX_PLLCON0_INSEL_SHIFT)

#define TC3XX_PLLCON1_K2DIV_MASK   (0x7 << 0)
#define TC3XX_PLLCON1_K3DIV_SHIFT  8           /* PERPLLCON1 only */
#define TC3XX_PLLCON1_K3DIV_MASK   (0x7 << TC3XX_PLLCON1_K3DIV_SHIFT)

#define TC3XX_PLLSTAT_PWDSTAT      (1 << 1)
#define TC3XX_PLLSTAT_LOCK         (1 << 2)
#define TC3XX_PLLSTAT_K3RDY        (1 << 4)    /* PERPLLSTAT only */
#define TC3XX_PLLSTAT_K2RDY        (1 << 5)

#define TC3XX_CCUCON_UP            (1 << 30)
#define TC3XX_CCUCON_LCK           (1 << 31)
#define TC3XX_CCUCON0_CLKSEL_MASK  (0x3 << 28)
#define TC3XX_CCUCON0_CLKSEL_PLL   (0x1 << 28)
#define TC3XX_CCUCON1_CLKSEL_MASK  ((0x3 << 4) | (0x3 << 20) | (0x3 << 28))
#define TC3XX_CCUCON2_ASCLINFDIV   (0xf << 0)
#define TC3XX_CCUCON2_CLKSEL_MASK  (0x3 << 12)

#endif /* __ARCH_TRICORE_SRC_TC3XX_HARDWARE_TC3XX_CLOCK_H */
