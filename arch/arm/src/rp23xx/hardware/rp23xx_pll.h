/****************************************************************************
 * arch/arm/src/rp23xx/hardware/rp23xx_pll.h
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

#ifndef __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PLL_H
#define __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/rp23xx_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define RP23XX_PLL_CS_OFFSET         0x00000000
#define RP23XX_PLL_PWR_OFFSET        0x00000004
#define RP23XX_PLL_FBDIV_INT_OFFSET  0x00000008
#define RP23XX_PLL_PRIM_OFFSET       0x0000000c
#define RP23XX_PLL_INTR_OFFSET       0x00000010
#define RP23XX_PLL_INTE_OFFSET       0x00000014
#define RP23XX_PLL_INTF_OFFSET       0x00000018
#define RP23XX_PLL_INTS_OFFSET       0x0000001c

/* Register definitions *****************************************************/

#define RP23XX_PLL_CS         (RP23XX_PLL_BASE + RP23XX_PLL_CS_OFFSET)
#define RP23XX_PLL_PWR        (RP23XX_PLL_BASE + RP23XX_PLL_PWR_OFFSET)
#define RP23XX_PLL_FBDIV_INT  (RP23XX_PLL_BASE + RP23XX_PLL_FBDIV_INT_OFFSET)
#define RP23XX_PLL_PRIM       (RP23XX_PLL_BASE + RP23XX_PLL_PRIM_OFFSET)
#define RP23XX_PLL_INTR       (RP23XX_PLL_BASE + RP23XX_PLL_INTR_OFFSET)
#define RP23XX_PLL_INTE       (RP23XX_PLL_BASE + RP23XX_PLL_INTE_OFFSET)
#define RP23XX_PLL_INTF       (RP23XX_PLL_BASE + RP23XX_PLL_INTF_OFFSET)
#define RP23XX_PLL_INTS       (RP23XX_PLL_BASE + RP23XX_PLL_INTS_OFFSET)

/* Register bit definitions *************************************************/

#define RP23XX_PLL_CS_MASK              (0xc000013f)
#define RP23XX_PLL_CS_LOCK              (1 << 31)
#define RP23XX_PLL_CS_LOCK_N            (1 << 30)
#define RP23XX_PLL_CS_BYPASS            (1 << 8)
#define RP23XX_PLL_CS_REFDIV_MASK       (0x0000003f)
#define RP23XX_PLL_PWR_MASK             (0x0000002d)
#define RP23XX_PLL_PWR_VCOPD            (1 << 5)
#define RP23XX_PLL_PWR_POSTDIVPD        (1 << 3)
#define RP23XX_PLL_PWR_DSMPD            (1 << 2)
#define RP23XX_PLL_PWR_PD               (1 << 0)
#define RP23XX_PLL_FBDIV_INT_MASK       (0x00000fff)
#define RP23XX_PLL_PRIM_MASK            (0x00077000)
#define RP23XX_PLL_PRIM_POSTDIV1_SHIFT  (16)  /* divide by 1-7 */
#define RP23XX_PLL_PRIM_POSTDIV1_MASK   (0x07 << RP23XX_PLL_PRIM_POSTDIV1_SHIFT)
#define RP23XX_PLL_PRIM_POSTDIV2_SHIFT  (12)  /* divide by 1-7 */
#define RP23XX_PLL_PRIM_POSTDIV2_MASK   (0x07 << RP23XX_PLL_PRIM_POSTDIV2_SHIFT)
#define RP23XX_PLL_INTR_LOCK_N_STICKY   (1 << 0)
#define RP23XX_PLL_INTE_LOCK_N_STICKY   (1 << 0)
#define RP23XX_PLL_INTF_LOCK_N_STICKY   (1 << 0)
#define RP23XX_PLL_INTS_LOCK_N_STICKY   (1 << 0)

#endif /* __ARCH_ARM_SRC_RP23XX_HARDWARE_RP23XX_PLL_H */
