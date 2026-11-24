/****************************************************************************
 * arch/arm/src/pp502x/hardware/pp502x_sysctl.h
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

#ifndef __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_SYSCTL_H
#define __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_SYSCTL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/pp502x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

#define PP502X_PROCID_OFFSET        0x0000  /* Processor id */

#define PP502X_CPU_CTL_OFFSET       0x0000  /* CPU control */
#define PP502X_COP_CTL_OFFSET       0x0004  /* COP control */

#define PP502X_DEV_RS_OFFSET        0x0004  /* Device reset */
#define PP502X_DEV_EN_OFFSET        0x000c  /* Device clock enable */
#define PP502X_CLOCK_SOURCE_OFFSET  0x0020  /* Clock source select */
#define PP502X_PLL_CONTROL_OFFSET   0x0034  /* PLL control */
#define PP502X_PLL_STATUS_OFFSET    0x003c  /* PLL status */

#define PP502X_DEV_INIT2_OFFSET     0x0020  /* Device init 2 */
#define PP502X_DEV_TIMING1_OFFSET   0x0034  /* Bus timing */
#define PP502X_GPO32_ENABLE_OFFSET  0x0084  /* GPO32 output enable */
#define PP502X_SER_PINMUX_OFFSET    0x008c  /* UART pin routing */

/* Register Addresses *******************************************************/

#define PP502X_PROC_ID          (PP502X_PROCID_BASE + PP502X_PROCID_OFFSET)

#define PP502X_CPU_CTL          (PP502X_PROCCTL_BASE + PP502X_CPU_CTL_OFFSET)
#define PP502X_COP_CTL          (PP502X_PROCCTL_BASE + PP502X_COP_CTL_OFFSET)

#define PP502X_DEV_RS           (PP502X_DEVCTL_BASE + PP502X_DEV_RS_OFFSET)
#define PP502X_DEV_EN           (PP502X_DEVCTL_BASE + PP502X_DEV_EN_OFFSET)
#define PP502X_CLOCK_SOURCE     (PP502X_DEVCTL_BASE + \
                                 PP502X_CLOCK_SOURCE_OFFSET)
#define PP502X_PLL_CONTROL      (PP502X_DEVCTL_BASE + \
                                 PP502X_PLL_CONTROL_OFFSET)
#define PP502X_PLL_STATUS       (PP502X_DEVCTL_BASE + PP502X_PLL_STATUS_OFFSET)

#define PP502X_DEV_INIT2        (PP502X_SYSCFG_BASE + PP502X_DEV_INIT2_OFFSET)
#define PP502X_DEV_TIMING1      (PP502X_SYSCFG_BASE + \
                                 PP502X_DEV_TIMING1_OFFSET)
#define PP502X_GPO32_ENABLE     (PP502X_SYSCFG_BASE + \
                                 PP502X_GPO32_ENABLE_OFFSET)
#define PP502X_SER_PINMUX       (PP502X_SYSCFG_BASE + \
                                 PP502X_SER_PINMUX_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Processor id: the low byte reads differently on each core */

#define PP502X_PROCID_CPU       0x55
#define PP502X_PROCID_COP       0xaa

/* CPU_CTL and COP_CTL */

#define PP502X_PROCCTL_SLEEP    (1 << 31)  /* Stop until an interrupt */
#define PP502X_PROCCTL_WAIT_CNT (1 << 30)  /* Stop until COUNT expires */
#define PP502X_PROCCTL_CNT_CLKS (1 << 27)  /* COUNT is in clock cycles */

/* Stall the core for n clock cycles */

#define PP502X_PROCCTL_STALL(n) (PP502X_PROCCTL_WAIT_CNT | \
                                 PP502X_PROCCTL_CNT_CLKS | (n))

/* DEV_RS and DEV_EN */

#define PP502X_DEV_SER0         (1 << 6)   /* UART 0 */

/* DEV_INIT2 */

#define PP502X_DEV_INIT2_PLL_POWER  (1 << 30)  /* PLL power */

/* PLL_CONTROL: enable bits, and 80 MHz (24 MHz x 20 / 3 / 2) */

#define PP502X_PLL_CONTROL_ENABLE   0x88000000
#define PP502X_PLL_CONTROL_RUN      0x80000000
#define PP502X_PLL_CONTROL_80MHZ    0x8a121403

/* PLL_STATUS */

#define PP502X_PLL_STATUS_LOCKED    (1 << 31)

/* CLOCK_SOURCE: all four slots on the crystal or on the PLL, slot 2 active */

#define PP502X_CLOCK_SOURCE_CRYSTAL 0x20002222
#define PP502X_CLOCK_SOURCE_PLL     0x20007777

/* DEV_TIMING1 for the crystal and for an 80 MHz core clock */

#define PP502X_DEV_TIMING1_CRYSTAL  0x303
#define PP502X_DEV_TIMING1_80MHZ    0x808

/* SER_PINMUX and GPO32_ENABLE: pins of UART 0, clear to route it to the
 * dock connector
 */

#define PP502X_SER0_PINS_MASK       (3 << 2)

#endif /* __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_SYSCTL_H */
