/****************************************************************************
 * arch/arm/src/pp502x/hardware/pp502x_intc.h
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

#ifndef __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_INTC_H
#define __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_INTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "hardware/pp502x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register Offsets *********************************************************/

/* The high bank has the same layout at PP502X_INTC_HI_OFFSET */

#define PP502X_INTC_CPU_STAT_OFFSET     0x0000  /* Active CPU IRQ sources */
#define PP502X_INTC_FORCEDCLR_OFFSET    0x001c  /* Write 1 to unforce */
#define PP502X_INTC_CPU_ENSTAT_OFFSET   0x0020  /* CPU enabled sources */
#define PP502X_INTC_CPU_EN_OFFSET       0x0024  /* Write 1 to enable */
#define PP502X_INTC_CPU_DIS_OFFSET      0x0028  /* Write 1 to disable */
#define PP502X_INTC_CPU_PRIO_OFFSET     0x002c  /* 1 routes to FIQ */
#define PP502X_INTC_COP_DIS_OFFSET      0x0038  /* Write 1 to disable */

#define PP502X_INTC_HI_OFFSET           0x0100  /* High bank */

/* Register Addresses *******************************************************/

/* Normal bank */

#define PP502X_INTC_CPU_STAT    (PP502X_INTC_BASE + \
                                 PP502X_INTC_CPU_STAT_OFFSET)
#define PP502X_INTC_FORCEDCLR   (PP502X_INTC_BASE + \
                                 PP502X_INTC_FORCEDCLR_OFFSET)
#define PP502X_INTC_CPU_ENSTAT  (PP502X_INTC_BASE + \
                                 PP502X_INTC_CPU_ENSTAT_OFFSET)
#define PP502X_INTC_CPU_EN      (PP502X_INTC_BASE + PP502X_INTC_CPU_EN_OFFSET)
#define PP502X_INTC_CPU_DIS     (PP502X_INTC_BASE + PP502X_INTC_CPU_DIS_OFFSET)
#define PP502X_INTC_CPU_PRIO    (PP502X_INTC_BASE + \
                                 PP502X_INTC_CPU_PRIO_OFFSET)
#define PP502X_INTC_COP_DIS     (PP502X_INTC_BASE + PP502X_INTC_COP_DIS_OFFSET)

/* High bank */

#define PP502X_INTC_HI_CPU_STAT   (PP502X_INTC_BASE + PP502X_INTC_HI_OFFSET + \
                                   PP502X_INTC_CPU_STAT_OFFSET)
#define PP502X_INTC_HI_FORCEDCLR  (PP502X_INTC_BASE + PP502X_INTC_HI_OFFSET + \
                                   PP502X_INTC_FORCEDCLR_OFFSET)
#define PP502X_INTC_HI_CPU_ENSTAT (PP502X_INTC_BASE + PP502X_INTC_HI_OFFSET + \
                                   PP502X_INTC_CPU_ENSTAT_OFFSET)
#define PP502X_INTC_HI_CPU_EN     (PP502X_INTC_BASE + PP502X_INTC_HI_OFFSET + \
                                   PP502X_INTC_CPU_EN_OFFSET)
#define PP502X_INTC_HI_CPU_DIS    (PP502X_INTC_BASE + PP502X_INTC_HI_OFFSET + \
                                   PP502X_INTC_CPU_DIS_OFFSET)
#define PP502X_INTC_HI_CPU_PRIO   (PP502X_INTC_BASE + PP502X_INTC_HI_OFFSET + \
                                   PP502X_INTC_CPU_PRIO_OFFSET)
#define PP502X_INTC_HI_COP_DIS    (PP502X_INTC_BASE + PP502X_INTC_HI_OFFSET + \
                                   PP502X_INTC_COP_DIS_OFFSET)

/* Register Bitfield Definitions ********************************************/

/* Bit of the normal bank that reports and enables the whole high bank */

#define PP502X_INTC_HI_CASCADE  (1 << 30)

/* CPU_PRIO: sources 31 and 27 are routed to FIQ.  FIQ stays masked, so
 * neither is delivered.
 */

#define PP502X_INTC_CPU_PRIO_FIQ    ((1u << 31) | (1u << 27))

#endif /* __ARCH_ARM_SRC_PP502X_HARDWARE_PP502X_INTC_H */
