/****************************************************************************
 * arch/arm/src/efr32/hardware/efr32_dpll.h
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

#ifndef __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_DPLL_H
#define __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_DPLL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "hardware/efr32_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register offsets *********************************************************/

#define EFR32_DPLL_EN_OFFSET      0x0004  /* Enable */
#define EFR32_DPLL_CFG_OFFSET     0x0008  /* Configuration */
#define EFR32_DPLL_CFG1_OFFSET    0x000c  /* Configuration 1 (N, M) */
#define EFR32_DPLL_IF_OFFSET      0x0010  /* Interrupt flags */
#define EFR32_DPLL_STATUS_OFFSET  0x0018  /* Status */
#define EFR32_DPLL_LOCK_OFFSET    0x0024  /* Configuration lock */

/* Register addresses *******************************************************/

#define EFR32_DPLL0_EN            (EFR32_DPLL0_BASE + EFR32_DPLL_EN_OFFSET)
#define EFR32_DPLL0_CFG           (EFR32_DPLL0_BASE + EFR32_DPLL_CFG_OFFSET)
#define EFR32_DPLL0_CFG1          (EFR32_DPLL0_BASE + EFR32_DPLL_CFG1_OFFSET)
#define EFR32_DPLL0_IF            (EFR32_DPLL0_BASE + EFR32_DPLL_IF_OFFSET)
#define EFR32_DPLL0_STATUS        (EFR32_DPLL0_BASE + EFR32_DPLL_STATUS_OFFSET)
#define EFR32_DPLL0_LOCK          (EFR32_DPLL0_BASE + EFR32_DPLL_LOCK_OFFSET)

#define EFR32_DPLL0_EN_SET        (EFR32_DPLL0_EN + EFR32_SET_OFFSET)
#define EFR32_DPLL0_EN_CLR        (EFR32_DPLL0_EN + EFR32_CLR_OFFSET)
#define EFR32_DPLL0_IF_CLR        (EFR32_DPLL0_IF + EFR32_CLR_OFFSET)

/* Register bit definitions *************************************************/

/* Enable */

#define DPLL_EN_EN                (1 << 0)
#define DPLL_EN_DISABLING         (1 << 1)

/* Configuration */

#define DPLL_CFG_MODE_PLL         (1 << 0)  /* Phase lock (else frequency) */
#define DPLL_CFG_EDGESEL_RISE     (1 << 1)  /* Rising edge (else falling) */
#define DPLL_CFG_AUTORECOVER      (1 << 2)
#define DPLL_CFG_DITHEN           (1 << 6)

/* Configuration 1: fout = fref * (N + 1) / (M + 1) */

#define DPLL_CFG1_M_SHIFT         (0)
#define DPLL_CFG1_M_MASK          (0xfff << DPLL_CFG1_M_SHIFT)
#  define DPLL_CFG1_M(n)          ((uint32_t)(n) << DPLL_CFG1_M_SHIFT)
#define DPLL_CFG1_N_SHIFT         (16)
#define DPLL_CFG1_N_MASK          (0xfff << DPLL_CFG1_N_SHIFT)
#  define DPLL_CFG1_N(n)          ((uint32_t)(n) << DPLL_CFG1_N_SHIFT)

/* Interrupt flags */

#define DPLL_IF_LOCK              (1 << 0)
#define DPLL_IF_LOCKFAILLOW       (1 << 1)
#define DPLL_IF_LOCKFAILHIGH      (1 << 2)

/* Status */

#define DPLL_STATUS_RDY           (1 << 0)
#define DPLL_STATUS_ENS           (1 << 1)
#define DPLL_STATUS_LOCK          (1 << 31)

/* Configuration lock */

#define DPLL_LOCK_LOCKKEY_UNLOCK  0x7102

#endif /* __ARCH_ARM_SRC_EFR32_HARDWARE_EFR32_DPLL_H */
