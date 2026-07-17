/****************************************************************************
 * arch/risc-v/src/gd32vw55x/hardware/gd32vw55x_eclic.h
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

#ifndef __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_ECLIC_H
#define __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_ECLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "gd32vw55x_memorymap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Nuclei ECLIC (Enhanced Core-Local Interrupt Controller).
 * The GD32VW55x implements 116 interrupt inputs with 4 level/priority
 * control bits (CLICINTCTLBITS = 4).
 */

/* Global registers (byte-wide accesses) */

#define GD32VW55X_ECLIC_CFG          (GD32VW55X_ECLIC_BASE + 0x0000) /* 8-bit */
#define GD32VW55X_ECLIC_INFO         (GD32VW55X_ECLIC_BASE + 0x0004) /* 32-bit RO */
#define GD32VW55X_ECLIC_MTH          (GD32VW55X_ECLIC_BASE + 0x000b) /* 8-bit */

/* Per-interrupt registers: 4 bytes per interrupt starting at +0x1000 */

#define GD32VW55X_ECLIC_INTIP(n)     (GD32VW55X_ECLIC_BASE + 0x1000 + 4 * (n))
#define GD32VW55X_ECLIC_INTIE(n)     (GD32VW55X_ECLIC_BASE + 0x1001 + 4 * (n))
#define GD32VW55X_ECLIC_INTATTR(n)   (GD32VW55X_ECLIC_BASE + 0x1002 + 4 * (n))
#define GD32VW55X_ECLIC_INTCTL(n)    (GD32VW55X_ECLIC_BASE + 0x1003 + 4 * (n))

/* CFG register */

#define ECLIC_CFG_NLBITS_SHIFT       1
#define ECLIC_CFG_NLBITS_MASK        (0xf << ECLIC_CFG_NLBITS_SHIFT)

/* INFO register */

#define ECLIC_INFO_NUMINT_MASK       0x1fff
#define ECLIC_INFO_CTLBITS_SHIFT     21
#define ECLIC_INFO_CTLBITS_MASK      (0xf << ECLIC_INFO_CTLBITS_SHIFT)

/* INTIP register */

#define ECLIC_INTIP_IP               (1 << 0)

/* INTIE register */

#define ECLIC_INTIE_IE               (1 << 0)

/* INTATTR register */

#define ECLIC_INTATTR_SHV            (1 << 0)  /* Hardware-vectored */
#define ECLIC_INTATTR_TRIG_SHIFT     1
#define ECLIC_INTATTR_TRIG_MASK      (3 << ECLIC_INTATTR_TRIG_SHIFT)
#define ECLIC_INTATTR_TRIG_LEVEL     (0 << ECLIC_INTATTR_TRIG_SHIFT)
#define ECLIC_INTATTR_TRIG_RISING    (1 << ECLIC_INTATTR_TRIG_SHIFT)
#define ECLIC_INTATTR_TRIG_FALLING   (3 << ECLIC_INTATTR_TRIG_SHIFT)

/* INTCTL register: with CLICINTCTLBITS = 4 and cfg.nlbits = 4, the upper
 * four bits [7:4] hold the interrupt level; unimplemented low bits read
 * as one.  Higher value = higher level.
 */

#define ECLIC_INTCTL_LEVEL(l)        ((((l) & 0xf) << 4) | 0x0f)

/* Nuclei-specific CSRs used with the ECLIC.  Note: the Nuclei interrupt
 * level status CSR lives at 0x346, not at the standard CLIC MINTSTATUS
 * address that arch/risc-v/include/csr.h names CSR_MINTSTATUS (0xfb1); a
 * distinct name avoids redefining it.
 */

#define CSR_MTVT                     0x307  /* Vector table base */
#define CSR_NUCLEI_MINTSTATUS        0x346  /* Interrupt level status */
#define CSR_MCACHE_CTL               0x7ca  /* I-cache control */
#define CSR_MMISC_CTL                0x7d0  /* Misc control (NMI base) */
#define CSR_MTVT2                    0x7ec  /* Non-vectored irq entry */

/* MCACHE_CTL: bit 0 enables the 32 KB I-cache (required for usable
 * performance when executing from flash)
 */

#define MCACHE_CTL_IC_EN             (1 << 0)

/* MMISC_CTL: bit 9 = NMI shares mtvec entry */

#define MMISC_CTL_NMI_CAUSE_FFF      (1 << 9)

/* MTVEC low bits selecting ECLIC mode */

#define ECLIC_MTVEC_MODE             0x03

#endif /* __ARCH_RISCV_SRC_GD32VW55X_HARDWARE_GD32VW55X_ECLIC_H */
