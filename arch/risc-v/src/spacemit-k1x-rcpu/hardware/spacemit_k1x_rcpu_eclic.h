/****************************************************************************
 * arch/risc-v/src/spacemit-k1x-rcpu/hardware/spacemit_k1x_rcpu_eclic.h
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

#ifndef __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_HARDWARE_SPACEMIT_K1X_RCPU_ECLIC_H
#define __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_HARDWARE_SPACEMIT_K1X_RCPU_ECLIC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The K1 RCPU uses a Nuclei ECLIC (Enhanced Core-Level Interrupt
 * Controller).  It is a CLIC implementation, not a PLIC: the per-source
 * enable lives in an ECLIC register rather than in the mie CSR, and the
 * core must be put into CLIC mode by programming mtvec with mode 3.
 *
 * The ECLIC is not at a compile time address.  It lives inside the Nuclei
 * IRegion, whose base must be read from CSR_MIRGB_INFO.  All of the macros
 * below therefore take the discovered base as their first argument.  See
 * spacemit_k1x_rcpu_memorymap.h for the measured base and
 * spacemit_k1x_rcpu_irq.c for the discovery.
 *
 * The register layout below is the Nuclei ECLIC layout, which is also what
 * the NuttX gd32vw55x port (another Nuclei core) uses.  It was confirmed
 * against the values read back from a running K1 over JTAG.
 */

#define SPACEMIT_K1X_RCPU_ECLIC_CFG_OFFSET        0x0000  /* 8-bit  R/W  global config   */
#define SPACEMIT_K1X_RCPU_ECLIC_INFO_OFFSET       0x0004  /* 32-bit RO   capability      */
#define SPACEMIT_K1X_RCPU_ECLIC_MTH_OFFSET        0x000b  /* 8-bit  R/W  threshold       */

#define SPACEMIT_K1X_RCPU_ECLIC_INTIP_OFFSET      0x1000  /* 8-bit  R/W  pending         */
#define SPACEMIT_K1X_RCPU_ECLIC_INTIE_OFFSET      0x1001  /* 8-bit  R/W  enable          */
#define SPACEMIT_K1X_RCPU_ECLIC_INTATTR_OFFSET    0x1002  /* 8-bit  R/W  attribute       */
#define SPACEMIT_K1X_RCPU_ECLIC_INTCTL_OFFSET     0x1003  /* 8-bit  R/W  level + prio    */

#define SPACEMIT_K1X_RCPU_ECLIC_INT_STRIDE        4       /* Bytes per interrupt source  */

#define SPACEMIT_K1X_RCPU_ECLIC_CFG(base)         ((base) + SPACEMIT_K1X_RCPU_ECLIC_CFG_OFFSET)
#define SPACEMIT_K1X_RCPU_ECLIC_INFO(base)        ((base) + SPACEMIT_K1X_RCPU_ECLIC_INFO_OFFSET)
#define SPACEMIT_K1X_RCPU_ECLIC_MTH(base)         ((base) + SPACEMIT_K1X_RCPU_ECLIC_MTH_OFFSET)
#define SPACEMIT_K1X_RCPU_ECLIC_INTIP(base,n)     ((base) + SPACEMIT_K1X_RCPU_ECLIC_INTIP_OFFSET + \
                                     SPACEMIT_K1X_RCPU_ECLIC_INT_STRIDE * (n))
#define SPACEMIT_K1X_RCPU_ECLIC_INTIE(base,n)     ((base) + SPACEMIT_K1X_RCPU_ECLIC_INTIE_OFFSET + \
                                     SPACEMIT_K1X_RCPU_ECLIC_INT_STRIDE * (n))
#define SPACEMIT_K1X_RCPU_ECLIC_INTATTR(base,n)   ((base) + SPACEMIT_K1X_RCPU_ECLIC_INTATTR_OFFSET + \
                                     SPACEMIT_K1X_RCPU_ECLIC_INT_STRIDE * (n))
#define SPACEMIT_K1X_RCPU_ECLIC_INTCTL(base,n)    ((base) + SPACEMIT_K1X_RCPU_ECLIC_INTCTL_OFFSET + \
                                     SPACEMIT_K1X_RCPU_ECLIC_INT_STRIDE * (n))

/* CFG: number of bits in the level field of INTCTL is NLBITS + 1.
 *
 * A stock MUSE-Pi Pro reports CFG = 0x07, i.e. NLBITS = 3 and therefore four
 * level bits.  That agrees with the CTLBITS field of INFO, which reports 3
 * as well, and with the audio firmware, which programs this field before
 * enabling anything.
 */

#define SPACEMIT_K1X_RCPU_ECLIC_CFG_NLBITS_SHIFT  1
#define SPACEMIT_K1X_RCPU_ECLIC_CFG_NLBITS_MASK   (0xf << SPACEMIT_K1X_RCPU_ECLIC_CFG_NLBITS_SHIFT)

/* INFO: NUMINT is the number of interrupt sources, CTLBITS repeats the
 * INTCTL level width.  Both are read at run time so that the driver does not
 * have to carry a hard coded source count.
 */

#define SPACEMIT_K1X_RCPU_ECLIC_INFO_NUMINT_MASK  0x1fff
#define SPACEMIT_K1X_RCPU_ECLIC_INFO_CTLBITS_SHIFT 21
#define SPACEMIT_K1X_RCPU_ECLIC_INFO_CTLBITS_MASK (0xf << SPACEMIT_K1X_RCPU_ECLIC_INFO_CTLBITS_SHIFT)

/* Per-source registers. */

#define SPACEMIT_K1X_RCPU_ECLIC_INTIP_IP          (1 << 0)  /* Interrupt is pending      */
#define SPACEMIT_K1X_RCPU_ECLIC_INTIE_IE          (1 << 0)  /* Interrupt is enabled      */
#define SPACEMIT_K1X_RCPU_ECLIC_INTATTR_SHV       (1 << 0)  /* Hardware vectored         */
#define SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_SHIFT 1
#define SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_MASK (3 << SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_SHIFT)
#define SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_LEVEL   (0 << SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_SHIFT)
#define SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_RISING  (1 << SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_SHIFT)
#define SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_FALLING (3 << SPACEMIT_K1X_RCPU_ECLIC_INTATTR_TRIG_SHIFT)

/* INTCTL packs the interrupt level in the high bits and the priority in the
 * low ones.  With four level bits, a level of l is encoded as
 * (l << 4) | 0x0f; the 0x0f keeps the source at the highest priority.
 */

#define SPACEMIT_K1X_RCPU_ECLIC_INTCTL_LEVEL(l)   ((((l) & 0xf) << 4) | 0x0f)

/* Nuclei private CSRs used to locate and drive the ECLIC.
 *
 * These are core CSRs, not memory mapped registers.  The values were taken
 * from the Nuclei NMSIS headers shipped with the vendor RTOS and were
 * confirmed on hardware (see spacemit_k1x_rcpu_memorymap.h for the measured
 * values).
 *
 * CSR_MTVT is not repeated here: the common RISC-V header csr.h already
 * defines it, with the same value.
 */

#define CSR_MTVT2                   0x7ec   /* Non-vectored entry + enable */
#define CSR_MMISC_CTL               0x7d0   /* Misc control                */
#define CSR_MCACHE_CTL              0x7ca   /* Cache control               */
#define CSR_MCFG_INFO               0xfc2   /* Core capabilities           */
#define CSR_MIRGB_INFO              0x7f7   /* IRegion base                */

/* mtvec mode field.  In CLIC mode the low bits select the CLIC vectoring
 * scheme; mode 3 selects vectored CLIC with the table based at mtvec and the
 * individual vectors taken from mtvt.
 */

#define SPACEMIT_K1X_RCPU_ECLIC_MTVEC_MODE        0x03

/* MMISC_CTL bit 9 makes the NMI share mtvec with the regular trap vector. */

#define SPACEMIT_K1X_RCPU_MMISC_CTL_NMI_CAUSE_FFF (1 << 9)

/* MCACHE_CTL bit 0 enables the instruction cache. */

#define SPACEMIT_K1X_RCPU_MCACHE_CTL_IC_EN        (1 << 0)

#endif /* __ARCH_RISCV_SRC_SPACEMIT_K1X_RCPU_HARDWARE_SPACEMIT_K1X_RCPU_ECLIC_H */
