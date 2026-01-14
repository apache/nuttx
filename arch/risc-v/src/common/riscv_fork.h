/****************************************************************************
 * arch/risc-v/src/common/riscv_fork.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_RISCV_FORK_H
#define __ARCH_RISCV_SRC_COMMON_RISCV_FORK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

#include "riscv_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Register x8 may be a frame pointer in some ABIs.  Or may just be saved
 * register s0.  It makes a difference for fork handling.
 */

#undef FORK_HAVE_FP

/* Register ABI Name Description                        Saver
 *
 * x0       zero     Hard-wired zero                    —
 * x1       ra       Return address                     Caller
 * x2       sp       Stack pointer                      Callee
 * x3       gp       Global pointer                     —
 * x4       tp       Thread pointer                     —
 * x5–7     t0–2     Temporaries                        Caller
 * x8       s0/fp    Saved register/frame pointer       Callee
 * x9       s1       Saved register                     Callee
 * x10–11   a0–1     Function arguments/return values   Caller
 * x12–17   a2–7     Function arguments                 Caller
 * x18–27   s2–11    Saved registers                    Callee
 * x28–31   t3–6     Temporaries                        Caller
 * f0–7     ft0–7    FP temporaries                     Caller
 * f8–9     fs0–1    FP saved registers                 Callee
 * f10–11   fa0–1    FP arguments/return values         Caller
 * f12–17   fa2–7    FP arguments                       Caller
 * f18–27   fs2–11   FP saved registers                 Callee
 * f28–31   ft8–11   FP temporaries                     Caller
 */

#define FORK_S1_OFFSET     (1*INT_REG_SIZE)   /* Saved register s1 */
#define FORK_S2_OFFSET     (2*INT_REG_SIZE)   /* Saved register s2 */
#define FORK_S3_OFFSET     (3*INT_REG_SIZE)   /* Saved register s3 */
#define FORK_S4_OFFSET     (4*INT_REG_SIZE)   /* Saved register s4 */
#define FORK_S5_OFFSET     (5*INT_REG_SIZE)   /* Saved register s5 */
#define FORK_S6_OFFSET     (6*INT_REG_SIZE)   /* Saved register s6 */
#define FORK_S7_OFFSET     (7*INT_REG_SIZE)   /* Saved register s7 */
#define FORK_S8_OFFSET     (8*INT_REG_SIZE)   /* Saved register s8 */
#define FORK_S9_OFFSET     (9*INT_REG_SIZE)   /* Saved register s9 */
#define FORK_S10_OFFSET    (10*INT_REG_SIZE)  /* Saved register s10 */
#define FORK_S11_OFFSET    (11*INT_REG_SIZE)  /* Saved register s11 */

#ifdef CONFIG_RISCV_FRAMEPOINTER
#  define FORK_FP_OFFSET   (0*INT_REG_SIZE)   /* Frame pointer */
#else
#  define FORK_S0_OFFSET   (0*INT_REG_SIZE)   /* Saved register s0 */
#endif

#define FORK_SP_OFFSET     (12*INT_REG_SIZE)  /* Stack pointer*/
#define FORK_RA_OFFSET     (13*INT_REG_SIZE)  /* Return address*/
#ifdef RISCV_SAVE_GP
#  define FORK_GP_OFFSET   (14*INT_REG_SIZE)  /* Global pointer */
#  define FORK_INT_SIZE    (15*INT_REG_SIZE)
#else
#  define FORK_INT_SIZE    (14*INT_REG_SIZE)
#endif

#ifdef CONFIG_ARCH_FPU
#  define FPU_REG_FULL_SIZE (INT_REG_SIZE * FPU_REG_SIZE)
#  define FORK_FS0_OFFSET  (FORK_INT_SIZE + 0*FPU_REG_FULL_SIZE)
#  define FORK_FS1_OFFSET  (FORK_INT_SIZE + 1*FPU_REG_FULL_SIZE)
#  define FORK_FS2_OFFSET  (FORK_INT_SIZE + 2*FPU_REG_FULL_SIZE)
#  define FORK_FS3_OFFSET  (FORK_INT_SIZE + 3*FPU_REG_FULL_SIZE)
#  define FORK_FS4_OFFSET  (FORK_INT_SIZE + 4*FPU_REG_FULL_SIZE)
#  define FORK_FS5_OFFSET  (FORK_INT_SIZE + 5*FPU_REG_FULL_SIZE)
#  define FORK_FS6_OFFSET  (FORK_INT_SIZE + 6*FPU_REG_FULL_SIZE)
#  define FORK_FS7_OFFSET  (FORK_INT_SIZE + 7*FPU_REG_FULL_SIZE)
#  define FORK_FS8_OFFSET  (FORK_INT_SIZE + 8*FPU_REG_FULL_SIZE)
#  define FORK_FS9_OFFSET  (FORK_INT_SIZE + 9*FPU_REG_FULL_SIZE)
#  define FORK_FS10_OFFSET (FORK_INT_SIZE + 10*FPU_REG_FULL_SIZE)
#  define FORK_FS11_OFFSET (FORK_INT_SIZE + 11*FPU_REG_FULL_SIZE)
#  define FORK_FPU_SIZE    (12*FPU_REG_FULL_SIZE)
#else
#  define FORK_FPU_SIZE    (0)
#endif

#define FORK_SIZEOF        STACK_ALIGN_UP(FORK_INT_SIZE + FORK_FPU_SIZE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
struct fork_s
{
  /* CPU registers */

#ifdef CONFIG_RISCV_FRAMEPOINTER
  uintreg_t fp;   /* Frame pointer */
#else
  uintreg_t s0;   /* Saved register s0 */
#endif
  uintreg_t s1;   /* Saved register s1 */
  uintreg_t s2;   /* Saved register s2 */
  uintreg_t s3;   /* Saved register s3 */
  uintreg_t s4;   /* Saved register s4 */
  uintreg_t s5;   /* Saved register s5 */
  uintreg_t s6;   /* Saved register s6 */
  uintreg_t s7;   /* Saved register s7 */
  uintreg_t s8;   /* Saved register s8 */
  uintreg_t s9;   /* Saved register s9 */
  uintreg_t s10;  /* Saved register s10 */
  uintreg_t s11;  /* Saved register s11 */
  uintreg_t sp;   /* Stack pointer */
  uintreg_t ra;   /* Return address */
#ifdef RISCV_SAVE_GP
  uintreg_t gp;   /* Global pointer */
#endif

  /* Floating point registers */

#ifdef CONFIG_ARCH_FPU
  uintreg_t fs0;   /* Saved register fs0 */
  uintreg_t fs1;   /* Saved register fs1 */
  uintreg_t fs2;   /* Saved register fs2 */
  uintreg_t fs3;   /* Saved register fs3 */
  uintreg_t fs4;   /* Saved register fs4 */
  uintreg_t fs5;   /* Saved register fs5 */
  uintreg_t fs6;   /* Saved register fs6 */
  uintreg_t fs7;   /* Saved register fs7 */
  uintreg_t fs8;   /* Saved register fs8 */
  uintreg_t fs9;   /* Saved register fs9 */
  uintreg_t fs10;  /* Saved register fs10 */
  uintreg_t fs11;  /* Saved register fs11 */
#endif
};
#endif

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_FORK_H */
