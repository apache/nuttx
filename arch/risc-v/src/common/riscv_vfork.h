/****************************************************************************
 * arch/risc-v/src/common/riscv_vfork.h
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

#ifndef __ARCH_RISCV_SRC_COMMON_RISCV_VFORK_H
#define __ARCH_RISCV_SRC_COMMON_RISCV_VFORK_H

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
 * register s0.  It makes a difference for vfork handling.
 */

#undef VFORK_HAVE_FP

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

#define VFORK_S1_OFFSET     (1*INT_REG_SIZE)   /* Saved register s1 */
#define VFORK_S2_OFFSET     (2*INT_REG_SIZE)   /* Saved register s2 */
#define VFORK_S3_OFFSET     (3*INT_REG_SIZE)   /* Saved register s3 */
#define VFORK_S4_OFFSET     (4*INT_REG_SIZE)   /* Saved register s4 */
#define VFORK_S5_OFFSET     (5*INT_REG_SIZE)   /* Saved register s5 */
#define VFORK_S6_OFFSET     (6*INT_REG_SIZE)   /* Saved register s6 */
#define VFORK_S7_OFFSET     (7*INT_REG_SIZE)   /* Saved register s7 */
#define VFORK_S8_OFFSET     (8*INT_REG_SIZE)   /* Saved register s8 */
#define VFORK_S9_OFFSET     (9*INT_REG_SIZE)   /* Saved register s9 */
#define VFORK_S10_OFFSET    (10*INT_REG_SIZE)  /* Saved register s10 */
#define VFORK_S11_OFFSET    (11*INT_REG_SIZE)  /* Saved register s11 */

#ifdef CONFIG_RISCV_FRAMEPOINTER
#  define VFORK_FP_OFFSET   (0*INT_REG_SIZE)   /* Frame pointer */
#else
#  define VFORK_S0_OFFSET   (0*INT_REG_SIZE)   /* Saved register s0 */
#endif

#define VFORK_SP_OFFSET     (12*INT_REG_SIZE)  /* Stack pointer*/
#define VFORK_RA_OFFSET     (13*INT_REG_SIZE)  /* Return address*/
#ifdef RISCV_SAVE_GP
#  define VFORK_GP_OFFSET   (14*INT_REG_SIZE)  /* Global pointer */
#  define VFORK_INT_SIZE    (15*INT_REG_SIZE)
#else
#  define VFORK_INT_SIZE    (14*INT_REG_SIZE)
#endif

#ifdef CONFIG_ARCH_FPU
#  define VFORK_FS0_OFFSET  (VFORK_INT_SIZE + 0*FPU_REG_FULL_SIZE)
#  define VFORK_FS1_OFFSET  (VFORK_INT_SIZE + 1*FPU_REG_FULL_SIZE)
#  define VFORK_FS2_OFFSET  (VFORK_INT_SIZE + 2*FPU_REG_FULL_SIZE)
#  define VFORK_FS3_OFFSET  (VFORK_INT_SIZE + 3*FPU_REG_FULL_SIZE)
#  define VFORK_FS4_OFFSET  (VFORK_INT_SIZE + 4*FPU_REG_FULL_SIZE)
#  define VFORK_FS5_OFFSET  (VFORK_INT_SIZE + 5*FPU_REG_FULL_SIZE)
#  define VFORK_FS6_OFFSET  (VFORK_INT_SIZE + 6*FPU_REG_FULL_SIZE)
#  define VFORK_FS7_OFFSET  (VFORK_INT_SIZE + 7*FPU_REG_FULL_SIZE)
#  define VFORK_FS8_OFFSET  (VFORK_INT_SIZE + 8*FPU_REG_FULL_SIZE)
#  define VFORK_FS9_OFFSET  (VFORK_INT_SIZE + 9*FPU_REG_FULL_SIZE)
#  define VFORK_FS10_OFFSET (VFORK_INT_SIZE + 10*FPU_REG_FULL_SIZE)
#  define VFORK_FS11_OFFSET (VFORK_INT_SIZE + 11*FPU_REG_FULL_SIZE)
#  define VFORK_FPU_SIZE    (12*FPU_REG_FULL_SIZE)
#else
#  define VFORK_FPU_SIZE    (0)
#endif

#define VFORK_SIZEOF        STACK_ALIGN_UP(VFORK_INT_SIZE + VFORK_FPU_SIZE)

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
struct vfork_s
{
  /* CPU registers */

  uintptr_t s1;   /* Saved register s1 */
  uintptr_t s2;   /* Saved register s2 */
  uintptr_t s3;   /* Saved register s3 */
  uintptr_t s4;   /* Saved register s4 */
  uintptr_t s5;   /* Saved register s5 */
  uintptr_t s6;   /* Saved register s6 */
  uintptr_t s7;   /* Saved register s7 */
  uintptr_t s8;   /* Saved register s8 */
  uintptr_t s9;   /* Saved register s9 */
  uintptr_t s10;  /* Saved register s10 */
  uintptr_t s11;  /* Saved register s11 */
#ifdef CONFIG_RISCV_FRAMEPOINTER
  uintptr_t fp;   /* Frame pointer */
#else
  uintptr_t s0;   /* Saved register s0 */
#endif
  uintptr_t sp;   /* Stack pointer */
  uintptr_t ra;   /* Return address */
#ifdef RISCV_SAVE_GP
  uintptr_t gp;   /* Global pointer */
#endif

  /* Floating point registers */

#ifdef CONFIG_ARCH_FPU
  uintptr_t fs0;   /* Saved register fs0 */
  uintptr_t fs1;   /* Saved register fs1 */
  uintptr_t fs2;   /* Saved register fs2 */
  uintptr_t fs3;   /* Saved register fs3 */
  uintptr_t fs4;   /* Saved register fs4 */
  uintptr_t fs5;   /* Saved register fs5 */
  uintptr_t fs6;   /* Saved register fs6 */
  uintptr_t fs7;   /* Saved register fs7 */
  uintptr_t fs8;   /* Saved register fs8 */
  uintptr_t fs9;   /* Saved register fs9 */
  uintptr_t fs10;  /* Saved register fs10 */
  uintptr_t fs11;  /* Saved register fs11 */
#endif
};
#endif

#endif /* __ARCH_RISCV_SRC_COMMON_RISCV_VFORK_H */
