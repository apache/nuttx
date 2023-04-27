/****************************************************************************
 * arch/risc-v/src/common/riscv_macros.S
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

.file "riscv_macros.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/arch.h>
#include <arch/csr.h>
#include <arch/irq.h>

#include <sys/types.h>

#include "riscv_internal.h"
#include "riscv_percpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Name: save_ctx
 *
 * Parameter:
 *   in - Pointer to where the save is performed (e.g. sp)
 *
 * Description:
 *   Save the common context registers (i.e. work / temp / etc).
 *
 ****************************************************************************/

.macro save_ctx in

  REGSTORE   x1,  REG_X1(\in)    /* ra */
#ifdef RISCV_SAVE_GP
  REGSTORE   x3,  REG_X3(\in)    /* gp */
#endif
  REGSTORE   x4,  REG_X4(\in)    /* tp */
  REGSTORE   x5,  REG_X5(\in)    /* t0 */
  REGSTORE   x6,  REG_X6(\in)    /* t1 */
  REGSTORE   x7,  REG_X7(\in)    /* t2 */
  REGSTORE   x8,  REG_X8(\in)    /* s0 */
  REGSTORE   x9,  REG_X9(\in)    /* s1 */
  REGSTORE   x10, REG_X10(\in)   /* a0 */
  REGSTORE   x11, REG_X11(\in)   /* a1 */
  REGSTORE   x12, REG_X12(\in)   /* a2 */
  REGSTORE   x13, REG_X13(\in)   /* a3 */
  REGSTORE   x14, REG_X14(\in)   /* a4 */
  REGSTORE   x15, REG_X15(\in)   /* a5 */
  REGSTORE   x16, REG_X16(\in)   /* a6 */
  REGSTORE   x17, REG_X17(\in)   /* a7 */
  REGSTORE   x18, REG_X18(\in)   /* s2 */
  REGSTORE   x19, REG_X19(\in)   /* s3 */
  REGSTORE   x20, REG_X20(\in)   /* s4 */
  REGSTORE   x21, REG_X21(\in)   /* s5 */
  REGSTORE   x22, REG_X22(\in)   /* s6 */
  REGSTORE   x23, REG_X23(\in)   /* s7 */
  REGSTORE   x24, REG_X24(\in)   /* s8 */
  REGSTORE   x25, REG_X25(\in)   /* s9 */
  REGSTORE   x26, REG_X26(\in)   /* s10 */
  REGSTORE   x27, REG_X27(\in)   /* s11 */
  REGSTORE   x28, REG_X28(\in)   /* t3 */
  REGSTORE   x29, REG_X29(\in)   /* t4 */
  REGSTORE   x30, REG_X30(\in)   /* t5 */
  REGSTORE   x31, REG_X31(\in)   /* t6 */

.endm

/****************************************************************************
 * Name: riscv_savefpu
 *
 * Parameter:
 *   in - Pointer to where the save is performed (e.g. sp)
 *
 * Description:
 *   Save the FPU context registers (i.e. work / temp / etc).
 *
 ****************************************************************************/

.macro riscv_savefpu in

#ifdef CONFIG_ARCH_FPU
  REGLOAD    t0, REG_INT_CTX(\in)
  li         t1, MSTATUS_FS
  and        t2, t0, t1
  li         t1, MSTATUS_FS_DIRTY
  bne        t2, t1, 1f
  li         t1, ~MSTATUS_FS
  and        t0, t0, t1
  li         t1, MSTATUS_FS_CLEAN
  or         t0, t0, t1
  REGSTORE   t0, REG_INT_CTX(\in)

  /* Store all floating point registers */

  FSTORE     f0,  REG_F0(\in)
  FSTORE     f1,  REG_F1(\in)
  FSTORE     f2,  REG_F2(\in)
  FSTORE     f3,  REG_F3(\in)
  FSTORE     f4,  REG_F4(\in)
  FSTORE     f5,  REG_F5(\in)
  FSTORE     f6,  REG_F6(\in)
  FSTORE     f7,  REG_F7(\in)
  FSTORE     f8,  REG_F8(\in)
  FSTORE     f9,  REG_F9(\in)
  FSTORE     f10, REG_F10(\in)
  FSTORE     f11, REG_F11(\in)
  FSTORE     f12, REG_F12(\in)
  FSTORE     f13, REG_F13(\in)
  FSTORE     f14, REG_F14(\in)
  FSTORE     f15, REG_F15(\in)
  FSTORE     f16, REG_F16(\in)
  FSTORE     f17, REG_F17(\in)
  FSTORE     f18, REG_F18(\in)
  FSTORE     f19, REG_F19(\in)
  FSTORE     f20, REG_F20(\in)
  FSTORE     f21, REG_F21(\in)
  FSTORE     f22, REG_F22(\in)
  FSTORE     f23, REG_F23(\in)
  FSTORE     f24, REG_F24(\in)
  FSTORE     f25, REG_F25(\in)
  FSTORE     f26, REG_F26(\in)
  FSTORE     f27, REG_F27(\in)
  FSTORE     f28, REG_F28(\in)
  FSTORE     f29, REG_F29(\in)
  FSTORE     f30, REG_F30(\in)
  FSTORE     f31, REG_F31(\in)

  frcsr      t0
  REGSTORE   t0, REG_FCSR(\in)

1:

  /* Restore what we have just destroyed (t0, t1, t2) */

  REGLOAD      t0, REG_T0(\in)
  REGLOAD      t1, REG_T1(\in)
  REGLOAD      t2, REG_T2(\in)

#endif

.endm

/****************************************************************************
 * Name: load_ctx
 *
 * Parameter:
 *   out - Pointer to where the load is performed (e.g. sp)
 *
 * Description:
 *   Load the common context registers (i.e. work / temp / etc).
 *
 ****************************************************************************/

.macro load_ctx out

  REGLOAD    x1,  REG_X1(\out)   /* ra */
#ifdef RISCV_SAVE_GP
  REGLOAD    x3,  REG_X3(\out)   /* gp */
#endif
  REGLOAD    x4,  REG_X4(\out)   /* tp */
  REGLOAD    x5,  REG_X5(\out)   /* t0 */
  REGLOAD    x6,  REG_X6(\out)   /* t1 */
  REGLOAD    x7,  REG_X7(\out)   /* t2 */
  REGLOAD    x8,  REG_X8(\out)   /* s0 */
  REGLOAD    x9,  REG_X9(\out)   /* s1 */
  REGLOAD    x10, REG_X10(\out)  /* a0 */
  REGLOAD    x11, REG_X11(\out)  /* a1 */
  REGLOAD    x12, REG_X12(\out)  /* a2 */
  REGLOAD    x13, REG_X13(\out)  /* a3 */
  REGLOAD    x14, REG_X14(\out)  /* a4 */
  REGLOAD    x15, REG_X15(\out)  /* a5 */
  REGLOAD    x16, REG_X16(\out)  /* a6 */
  REGLOAD    x17, REG_X17(\out)  /* a7 */
  REGLOAD    x18, REG_X18(\out)  /* s2 */
  REGLOAD    x19, REG_X19(\out)  /* s3 */
  REGLOAD    x20, REG_X20(\out)  /* s4 */
  REGLOAD    x21, REG_X21(\out)  /* s5 */
  REGLOAD    x22, REG_X22(\out)  /* s6 */
  REGLOAD    x23, REG_X23(\out)  /* s7 */
  REGLOAD    x24, REG_X24(\out)  /* s8 */
  REGLOAD    x25, REG_X25(\out)  /* s9 */
  REGLOAD    x26, REG_X26(\out)  /* s10 */
  REGLOAD    x27, REG_X27(\out)  /* s11 */
  REGLOAD    x28, REG_X28(\out)  /* t3 */
  REGLOAD    x29, REG_X29(\out)  /* t4 */
  REGLOAD    x30, REG_X30(\out)  /* t5 */
  REGLOAD    x31, REG_X31(\out)  /* t6 */

.endm

/****************************************************************************
 * Name: riscv_loadfpu
 *
 * Parameter:
 *   out - Pointer to where the load is performed (e.g. sp)
 *
 * Description:
 *   Load the FPU context registers (i.e. work / temp / etc).
 *
 ****************************************************************************/

.macro riscv_loadfpu out

#ifdef CONFIG_ARCH_FPU
  REGLOAD      t0, REG_INT_CTX(\out)
  li           t1, MSTATUS_FS
  and          t2, t0, t1
  li           t1, MSTATUS_FS_INIT
  ble          t2, t1, 1f

  /* Load all floating point registers */

  FLOAD        f0, REG_F0(\out)
  FLOAD        f1, REG_F1(\out)
  FLOAD        f2, REG_F2(\out)
  FLOAD        f3, REG_F3(\out)
  FLOAD        f4, REG_F4(\out)
  FLOAD        f5, REG_F5(\out)
  FLOAD        f6, REG_F6(\out)
  FLOAD        f7, REG_F7(\out)
  FLOAD        f8, REG_F8(\out)
  FLOAD        f9, REG_F9(\out)
  FLOAD        f10, REG_F10(\out)
  FLOAD        f11, REG_F11(\out)
  FLOAD        f12, REG_F12(\out)
  FLOAD        f13, REG_F13(\out)
  FLOAD        f14, REG_F14(\out)
  FLOAD        f15, REG_F15(\out)
  FLOAD        f16, REG_F16(\out)
  FLOAD        f17, REG_F17(\out)
  FLOAD        f18, REG_F18(\out)
  FLOAD        f19, REG_F19(\out)
  FLOAD        f20, REG_F20(\out)
  FLOAD        f21, REG_F21(\out)
  FLOAD        f22, REG_F22(\out)
  FLOAD        f23, REG_F23(\out)
  FLOAD        f24, REG_F24(\out)
  FLOAD        f25, REG_F25(\out)
  FLOAD        f26, REG_F26(\out)
  FLOAD        f27, REG_F27(\out)
  FLOAD        f28, REG_F28(\out)
  FLOAD        f29, REG_F29(\out)
  FLOAD        f30, REG_F30(\out)
  FLOAD        f31, REG_F31(\out)

  /* Store the floating point control and status register */

  REGLOAD      t0, REG_FCSR(\out)
  fscsr        t0

1:

  /* Restore what we have just destroyed (t0, t1, t2) */

  REGLOAD      t0, REG_T0(\out)
  REGLOAD      t1, REG_T1(\out)
  REGLOAD      t2, REG_T2(\out)

#endif

.endm

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the "top" the interrupt stack. Works
 *   for single CPU case in flat mode.
 *   Must be provided by MCU-specific logic in the SMP case, or the kernel
 *   runs in supervisor mode (S-mode).
 *
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 15
#if !defined(CONFIG_SMP) && !defined(CONFIG_ARCH_USE_S_MODE)
.macro  setintstack tmp0, tmp1
  la    sp, g_intstacktop
.endm
#endif /* !defined(CONFIG_SMP) && !defined(CONFIG_ARCH_USE_S_MODE) */
#endif /* CONFIG_ARCH_INTERRUPTSTACK > 15 */

/****************************************************************************
 * Name: riscv_mhartid
 *
 * Description:
 *   Context aware way to query hart id
 *
 * Returned Value:
 *   Hart id
 *
 ****************************************************************************/

.macro  riscv_mhartid out
#ifdef CONFIG_ARCH_USE_S_MODE
  csrr    \out, CSR_SCRATCH
  REGLOAD \out, RISCV_PERCPU_HARTID(\out)
#else
  csrr    \out, mhartid
#endif
.endm
