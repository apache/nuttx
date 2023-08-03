/****************************************************************************
 * arch/risc-v/src/common/riscv_fpu.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/arch.h>
#include <arch/csr.h>
#include <arch/irq.h>
#include <arch/mode.h>

#include "riscv_macros.S"

#ifdef CONFIG_ARCH_FPU

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .globl        riscv_fpuconfig
    .globl        riscv_savefpu
    .globl        riscv_restorefpu
    .file         "riscv_fpu.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_fpuconfig
 *
 * Description:
 *   init fpu
 *
 * C Function Prototype:
 *   void riscv_fpuconfig(void);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   This function does not return anything explicitly.
 *
 ****************************************************************************/

    .type        riscv_fpuconfig, function

riscv_fpuconfig:
  li           a0, MSTATUS_FS_INIT
  csrs         CSR_STATUS, a0

  fscsr        zero

  fence.i
  ret

/****************************************************************************
 * Name: riscv_savefpu
 *
 * Description:
 *   Given the pointer to a register save area (in A0), save the state of the
 *   floating point registers.
 *
 * C Function Prototype:
 *   void riscv_savefpu(uintptr_t *regs, uintptr_t *fregs);
 *
 * Input Parameters:
 *   regs  - A pointer to the integer registers that contain the status
 *   fregs - A pointer to the register save area in which to save the
 *           floating point registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

    .type         riscv_savefpu, function

riscv_savefpu:

  REGLOAD    t0, REG_INT_CTX(a0)
  li         t1, MSTATUS_FS
  and        t2, t0, t1
  li         t1, MSTATUS_FS_DIRTY
#ifdef CONFIG_ARCH_LAZYFPU
  bne        t2, t1, 1f
#else
  blt        t2, t1, 1f
#endif
  li         t1, ~MSTATUS_FS
  and        t0, t0, t1
  li         t1, MSTATUS_FS_CLEAN
  or         t0, t0, t1
  REGSTORE   t0, REG_INT_CTX(a0)

  riscv_savefpu a1

1:
  ret

/****************************************************************************
 * Name: riscv_restorefpu
 *
 * Description:
 *   Given the pointer to a register save area (in A0), restore the state of
 *   the floating point registers.
 *
 * C Function Prototype:
 *   void riscv_restorefpu(uintptr_t *regs, uintptr_t *fregs);
 *
 * Input Parameters:
 *   regs  - A pointer to the integer registers that contain the status
 *   fregs - A pointer to the register save area containing the floating
 *           point registers.
 *
 * Returned Value:
 *   This function does not return anything explicitly.  However, it is
 *   called from interrupt level assembly logic that assumes that r0 is
 *   preserved.
 *
 ****************************************************************************/

    .type        riscv_restorefpu, function

riscv_restorefpu:

  REGLOAD      t0, REG_INT_CTX(a0)
  li           t1, MSTATUS_FS
  and          t2, t0, t1
  li           t1, MSTATUS_FS_INIT
  ble          t2, t1, 1f

  riscv_loadfpu a1

1:
  ret

#endif /* CONFIG_ARCH_FPU */
