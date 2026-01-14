/****************************************************************************
 * arch/risc-v/src/common/riscv_vpu.S
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <arch/arch.h>
#include <arch/csr.h>
#include <arch/irq.h>
#include <arch/mode.h>

#include "riscv_macros.S"

#ifdef CONFIG_ARCH_RV_ISA_V

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .globl        riscv_vpuconfig
    .globl        riscv_savevpu
    .globl        riscv_restorevpu
    .file         "riscv_vpu.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_vpuconfig
 *
 * Description:
 *   init vpu
 *
 * C Function Prototype:
 *   void riscv_vpuconfig(void);
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   This function does not return anything explicitly.
 *
 ****************************************************************************/

    .type        riscv_vpuconfig, function

riscv_vpuconfig:
  li           a0, MSTATUS_VS_INIT
  csrs         CSR_STATUS, a0

  fence.i
  ret

/****************************************************************************
 * Name: riscv_savevpu
 *
 * Description:
 *   Given the pointer to a register save area (in A1), save the state of the
 *   vector point registers.
 *
 * C Function Prototype:
 *   void riscv_savevpu(uintptr_t *regs, uintptr_t *fregs);
 *
 * Input Parameters:
 *   regs  - A pointer to the integer registers that contain the status
 *   fregs - A pointer to the register save area in which to save the
 *           vector point registers
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

    .type         riscv_savevpu, function

riscv_savevpu:

  REGLOAD    t0, REG_INT_CTX(a0)
  li         t1, MSTATUS_VS
  and        t2, t0, t1
  li         t1, MSTATUS_VS_DIRTY
#ifdef CONFIG_ARCH_LAZYVPU
  bne        t2, t1, 1f
#else
  blt        t2, t1, 1f
#endif
  li         t1, ~MSTATUS_VS
  and        t0, t0, t1
  li         t1, MSTATUS_VS_CLEAN
  or         t0, t0, t1
  REGSTORE   t0, REG_INT_CTX(a0)

  riscv_savevpu a1

1:
  ret

/****************************************************************************
 * Name: riscv_restorevpu
 *
 * Description:
 *   Given the pointer to a register save area (in A1), restore the state of
 *   the vector point registers.
 *
 * C Function Prototype:
 *   void riscv_restorevpu(uintptr_t *regs, uintptr_t *fregs);
 *
 * Input Parameters:
 *   regs  - A pointer to the integer registers that contain the status
 *   fregs - A pointer to the register save area containing the vector
 *           point registers.
 *
 * Returned Value:
 *   This function does not return anything explicitly.  However, it is
 *   called from interrupt level assembly logic that assumes that r0 is
 *   preserved.
 *
 ****************************************************************************/

    .type        riscv_restorevpu, function

riscv_restorevpu:

  REGLOAD      t0, REG_INT_CTX(a0)
  li           t1, MSTATUS_VS
  and          t2, t0, t1
  li           t1, MSTATUS_VS_INIT
  ble          t2, t1, 1f

  riscv_loadvpu a1

1:
  ret

#endif /* CONFIG_ARCH_RV_ISA_V */
