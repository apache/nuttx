/****************************************************************************
 * arch/risc-v/src/common/supervisor/riscv_syscall.S
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

.file "riscv_syscall.S"

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/mode.h>

#include "chip.h"

#include "riscv_macros.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

    .globl  sys_call0
    .globl  sys_call1
    .globl  sys_call2
    .globl  sys_call3
    .globl  sys_call4
    .globl  sys_call5
    .globl  sys_call6

/****************************************************************************
 * Name: sys_callx
 *
 * Description:
 *   Dispatch syscall from kernel
 *
 * C Function Prototype:
 *   uintptr_t sys_callx(unsigned int nbr, ...);
 *
 * Input Parameters:
 *   Assumes the context to return is already set up
 *
 * Returned Value:
 *   Return value of system call is returned into contex
 *
 * Assumptions:
 *   Task is running in privileged mode
 *
 ****************************************************************************/

.type sys_call0, function
.type sys_call1, function
.type sys_call2, function
.type sys_call3, function
.type sys_call4, function
.type sys_call5, function
.type sys_call6, function

sys_call0:
sys_call1:
sys_call2:
sys_call3:
sys_call4:
sys_call5:
sys_call6:

  addi       sp, sp, -XCPTCONTEXT_SIZE /* make room */
  save_ctx   sp                        /* save current context */

  /* Mask interrupts and store the status register to context */

  li         s1, STATUS_IE             /* move IE -> PIE */
  csrrc      s0, CSR_STATUS, s1
  and        s1, s0, s1                /* if (STATUS & IE) */
  beqz       s1, 1f
  li         s1, ~STATUS_IE            /* clear IE */
  and        s0, s0, s1
  li         s1, STATUS_PIE            /* set PIE */
  or         s0, s0, s1

 1:
  /* Set previous privilege, we are in privileged mode now */

  li         s1, STATUS_PPP            /* set previous privilege */
  or         s0, s0, s1
  REGSTORE   s0, REG_INT_CTX(sp)       /* store status to context */

  REGSTORE   x1, REG_EPC(sp)           /* save ra to epc */

  addi       s0, sp, XCPTCONTEXT_SIZE
  REGSTORE   s0, REG_SP(sp)            /* original SP */

  riscv_savefpu sp

  mv         a0, sp                    /* a0 = context */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  /* Switch to interrupt stack */

  setintstack t0, t1
#endif

  /* Run the handler */

  jal        x1, riscv_perform_syscall

  /* Restore (potentially new) context */

  mv         sp, a0                    /* use sp, as a0 gets wiped */

  riscv_loadfpu sp

  REGLOAD    s0, REG_EPC(sp)           /* restore epc */
  csrw       CSR_EPC, s0

  /* Restore status register, but don't enable interrupts yet */

  REGLOAD    s0, REG_INT_CTX(sp)       /* restore status */
  li         s1, STATUS_IE             /* move IE -> PIE */
  and        s1, s0, s1                /* if (STATUS & IE) */
  beqz       s1, 1f
  li         s1, ~STATUS_IE            /* clear IE */
  and        s0, s0, s1
  li         s1, STATUS_PIE            /* set PIE */
  or         s0, s0, s1

1:
  csrw       CSR_STATUS, s0

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* Returning to userspace ? */

  li         s1, STATUS_PPP
  and        s0, s0, s1
  bnez       s0, 1f

  /* Set the next task's kernel stack to the scratch area */

  jal        x1, riscv_current_ksp
  csrr       s0, CSR_SCRATCH
  REGSTORE   a0, RISCV_PERCPU_KSP(s0)

1:
#endif

  load_ctx   sp

  REGLOAD    sp, REG_SP(sp)            /* restore original sp */

  /* return from exception, which updates the status register */

  ERET

  .size sys_call0, .-sys_call0
  .size sys_call1, .-sys_call1
  .size sys_call2, .-sys_call2
  .size sys_call3, .-sys_call3
  .size sys_call4, .-sys_call4
  .size sys_call5, .-sys_call5
  .size sys_call6, .-sys_call6

  .end
