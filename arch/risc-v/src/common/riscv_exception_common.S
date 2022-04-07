/****************************************************************************
 * arch/risc-v/src/common/riscv_exception_common.S
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
#include <arch/irq.h>
#include <arch/mode.h>

#include "chip.h"
#include "riscv_macros.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Using address environments currently require that a common interrupt stack
 * is in place. This is needed because during context switch the procedure
 * that swaps the active address environment is dependent on a stack, which
 * must be a 'neutral' stack.
 *
 * Another option would be to use a per-process kernel stack, but full
 * support for this is not yet in place, so use the common IRQ stack instead.
 */

#ifdef CONFIG_ARCH_ADDRENV
#  if CONFIG_ARCH_INTERRUPTSTACK == 0 && !defined(CONFIG_ARCH_KERNEL_STACK)
#    error "IRQ or kernel stack is needed for swapping address environments"
#  endif
#endif

/* Provide a default section for the exeception handler. */

#ifndef EXCEPTION_SECTION
#  define EXCEPTION_SECTION .text
#endif

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

/****************************************************************************
 * Name: exception_common
 *
 * Description:
 *   Handles interrupts. If kernel is in S-mode, handles delegated interrupts
 *   in S-mode interrupt handler.
 *
 ****************************************************************************/

  .section EXCEPTION_SECTION
  .global exception_common
  .align  8

exception_common:

  addi       sp, sp, -XCPTCONTEXT_SIZE
  save_ctx   sp

  csrr       s0, CSR_STATUS
  REGSTORE   s0, REG_INT_CTX(sp)  /* status */

  addi       s0, sp, XCPTCONTEXT_SIZE
  REGSTORE   s0, REG_X2(sp)       /* original SP */

  csrr       s0, CSR_EPC
  REGSTORE   s0, REG_EPC(sp)      /* exception PC */

#ifdef CONFIG_ARCH_FPU
  mv         a0, sp
  jal        x1, riscv_savefpu    /* save FPU context */
#endif

  /* Setup arg0(exception cause), arg1(context) */

  csrr       a0, CSR_CAUSE        /* exception cause */
  mv         a1, sp               /* context = sp */

#if CONFIG_ARCH_INTERRUPTSTACK > 15

  /* Offset to hartid */

  mv         s0, a0               /* save cause */
#ifdef CONFIG_SMP
  jal        x1, riscv_mhartid    /* get hartid */
#else
  li         a0, 0
#endif

  /* Switch to interrupt stack */

#if IRQ_NSTACKS > 1
  li         t0, (CONFIG_ARCH_INTERRUPTSTACK & ~15)
  mul        t0, a0, t0
  la         a0, g_intstacktop
  sub        sp, a0, t0
#else
  la         sp, g_intstacktop
#endif

  mv         a0, s0               /* restore cause */

  /* Call interrupt handler in C */

  jal        x1, riscv_dispatch_irq

#else
  /* Reserve some space for CURRENT_REGS if interrupt stack disabled */

  addi       sp, sp, -XCPTCONTEXT_SIZE

  /* Call interrupt handler in C */

  jal        x1, riscv_dispatch_irq

  /* Restore sp */

  addi       sp, sp, XCPTCONTEXT_SIZE
#endif

#ifdef CONFIG_ARCH_FPU
  jal        x1, riscv_restorefpu /* restore FPU context */
#endif

  /* If context switch is needed, return a new sp */

  mv         sp, a0

  REGLOAD    s0, REG_EPC(sp)      /* restore sepc */
  csrw       CSR_EPC, s0

  REGLOAD    s0, REG_INT_CTX(sp)  /* restore sstatus */
  csrw       CSR_STATUS, s0

  load_ctx   sp

  REGLOAD    sp, REG_SP(sp)       /* restore original sp */

  /* Return from exception */

  ERET

/*****************************************************************************
 *  Name: g_intstackalloc and g_intstacktop
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  .bss
  .balign 16
  .global g_intstackalloc
  .global g_intstacktop
  .type   g_intstackalloc, object
  .type   g_intstacktop, object
g_intstackalloc:
  .skip  (((CONFIG_ARCH_INTERRUPTSTACK * IRQ_NSTACKS) + 8) & ~15)
g_intstacktop:
  .size  g_intstacktop, 0
  .size  g_intstackalloc, ((CONFIG_ARCH_INTERRUPTSTACK * IRQ_NSTACKS) & ~15)
#endif
