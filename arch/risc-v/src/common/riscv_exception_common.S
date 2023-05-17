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

#include <sys/types.h>

#include "chip.h"

#include "riscv_percpu.h"

#include "riscv_macros.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Using address environments requires that the per-process kernel stack is
 * enabled. Using user stack to run exception and/or kernel code is a very
 * very bad idea, thus enforce the kernel stack
 */

#ifdef CONFIG_ARCH_ADDRENV
#  ifndef CONFIG_ARCH_KERNEL_STACK
#    error "Kernel stack is needed for handling exceptions"
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

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* Take the kernel stack into use */

  csrrw      a0, CSR_SCRATCH, a0
  REGSTORE   sp, RISCV_PERCPU_USP(a0)
  REGLOAD    sp, RISCV_PERCPU_KSP(a0)
  REGSTORE   x0, RISCV_PERCPU_KSP(a0)
  bnez       sp, 1f

  /* No kernel stack, exception comes from kernel */

  REGLOAD    sp, RISCV_PERCPU_USP(a0)

1:
  /* Restore the per-cpu structure */

  csrrw      a0, CSR_SCRATCH, a0
#endif

  addi       sp, sp, -XCPTCONTEXT_SIZE
  save_ctx   sp

  csrr       s0, CSR_STATUS
  REGSTORE   s0, REG_INT_CTX(sp)  /* status */

#ifdef CONFIG_ARCH_KERNEL_STACK
  csrr       s0, CSR_SCRATCH
  REGLOAD    s0, RISCV_PERCPU_USP(s0)
#else
  addi       s0, sp, XCPTCONTEXT_SIZE
#endif

  REGSTORE   s0, REG_X2(sp)       /* original SP */

  csrr       s0, CSR_EPC
  REGSTORE   s0, REG_EPC(sp)      /* exception PC */

  riscv_savefpu   sp

  /* Setup arg0(exception cause), arg1(context) */

  csrr       a0, CSR_CAUSE        /* exception cause */
  mv         a1, sp               /* context = sp */

#if CONFIG_ARCH_INTERRUPTSTACK > 15

  /* Switch to interrupt stack */

  setintstack t0, t1

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

  /* If context switch is needed, return a new sp */

  mv         sp, a0

  riscv_loadfpu   sp

  REGLOAD    s0, REG_EPC(sp)      /* restore sepc */
  csrw       CSR_EPC, s0

  REGLOAD    s0, REG_INT_CTX(sp)  /* restore sstatus */
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

  REGLOAD    sp, REG_SP(sp)      /* restore original sp */

  /* Return from exception */

  ERET

/*****************************************************************************
 *  Name: g_intstackalloc and g_intstacktop
 ****************************************************************************/

/* Total required interrupt stack size */

#define STACK_ALLOC_SIZE (CONFIG_ARCH_INTERRUPTSTACK * CONFIG_SMP_NCPUS)

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  .bss
  .balign 16
  .global g_intstackalloc
  .global g_intstacktop
  .type   g_intstackalloc, object
  .type   g_intstacktop, object
g_intstackalloc:
  .skip  STACK_ALIGN_UP(STACK_ALLOC_SIZE)
g_intstacktop:
  .size  g_intstacktop, 0
  .size  g_intstackalloc, STACK_ALIGN_DOWN(STACK_ALLOC_SIZE)
#endif
