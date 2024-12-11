/****************************************************************************
 * arch/risc-v/src/common/riscv_exception_common.S
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
#include <arch/irq.h>
#include <arch/mode.h>

#include <sys/types.h>

#ifdef CONFIG_LIB_SYSCALL
#  include <syscall.h>
#endif

#include "chip.h"

#include "riscv_percpu.h"

#include "riscv_macros.S"

/*
 * The riscv_extctx.S should be placed in the chip directory, for example:
 *   arch/risc-v/src/qemu-rv/riscv_extctx.S
 *
 * These macros should be provided by the chip-specific code:
 *   * save_extctx (extended context save)
 *   * load_extctx (extended context restore)
 *
 * Refer to the riscv_macros.S for implementation details.
 */

#ifdef CONFIG_ARCH_RISCV_INTXCPT_EXTENSIONS
#  include "riscv_extctx.S"
#endif

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

/* System calls require the per CPU scratch area */

#ifdef CONFIG_LIB_SYSCALL
#  ifndef CONFIG_RISCV_PERCPU_SCRATCH
#    error "CONFIG_RISCV_PERCPU_SCRATCH is needed for handling system calls"
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
  .global return_from_exception
  .global return_from_syscall
  .align  8

exception_common:
  .cfi_sections .debug_frame
  .cfi_startproc

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

  /* Save the extended context */

#ifdef CONFIG_ARCH_RISCV_INTXCPT_EXTENSIONS
  save_extctx sp
#endif

  csrr       s0, CSR_STATUS       /* s0=status */
  csrr       s1, CSR_EPC          /* s1=exception PC */
  csrr       s2, CSR_CAUSE        /* s2=cause */

#ifdef CONFIG_ARCH_KERNEL_STACK
  csrr       s3, CSR_SCRATCH
  REGLOAD    s3, RISCV_PERCPU_USP(s3)
#else
  addi       s3, sp, XCPTCONTEXT_SIZE
#endif

  REGSTORE   s0, REG_INT_CTX(sp)
  REGSTORE   s1, REG_EPC(sp)
  REGSTORE   s3, REG_SP(sp)

#ifdef CONFIG_LIB_SYSCALL
  /* Check whether it is an exception or interrupt */

  blt        s2, x0, handle_irq   /* If cause < 0 it is interrupt */

  /* Is it a system call ? */

  li         s3, RISCV_IRQ_ECALLU /* Is it a system call ? */
  bne        s2, s3, handle_irq

  /* Is it one of the reserved system calls ? */

  li         s3, CONFIG_SYS_RESERVED
  blt        a0, s3, handle_irq   /* If a0 < CONFIG_SYS_RESERVED */

  /* It is a system call, re-enable interrupts if they were enabled */

  andi       s3, s0, STATUS_PIE
  beqz       s3, 1f
  csrs       CSR_STATUS, STATUS_IE

1:
  addi       s1, s1, 0x4          /* Must move EPC forward by +4 */
  REGSTORE   s1, REG_EPC(sp)      /* Updated EPC to user context */

  csrr       tp, CSR_SCRATCH      /* Load kernel TP */
  REGLOAD    tp, RISCV_PERCPU_TCB(tp)

  mv         a7, sp               /* a7 = context */
  call       x1, dispatch_syscall /* Dispatch the system call */

return_from_syscall:

  /* System call is done, disable interrupts */

  csrc       CSR_STATUS, STATUS_IE

  /* Clean up after system call */

  REGSTORE   a0, REG_A0(sp)       /* Syscall return value to user context */
  mv         a0, sp               /* Return to same context */
  tail       return_from_exception

handle_irq:
#endif

  /* Setup arg0(exception cause), arg1(context) */

  mv         a0, s2               /* exception cause */
  mv         a1, sp               /* context = sp */
  mv         fp, a1               /* Use fp to debug frame */
  .cfi_def_cfa fp, 0              /* Register in fp, so we just set fp as frame */
  .cfi_offset x2, 8               /* Toolchain not support macro, is REG_X2 * 4 */
  .cfi_offset ra, 0               /* Toolchain not support macro, is REG_EPC * 4 */

#if CONFIG_ARCH_INTERRUPTSTACK > 15

  /* Switch to interrupt stack */

  setintstack t0, t1

  /* Call interrupt handler in C */

  jal        x1, riscv_dispatch_irq

#else
  /* Reserve some space for current_regs if interrupt stack disabled */

  addi       sp, sp, -XCPTCONTEXT_SIZE

  /* Call interrupt handler in C */

  jal        x1, riscv_dispatch_irq

  /* Restore sp */

  addi       sp, sp, XCPTCONTEXT_SIZE
#endif

return_from_exception:

  /* If context switch is needed, return a new sp */

  mv         sp, a0

  REGLOAD    s0, REG_EPC(sp)      /* restore sepc */
  csrw       CSR_EPC, s0

  REGLOAD    s0, REG_INT_CTX(sp)  /* restore status */
  csrw       CSR_STATUS, s0

#ifdef CONFIG_LIB_SYSCALL
  /* Store tcb to scratch register */

  call       x1, nxsched_self
  csrr       s1, CSR_SCRATCH
  REGSTORE   a0, RISCV_PERCPU_TCB(s1)
#endif

#ifdef CONFIG_ARCH_KERNEL_STACK
  /* Returning to userspace ? */

  li         s1, STATUS_PPP
  and        s0, s0, s1
  bnez       s0, 1f

  /* Set the unwound kernel stack to the scratch area */

  addi       a0, sp, XCPTCONTEXT_SIZE
  csrr       s0, CSR_SCRATCH
  REGSTORE   a0, RISCV_PERCPU_KSP(s0)

1:
#endif

  /* Restore the extended context */

#ifdef CONFIG_ARCH_RISCV_INTXCPT_EXTENSIONS
  load_extctx sp
#endif

  load_ctx   sp

  REGLOAD    sp, REG_SP(sp)      /* restore original sp */

  /* Return from exception */

  ERET
  .cfi_endproc

/****************************************************************************
 * Function: riscv_jump_to_user
 *
 * Description:
 *  Routine to jump to user space, called when a user process is started and
 *  the kernel is ready to give control to the user task in user space.
 *
 * riscv_jump_to_user(entry, a0, a1, a2, sp, regs)
 *     entry:  process entry point
 *     a0:     parameter 0 for process
 *     a1:     parameter 1 for process
 *     a2:     parameter 2 for process
 *     sp:     user stack pointer
 *     regs:   integer register save area to use
 *
 ****************************************************************************/

  .section EXCEPTION_SECTION
  .global riscv_jump_to_user

riscv_jump_to_user:

  csrc     CSR_STATUS, STATUS_IE
  mv       sp, a5
  REGSTORE a0, REG_EPC(sp)
  REGSTORE a1, REG_A0(sp)
  REGSTORE a2, REG_A1(sp)
  REGSTORE a3, REG_A2(sp)
  REGSTORE a4, REG_SP(sp)
  csrr     a0, CSR_STATUS
  li       a1, ~STATUS_PPP
  and      a0, a0, a1
  REGSTORE a0, REG_INT_CTX(sp)
  mv       a0, sp
  tail return_from_exception

/*****************************************************************************
 *  Name: g_intstackalloc and g_intstacktop
 ****************************************************************************/

/* Total required interrupt stack size */

#define STACK_ALLOC_SIZE (INT_STACK_SIZE * CONFIG_SMP_NCPUS)

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
