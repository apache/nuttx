/****************************************************************************
 * arch/risc-v/src/nuttsbi/sbi_machine_trap.S
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
#include <nuttx/irq.h>

#include "riscv_macros.S"

#include "riscv_internal.h"

#include "sbi_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

/****************************************************************************
 * Name: machine_trap
 *
 * Description:
 *   Machine mode trap handler. Handles Mtimer and simple mcall interface.
 *
 ****************************************************************************/

  .section .text
  .global  machine_trap
  .align   8

machine_trap:

  /* Switch to M-mode IRQ stack */

  csrrw      sp, mscratch, sp     /* mscratch has user stack */
  beqz       sp, .Lmtrap          /* Detect recursive traps */

  addi       sp, sp, -XCPTCONTEXT_SIZE
  save_ctx   sp

  csrr       a0, mcause           /* exception cause  */

  csrrw      s0, mscratch, x0     /* read user stack */
  REGSTORE   s0, REG_X2(sp)       /* original SP      */

  /* Check if this is an exception */

  bgez       a0, .Lmexception

  /* Figure out which interrupt this is */

  sll        a0, a0, 1            /* Shift msbit out */
  li         a1, MTIMER_IRQ * 2   /* Machine timer irq ? (shifted left) */
  bne        a0, a1, 1f

  /* Delegate interrupt to S-mode handler */

  li         a0, MIP_MTIP
  csrc       mie, a0
  li         a0, MIP_STIP
  csrs       mip, a0

1:
  /* Restore mscratch */

  addi       s0, sp, XCPTCONTEXT_SIZE
  csrw       mscratch, s0         /* original mscratch */

  /* Restore original context */

  load_ctx   sp

  REGLOAD    sp, REG_SP(sp)       /* restore original sp */

  /* Return from Machine Interrupt */

  mret

.Lmexception:

  /* Handle exception, only accepted source is ecall */

  sll        a0, a0, 1
  li         a1, RISCV_IRQ_ECALLS * 2
  bne        a0, a1, .Lmtrap

  /* Handle ecall */

  mv         a0, sp
  jal        x1, sbi_mcall_handle
  csrr       a0, mepc
  addi       a0, a0, 4
  csrw       mepc, a0
  j          1b

  /* An unhandled trap to M-mode: this is an error and we cannot proceed */

.Lmtrap:

  csrr        a0, mcause         /* Interrupt cause [arg0] */
  csrr        a1, mepc           /* Interrupt PC (instruction) [arg1] */
  jal         x1, sbi_mexception
  j           __start

.Lmtraploop:

  /* If we somehow get here, prevent the software from running away */

  nop
  nop
  j           .Lmtraploop

/*****************************************************************************
 *  Name: m_intstackalloc / m_intstacktop
 *
 *  Description:
 *    Allocate separate stack(s) for machine mode interrupts, only if kernel
 *    runs in S-mode. Separate stacks are needed as M-mode interrupts can
 *    pre-empt S-mode interrupts.
 *
 ****************************************************************************/

#define STACK_ALLOC_SIZE (MMODE_IRQSTACK * CONFIG_SMP_NCPUS)

  .bss
  .balign 16
  .global g_mintstackalloc
  .global g_mintstacktop
  .type   g_mintstackalloc, object
  .type   g_mintstacktop, object
g_mintstackalloc:
  .skip  STACK_ALIGN_UP(STACK_ALLOC_SIZE)
g_mintstacktop:
  .size  g_mintstacktop, 0
  .size  g_mintstackalloc, STACK_ALIGN_DOWN(STACK_ALLOC_SIZE)
