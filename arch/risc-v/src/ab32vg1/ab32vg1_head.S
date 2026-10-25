/****************************************************************************
 * arch/risc-v/src/ab32vg1/ab32vg1_head.S
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
#include <arch/irq.h>
#include <arch/mode.h>

#include "chip.h"
#include "ab32vg1.h"
#include "riscv_macros.S"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Trap kinds passed to ab32vg1_dispatch_trap() */

#define TRAP_EXCEPTION      0
#define TRAP_IRQ            1
#define TRAP_SYSCALL        2

/* Status word stored in REG_INT_CTX: STATUS_PIE mirrors EPICCON.IE, so the
 * common code (signal delivery, swint) can keep using the standard bits.
 */

#define INTCTX_PIE_SHIFT    7               /* log2(STATUS_PIE) */

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .global __start
  .global ab32vg1_vectors
  .global ab32vg1_syscall_trap
  .global ab32vg1_trap_return
  .global up_saveusercontext

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   Image entry point.  The mask ROM UART loader calls the image at its
 *   load address as a command handler, from the ROM stack at the top of
 *   SRAM; a boot header loader jumps to it.  Never returns.
 *
 ****************************************************************************/

  .section .start, "ax"
  .type   __start, function

__start:

  /* Mask all interrupts: the ROM leaves its HSUART interrupt enabled */

  sw      zero, AB32VG1_PICEN(zero)
  li      t0, (PICCON_IE | PICCON_EN)
  sw      t0, AB32VG1_PICCONCLR(zero)

  /* Install our vector table */

  la      t0, ab32vg1_vectors
  sw      t0, AB32VG1_PICADR(zero)

  /* Set the stack pointer to the idle thread stack */

  la      sp, AB32VG1_IDLESTACK_TOP

  /* Jump to __ab32vg1_start; it never returns */

  jal     x1, __ab32vg1_start

1:
  j       1b

  .size   __start, . - __start

/****************************************************************************
 * Name: ab32vg1_vectors
 *
 * Description:
 *   Vector table pointed by PICADR.  Entries are fixed offsets from the
 *   base: +0x10 exceptions (and high priority interrupts, unused), +0x40
 *   low priority interrupts, +0x80 XIP cache miss (unused, the image runs
 *   from RAM).
 *
 ****************************************************************************/

  .section .text.ab32vg1_vectors, "ax"
  .balign 1024
  .type   ab32vg1_vectors, function

ab32vg1_vectors:
  j       exception_entry               /* +0x00: not expected */
  .org    ab32vg1_vectors + AB32VG1_VEC_EXCEPTION
  j       exception_entry
  .org    ab32vg1_vectors + AB32VG1_VEC_IRQ
  j       irq_entry
  .org    ab32vg1_vectors + AB32VG1_VEC_CACHE
  j       exception_entry

  .size   ab32vg1_vectors, . - ab32vg1_vectors

/****************************************************************************
 * Name: trap_entry
 *
 * Description:
 *   Save the full context on the current stack and call
 *   ab32vg1_dispatch_trap(kind, regs).  Used by the hardware vectors.
 *
 ****************************************************************************/

.macro trap_entry kind

  addi    sp, sp, -XCPTCONTEXT_SIZE
  save_ctx sp

  lw      s0, AB32VG1_EPICCON(zero)     /* Previous interrupt enable */
  andi    s0, s0, PICCON_IE
  slli    s0, s0, INTCTX_PIE_SHIFT
  li      s1, STATUS_PPP
  or      s0, s0, s1
  REGSTORE s0, REG_INT_CTX(sp)

  lw      s1, AB32VG1_EPC(zero)         /* Return address */
  REGSTORE s1, REG_EPC(sp)

  addi    s2, sp, XCPTCONTEXT_SIZE      /* Interrupted sp */
  REGSTORE s2, REG_SP(sp)

  li      a0, \kind
  j       trap_dispatch

.endm

  .section .text.ab32vg1_trap, "ax"

exception_entry:
  trap_entry TRAP_EXCEPTION

irq_entry:
  trap_entry TRAP_IRQ

/****************************************************************************
 * Name: ab32vg1_syscall_trap
 *
 * Description:
 *   Software equivalent of ecall, reached through RISCV_SYSCALL_INSN with
 *   "jal ra, ab32vg1_syscall_trap" (the caller preserves its own ra).
 *   Builds the same frame as a hardware trap with interrupts masked.  The
 *   saved EPC is ra - 4 because riscv_doirq() advances it by 4, as it does
 *   for a real ecall.
 *
 ****************************************************************************/

  .type   ab32vg1_syscall_trap, function

ab32vg1_syscall_trap:

  addi    sp, sp, -XCPTCONTEXT_SIZE
  save_ctx sp

  lw      s0, AB32VG1_PICCON(zero)      /* Current interrupt enable */
  li      s1, PICCON_IE
  sw      s1, AB32VG1_PICCONCLR(zero)   /* Mask interrupts */
  andi    s0, s0, PICCON_IE
  slli    s0, s0, INTCTX_PIE_SHIFT
  li      s1, STATUS_PPP
  or      s0, s0, s1
  REGSTORE s0, REG_INT_CTX(sp)

  addi    s1, ra, -4
  REGSTORE s1, REG_EPC(sp)

  addi    s2, sp, XCPTCONTEXT_SIZE
  REGSTORE s2, REG_SP(sp)

  li      a0, TRAP_SYSCALL

  /* Fall through */

/****************************************************************************
 * Name: trap_dispatch
 *
 * Description:
 *   a0 = trap kind, sp = register frame.  Switch to the interrupt stack,
 *   call the C dispatcher and return to the context it selects.
 *
 ****************************************************************************/

trap_dispatch:

  mv      a1, sp                        /* a1 = regs */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  setintstack t0, t1
#endif

  call    ab32vg1_dispatch_trap

  /* Fall through with a0 = the register frame to restore */

/****************************************************************************
 * Name: ab32vg1_trap_return
 *
 * Description:
 *   Restore the context in a0 and return to it.  EPC and EPICCON are
 *   written before mret; interrupts are masked here, either by hardware
 *   (inside a trap) or by ab32vg1_syscall_trap.
 *
 ****************************************************************************/

ab32vg1_trap_return:

  mv      sp, a0

  REGLOAD s0, REG_EPC(sp)
  sw      s0, AB32VG1_EPC(zero)

  REGLOAD s0, REG_INT_CTX(sp)
  srli    s0, s0, INTCTX_PIE_SHIFT
  andi    s0, s0, PICCON_IE
  sw      s0, AB32VG1_EPICCON(zero)

  load_ctx sp
  REGLOAD sp, REG_SP(sp)

  mret

  .size   ab32vg1_syscall_trap, . - ab32vg1_syscall_trap

/****************************************************************************
 * Name: up_saveusercontext
 *
 * Description:
 *   Save the current register context (used by the assertion logic).
 *
 ****************************************************************************/

  .type   up_saveusercontext, function

up_saveusercontext:

  save_ctx a0

  lw      a1, AB32VG1_PICCON(zero)
  andi    a1, a1, PICCON_IE
  slli    a1, a1, INTCTX_PIE_SHIFT
  REGSTORE a1, REG_INT_CTX(a0)

  REGSTORE sp, REG_X2(a0)
  REGSTORE x1, REG_EPC(a0)

  li      a0, 0
  ret

  .size   up_saveusercontext, . - up_saveusercontext

/****************************************************************************
 * Name: g_intstackalloc and g_intstacktop
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  .bss
  .balign 16
  .global g_intstackalloc
  .global g_intstacktop
  .type   g_intstackalloc, object
  .type   g_intstacktop, object
g_intstackalloc:
  .skip   STACKFRAME_ALIGN_UP(INT_STACK_SIZE)
g_intstacktop:
  .size   g_intstacktop, 0
  .size   g_intstackalloc, STACKFRAME_ALIGN_DOWN(INT_STACK_SIZE)
#endif
