/****************************************************************************
 * arch/risc-v/src/k210/k210_head.S
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
#include <arch/rv64gc/irq.h>

#include "chip.h"
#include "k210_memorymap.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .global exception_common

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start

__start:

  /* Load mhartid (cpuid) */

  csrr a0, mhartid

  /* Set stack pointer to the idle thread stack */

  bnez a0, 1f
  la   sp, K210_IDLESTACK0_TOP
  j    2f
1:
  la   sp, K210_IDLESTACK1_TOP

  /* In case of single CPU config, stop here */

#if !(defined CONFIG_SMP) || (CONFIG_SMP_NCPUS == 1)
  wfi
#endif
2:

  /* Disable all interrupts (i.e. timer, external) in mie */

  csrw mie, zero

  /* Initialize the Machine Trap Vector */

  la   t0, __trap_vec
  csrw mtvec, t0

  /* Jump to __k210_start with mhartid */

  j    __k210_start

  /* We shouldn't return from __k210_start */

  .global _init
  .global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret

/****************************************************************************
 * Name: exception_common
 ****************************************************************************/

exception_common:

#if 0
  csrr gp, mcause   /* exception cause */
  addi tp, zero, 10 /* 10 = machine ecall */
  bgtu gp, tp, normal_irq
  ld   sp, g_fstack_top /* Set sp to fault stack */

normal_irq:
  addi gp, zero, 0  /* clear  */
#endif

  addi sp, sp, -XCPTCONTEXT_SIZE

  sd   x1,  1*8(sp)   /* ra */
  sd   x3,  3*8(sp)   /* gp */
  sd   x4,  4*8(sp)   /* tp */
  sd   x5,  5*8(sp)   /* t0 */
  sd   x6,  6*8(sp)   /* t1 */
  sd   x7,  7*8(sp)   /* t2 */
  sd   x8,  8*8(sp)   /* s0 */
  sd   x9,  9*8(sp)   /* s1 */
  sd   x10, 10*8(sp)  /* a0 */
  sd   x11, 11*8(sp)  /* a1 */
  sd   x12, 12*8(sp)  /* a2 */
  sd   x13, 13*8(sp)  /* a3 */
  sd   x14, 14*8(sp)  /* a4 */
  sd   x15, 15*8(sp)  /* a5 */
  sd   x16, 16*8(sp)  /* a6 */
  sd   x17, 17*8(sp)  /* a7 */
  sd   x18, 18*8(sp)  /* s2 */
  sd   x19, 19*8(sp)  /* s3 */
  sd   x20, 20*8(sp)  /* s4 */
  sd   x21, 21*8(sp)  /* s5 */
  sd   x22, 22*8(sp)  /* s6 */
  sd   x23, 23*8(sp)  /* s7 */
  sd   x24, 24*8(sp)  /* s8 */
  sd   x25, 25*8(sp)  /* s9 */
  sd   x26, 26*8(sp)  /* s10 */
  sd   x27, 27*8(sp)  /* s11 */
  sd   x28, 28*8(sp)  /* t3 */
  sd   x29, 29*8(sp)  /* t4 */
  sd   x30, 30*8(sp)  /* t5 */
  sd   x31, 31*8(sp)  /* t6 */

  csrr s0, mstatus
  sd   s0,  32*8(sp)  /* mstatus */

  addi s0, sp, XCPTCONTEXT_SIZE
  sd   s0,  2*8(sp)   /* original SP */

  /* Setup arg0(exception cause), arg1(context) */

  csrr a0, mcause  /* exception cause */
  csrr s0, mepc
  sd   s0, 0(sp)   /* exception PC */

  mv   a1, sp      /* context = sp */

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  /* Load mhartid (cpuid) */

  csrr s0, mhartid

  /* Switch to interrupt stack */

  bnez s0, 3f
  la   sp, g_intstacktop
  j    4f
3:
  la   sp, g_intstacktop
  addi sp, sp, -(CONFIG_ARCH_INTERRUPTSTACK & ~15)
4:

#endif

  /* Call interrupt handler in C */

  jal  x1, k210_dispatch_irq

  /* If context switch is needed, return a new sp */

  mv   sp, a0
  ld   s0, 0(sp)    /* restore mepc */
  csrw mepc, s0

  ld   s0, 32*8(sp) /* restore mstatus */
  csrw mstatus, s0

  ld  x3,  3*8(sp)  /* gp */
  ld  x4,  4*8(sp)  /* tp */
  ld  x5,  5*8(sp)  /* t0 */
  ld  x6,  6*8(sp)  /* t1 */
  ld  x7,  7*8(sp)  /* t2 */
  ld  x8,  8*8(sp)  /* s0 */
  ld  x9,  9*8(sp)  /* s1 */
  ld x10, 10*8(sp)  /* a0 */
  ld x11, 11*8(sp)  /* a1 */
  ld x12, 12*8(sp)  /* a2 */
  ld x13, 13*8(sp)  /* a3 */
  ld x14, 14*8(sp)  /* a4 */
  ld x15, 15*8(sp)  /* a5 */
  ld x16, 16*8(sp)  /* a6 */
  ld x17, 17*8(sp)  /* a7 */
  ld x18, 18*8(sp)  /* s2 */
  ld x19, 19*8(sp)  /* s3 */
  ld x20, 20*8(sp)  /* s4 */
  ld x21, 21*8(sp)  /* s5 */
  ld x22, 22*8(sp)  /* s6 */
  ld x23, 23*8(sp)  /* s7 */
  ld x24, 24*8(sp)  /* s8 */
  ld x25, 25*8(sp)  /* s9 */
  ld x26, 26*8(sp)  /* s10 */
  ld x27, 27*8(sp)  /* s11 */
  ld x28, 28*8(sp)  /* t3 */
  ld x29, 29*8(sp)  /* t4 */
  ld x30, 30*8(sp)  /* t5 */
  ld x31, 31*8(sp)  /* t6 */

  ld  x1,  1*8(sp)  /* ra */

  ld  sp,  2*8(sp)  /* restore original sp */

  /* Return from Machine Interrupt */

  mret

/************************************************************************************
 *  Name: g_intstackalloc and g_intstacktop
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  .bss
  .balign 16
  .global g_intstackalloc
  .global g_intstacktop
  .type   g_intstackalloc, object
  .type   g_intstacktop, object
g_intstackalloc:
#ifndef CONFIG_SMP
  .skip  ((CONFIG_ARCH_INTERRUPTSTACK + 8) & ~15)
#else
  .skip  (((CONFIG_ARCH_INTERRUPTSTACK * CONFIG_SMP_NCPUS) + 8) & ~15)
#endif
g_intstacktop:
  .size  g_intstacktop, 0
#ifndef CONFIG_SMP
  .size  g_intstackalloc, (CONFIG_ARCH_INTERRUPTSTACK & ~15)
#else
  .size  g_intstackalloc, ((CONFIG_ARCH_INTERRUPTSTACK * CONFIG_SMP_NCPUS) & ~15)
#endif
#endif
