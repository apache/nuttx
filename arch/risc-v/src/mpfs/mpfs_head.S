/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_head.S
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
#include <arch/csr.h>
#include <arch/rv64gc/irq.h>

#include "chip.h"
#include "mpfs_memorymap.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  .global exception_common

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start_mpfs

__start_mpfs:


  /* Disable all interrupts (i.e. timer, external) in mie */

  csrw mie, zero
  csrw mip, zero

  /* Initialize the Machine Trap Vector */

  la   t0, __trap_vec
  csrw mtvec, t0

  /* Make sure that mtvec is updated before continuing */

1:
  csrr t1, mtvec
  bne  t0, t1, 1b

  /* mscratch must be init to zero- we are not using scratch memory */

  csrw mscratch, zero
  csrw mcause, zero
  csrw mepc, zero
  li   x1,  0
  li   x2,  0
  li   x3,  0
  li   x4,  0
  li   x5,  0
  li   x6,  0
  li   x7,  0
  li   x8,  0
  li   x9,  0
  li   x10, 0
  li   x11, 0
  li   x12, 0
  li   x13, 0
  li   x14, 0
  li   x15, 0
  li   x16, 0
  li   x17, 0
  li   x18, 0
  li   x19, 0
  li   x20, 0
  li   x21, 0
  li   x22, 0
  li   x23, 0
  li   x24, 0
  li   x25, 0
  li   x26, 0
  li   x27, 0
  li   x28, 0
  li   x29, 0
  li   x30, 0
  li   x31, 0

  /* Skip delegation register, mmu and floating point initializations if E51 */
  csrr a0, mhartid
  beqz a0, .skip_e51

  /* Init delegation registers, mideleg, medeleg, if a U54
   * These are not initialised by the hardware and come up in a random state
   */

  csrw mideleg, 0
  csrw medeleg, 0

  /* invalid all MMU TLB Entry */

  sfence.vma x0, x0

  /* enable FPU and accelerator if present, setting ignored on E51
   * 15,16 = MSTATUS_XS, 17,18 = MSTATUS_MPRV
   * not defined on riscv-v/include/csr.h
   */

  /*  li      t0, MSTATUS_FS_DIRTY | (1 << 15) | (1 << 16) | (1 << 17) | (1 << 18) */
  li t0, 0x00006000 | 0x00018000 /* MSTATUS_FS | MSTATUS_XS */
  csrs    mstatus, t0

  /* Init floating point control register to zero */
#ifdef __riscv_flen
    fscsr x0
#endif

.skip_e51:

#ifdef CONFIG_MPFS_BOOTLOADER

  /* Set all but the boot hart into wfi */

  li a1, CONFIG_MPFS_BOOT_HART
  beq a0, a1, .continue_boot

  /* Enable IRQ_M_SOFT */

  li a2, (1U << 3)
  csrw mie, a2     /* Set MSIE bit to receive IPI */

  /* flush the instruction cache */
  fence.i

.wait_boot:
  wfi

  /* Only start if MIP_MSIP is set. Breakpoints in the debugger (halt)
   * will wakeup wfi, so we will check again that we actually got the soft
   * interrupt
   */

  csrr a2, mip
  andi a2, a2, (1U << 3) /* MIP_MSIP */
  beqz a2, .wait_boot

  /* Disable and clear all interrupts (the sw interrupt) */
  li a2, 0x00000008      /* MSTATUS_MIE */
  csrc mstatus, a2
  csrw mie, zero
  csrw mip, zero

  /* Jump to app (TODO: remove fixed address) */
  li a1, 0x80000000
  jr a1

.continue_boot:

#endif

  /* Set stack pointer to the idle thread stack */

  la   sp, MPFS_IDLESTACK_TOP


/* initialize global pointer, global data
 * The __global_pointer is allocated in the linker script.
 * It points to a location between _sdata and _edata as the offsets used in the gp are +/- 2k
 * See https://www.sifive.com/blog/2017/08/28/all-aboard-part-3-linker-relaxation-in-riscv-toolchain/
 * see: http://www.rowleydownload.co.uk/arm/documentation/gnu/as/RISC_002dV_002dDirectives.html
 */

.option push
.option norelax
  la  gp, __global_pointer$
.option pop


  /* Jump to __mpfs_start with mhartid in a0 */

  jal  __mpfs_start

  /* We shouldn't return from __mpfs_start
   * in case of return, loop forever. nop's added so can be seen in debugger
   */

1:
  nop
  nop
  j 1b

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

  addi sp, sp, -XCPTCONTEXT_SIZE

  sd   x1,  1*8(sp)   /* ra */

  /* leave gp(x3) in 3*8(sp) untouched */

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

  /* Switch to interrupt stack
   *
   * If booting on all harts, there are 5 irq stacks reserved,
   * one for each hart.
   * Just calculate the correct one for this hart
   *
   * For a single-hart boot mode just set the sp
   */

#ifdef CONFIG_MPFS_BOOTLOADER
  li s1, ((CONFIG_ARCH_INTERRUPTSTACK) & ~15)
  mul s1, s0, s1
  la s0, g_intstacktop
  sub sp, s0, s1
#else
  la sp, g_intstacktop
#endif /* CONFIG_MPFS_BOOTLOADER */

#endif

  /* Call interrupt handler in C */

  jal  x1, mpfs_dispatch_irq

  /* If context switch is needed, return a new sp */

  mv   sp, a0
  ld   s0, 0(sp)    /* restore mepc */
  csrw mepc, s0

  ld   s0, 32*8(sp) /* restore mstatus */
  csrw mstatus, s0

  /* leave gp(x3) in 3*8(sp) untouched */

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
 *  Name: g_intstackalloc and g_intstackbase
 ************************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK > 15
  .bss
  .balign 16
  .global g_intstackalloc
  .global g_intstacktop
  .type   g_intstackalloc, object
  .type   g_intstacktop, object

#ifdef CONFIG_MPFS_BOOTLOADER

/* If booting on all harts, reserve an own interruptstack for every hart */

g_intstackalloc:
  .skip  (((CONFIG_ARCH_INTERRUPTSTACK * 5) + 8) & ~15)
g_intstacktop:
  .size  g_intstacktop, 0
  .size  g_intstackalloc, ((CONFIG_ARCH_INTERRUPTSTACK * 5) & ~15)

#else

g_intstackalloc:
  .skip  ((CONFIG_ARCH_INTERRUPTSTACK + 8) & ~15)
g_intstacktop:
  .size  g_intstacktop, 0
  .size  g_intstackalloc, (CONFIG_ARCH_INTERRUPTSTACK & ~15)

#endif /* CONFIG_MPFS_BOOTLOADER */

#endif
