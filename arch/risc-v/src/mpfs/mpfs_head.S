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

#include <nuttx/irq.h>

#include "chip.h"
#include "mpfs_memorymap.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start

__start:

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

  /* Delegation registers must be explicitly reset */

  csrw mideleg, 0
  csrw medeleg, 0

  /* Remove MMU mappings (if any) */

  csrw satp, zero
  fence

  /* Flush TLB (does not make a difference really) */

  sfence.vma x0, x0

.skip_e51:

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

  /* Offset to g_entrypoints vector, acc. to hart id */
  li t1,8
  mul t1, a0, t1

  /* Load the entrypoint address */
  la t0, g_entrypoints
  add t0, t0, t1
  ld t0, 0(t0)

  /* Jump to app */
  jr t0

.continue_boot:

  /* Clear PMP */

  csrw pmpcfg0, zero
  csrw pmpcfg2, zero
#endif

  /* Set stack pointer to the idle thread stack */

  la   sp, MPFS_IDLESTACK_TOP

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
