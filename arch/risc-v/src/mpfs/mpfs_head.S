/****************************************************************************
 * arch/risc-v/src/mpfs/mpfs_head.S
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

#include <nuttx/irq.h>

#include "chip.h"
#include "mpfs_memorymap.h"
#include "riscv_internal.h"
#include "riscv_macros.S"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __trap_vec

  .section .start, "ax"
  .global __start

__start:

  /* Disable all interrupts (i.e. timer, external) in mie */

  csrw CSR_MIE, zero
  csrw CSR_MIP, zero

  /* Clear all IPIs (above doesn't clear them) */

  csrr a0, CSR_MHARTID
  slli t1, a0, 2
  li   t0, MPFS_CLINT_BASE
  add  t1, t1, t0
  sw   zero, 0(t1)

  /* Initialize the Machine Trap Vector */

  la   t0, __trap_vec
  csrw CSR_MTVEC, t0

  /* Make sure that mtvec is updated before continuing */

1:
  csrr t1, CSR_MTVEC
  bne  t0, t1, 1b

  /* mscratch must be init to zero- we are not using scratch memory */

  csrw CSR_MSCRATCH, zero
  csrw CSR_MCAUSE, zero
  csrw CSR_MEPC, zero
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
  csrr a0, CSR_MHARTID
  beqz a0, .skip_e51

  /* Delegation registers must be explicitly reset */

  csrw CSR_MIDELEG, 0
  csrw CSR_MEDELEG, 0

  /* Remove MMU mappings (if any) */

  csrw CSR_SATP, zero
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
  /* Clear PMP */

  csrw CSR_PMPCFG0, zero
  csrw CSR_PMPCFG2, zero

  /* Set all but the boot hart into wfi */

  li a1, CONFIG_MPFS_BOOT_HART
  beq a0, a1, .continue_boot

  /* Enable IRQ_M_SOFT */

  li a2, (1U << 3)
  csrw CSR_MIE, a2     /* Set MSIE bit to receive IPI */

  /* flush the instruction cache */
  fence.i

.wait_boot:
  wfi

  /* Only start if MIP_MSIP is set. Breakpoints in the debugger (halt)
   * will wakeup wfi, so we will check again that we actually got the soft
   * interrupt
   */

  csrr a2, CSR_MIP
  andi a2, a2, (1U << 3) /* MIP_MSIP */
  beqz a2, .wait_boot

  /* Disable and clear all interrupts (the sw interrupt) */
  li a2, 0x00000008      /* MSTATUS_MIE */
  csrc CSR_MSTATUS, a2
  csrw CSR_MIE, zero
  csrw CSR_MIP, zero

  /* Jump to application */
  tail mpfs_jump_to_app

.continue_boot:

#ifdef CONFIG_MPFS_L2_CLEAR
  /* L2 needs to be zeroed before ECC (error correction) is enabled later. */

  la  a4, __l2lim_start
  la  a5, __l2lim_end

.clear_l2lim:
  sd   x0, 0(a4)
  add  a4, a4, 8
  blt  a4, a5, .clear_l2lim
#endif /* CONFIG_MPFS_L2_CLEAR */
#endif

  /* Set stack pointer to the idle thread stack */

#ifdef CONFIG_SMP
  addi a1, a0, -1        /* Only cores 1...3 can participate in SMP */
  riscv_set_inital_sp MPFS_IDLESTACK_BASE, SMP_STACK_SIZE, a1
#else
  la   sp, MPFS_IDLESTACK_TOP
#endif

  /* Jump to __mpfs_start with mhartid in a0 */

  tail __mpfs_start

  .global _init
  .global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret
