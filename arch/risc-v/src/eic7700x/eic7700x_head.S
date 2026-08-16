/****************************************************************************
 * arch/risc-v/src/eic7700x/eic7700x_head.S
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

#include "chip.h"
#include "riscv_internal.h"
#include "riscv_macros.S"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Exported Symbols */

  .section .text
  .global __start
  .type   __start, function

__start:

  /* DO NOT MODIFY. Image Header expected by Linux bootloaders.
   *
   * This `li` instruction has no meaningful effect except that
   * its opcode forms the magic "MZ" signature of a PE/COFF file
   * that is required for UEFI applications.
   *
   * Some bootloaders check the magic "MZ" to see if the image is a valid
   * Linux image. But modifying the bootLoader is unnecessary unless we
   * need to do a customized secure boot. So we just put "MZ" in the
   * header to make the bootloader happy.
   */

  c.li    s4, -13              /* Magic Signature "MZ" (2 bytes) */
  j       real_start           /* Jump to Kernel Start (2 bytes) */
  .long   0                    /* Executable Code padded to 8 bytes */
  .quad   0x0200000            /* Image load offset from start of RAM */
  .quad   0x4000000            /* Kernel size (fdt_addr_r-kernel_addr_r) */
  .quad   0                    /* Kernel flags, little-endian */
  .long   2                    /* Version of this header */
  .long   0                    /* Reserved */
  .quad   0                    /* Reserved */
  .ascii  "RISCV\x00\x00\x00"  /* Magic number, "RISCV" (8 bytes) */
  .ascii  "RSC\x05"            /* Magic number 2, "RSC\x05" (4 bytes) */
  .long   0                    /* Reserved for PE COFF offset */

real_start:

  /* Take an idle stack.  a0 holds the Hart ID, placed there by whoever
   * entered us: the firmware on the first Hart in, and SBI on every Hart
   * started afterwards.
   */

#ifdef CONFIG_SMP

  /* One slot per CPU, indexed by Hart ID.  A Hart outside the set NuttX
   * knows about has no slot of its own and must not borrow one, so it stops
   * here instead.  chip.h refuses to build a configuration in which that
   * could be the Hart the firmware handed over on.
   */

  li   t1, CONFIG_SMP_NCPUS
  blt  a0, t1, 1f

2:
  csrw CSR_SIE, zero
  wfi
  j    2b

1:
  riscv_set_inital_sp EIC7700X_IDLESTACK_BASE, SMP_STACK_SIZE, a0

#else

  /* One CPU, so one slot, and it belongs to Hart 0.  Any other Hart is here
   * only to restart on Hart 0 and park, so it borrows the same slot; the two
   * briefly overlap, which is why the SMP path above does not do this.
   */

  li   a2, 0
  riscv_set_inital_sp EIC7700X_IDLESTACK_BASE, SMP_STACK_SIZE, a2

#endif

  /* Disable all interrupts (i.e. timer, external) in sie */

  csrw CSR_SIE, zero

  la   t0, __trap_vec
  csrw CSR_STVEC, t0

  /* Jump to eic7700x_start */

  jal  x1, eic7700x_start

  /* We shouldn't return from _start */

  .global _init
  .global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret
