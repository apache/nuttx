/****************************************************************************
 * arch/risc-v/src/c906/c906_head.S
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
#include "c906_memorymap.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __trap_vec

  .section .text
  .global __start

__start:

.option push
.option norelax
    la      gp, __global_pointer$
.option pop

  /* invalid all MMU TLB Entry */

  sfence.vma x0,x0

  /* enable thead ISA extension:
   * BIT22: enable the THEAD ISA extensions.
   * BIT21: enable extended attributes in PTE.
   * BIT15: enable misaligned address access.
   * mxstatus is a CSR which locates at 0x7C0.
   */

  li t0, THEAD_MXSTATUS_ISAEE | THEAD_MXSTATUS_MAEE | THEAD_MXSTATUS_MM
  csrr t1, THEAD_CSR_MXSTATUS
  or  t0, t1, t0
  csrw THEAD_CSR_MXSTATUS, t0

  /* Load mhartid (cpuid) */

  csrr a0, CSR_MHARTID

  /* Set stack pointer to the idle thread stack */

  la   sp, C906_IDLESTACK_TOP

  /* Disable all interrupts (i.e. timer, external) in mie */

  csrw CSR_MIE, zero
  csrw CSR_MIP, zero

  /* Initialize the Machine Trap Vector */

  la   t0, __trap_vec
  csrw CSR_MTVEC, t0

  /* Jump to __c906_start with mhartid */

  j    __c906_start

  /* We shouldn't return from __c906_start */

  .global _init
  .global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret
