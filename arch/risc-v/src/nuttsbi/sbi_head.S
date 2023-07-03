/****************************************************************************
 * arch/risc-v/src/nuttsbi/sbi_head.S
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

#include "sbi_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Imported symbols */

  .extern __mtrap_vec

  .section .text
  .global __start

__start:

  /* Disable all interrupts (i.e. timer, external) in mie */

  csrw mie, zero
  csrw mip, zero

  /* Initialize the Machine Trap Vector */

  la   t0, __mtrap_vec
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

  /* Delegation registers must be explicitly reset */

  csrw mideleg, 0
  csrw medeleg, 0

  /* Remove MMU mappings (if any) */

  csrw satp, zero
  fence

  /* Flush TLB (does not make a difference really) */

  sfence.vma x0, x0

  /* Clear PMP */

  csrw pmpcfg0, 0
  csrw pmpcfg2, 0

  /* Set up a temporary stack */

#if MMODE_HART_CNT > 1
  /* REVISIT: This requires that hartID is sequential, and starts from 0 */

  csrr a0, mhartid
  li   s0, TEMP_STACK_SIZE
  mul  s0, a0, s0
  la   s1, TEMP_STACK_BASE
  sub  sp, s1, s0
#else
  la   sp, TEMP_STACK_BASE
#endif

  /* Jump to start */

  j sbi_start
