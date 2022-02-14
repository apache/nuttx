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
#include <arch/irq.h>

#include "chip.h"
#include "k210_memorymap.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

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

  /* In case of single CPU config, stop here */

#if !defined(CONFIG_SMP) || (CONFIG_SMP_NCPUS == 1)
  csrw mie, zero
  wfi
#endif

  /* To get g_cpu_basestack[mhartid], must get g_cpu_basestack first */

  la   t0, g_cpu_basestack

  /* Offset = pointer width * hart id */

  slli t1, a0, 3
  add  t0, t0, t1

  /* Load idle stack base to sp */

  ld   sp, 0(t0)

  /* sp (stack top) = sp + idle stack size */

  li   t0, CONFIG_IDLETHREAD_STACKSIZE
  add  sp, sp, t0

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
