/****************************************************************************
 * arch/risc-v/src/qemu-rv/qemu_rv_head.S
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

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

  /* Exported Symbols */

  .section .text
  .global __start

__start:

  /* Preserve a1 as it contains the pointer to DTB */
  /* Load mhartid (cpuid) */

  csrr a0, CSR_MHARTID

  /* Set stack pointer to the idle thread stack */

  bnez a0, 1f
  la   sp, QEMU_RV_IDLESTACK_TOP
  j    2f
1:

  /* Load the number of CPUs that the kernel supports */

#ifdef CONFIG_SMP
  li   t1, CONFIG_SMP_NCPUS
#else
  li   t1, 1
#endif

  /* If a0 (mhartid) >= t1 (the number of CPUs), stop here */

  blt  a0, t1, 3f
  csrw CSR_MIE, zero
  wfi

3:
  /* Get start address of idle stack array */

  la   t0, g_cpux_idlestack

  /* Get idle stack offset */

  li   t1, SMP_STACK_SIZE
  mul  t1, t1, a0

  /* Load idle stack top to sp */

  add  sp, t0, t1

  /*
   * sp (stack top) = sp (stack top) - XCPTCONTEXT_SIZE
   *
   * Note: Reserve some space used by up_initial_state since we are already
   * running and using the per CPU idle stack.
   */

  li   t0, -STACK_ALIGN_UP(XCPTCONTEXT_SIZE)
  add  sp, sp, t0

2:

  /* Disable all interrupts (i.e. timer, external) in mie */

	csrw CSR_MIE, zero

  la   t0, __trap_vec
  csrw CSR_MTVEC, t0

  /* Jump to qemu_rv_start */

  jal  x1, qemu_rv_start

  /* We shouldn't return from _start */

  .global _init
  .global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret
