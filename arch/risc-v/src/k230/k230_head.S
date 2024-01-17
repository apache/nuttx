/****************************************************************************
 * arch/risc-v/src/k230/k230_head.S
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
#include <arch/mode.h>

#include "chip.h"
#include "riscv_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/
  /* Imported symbols */

  .extern __trap_vec

  /* Exported Symbols */

  .section .text

#ifdef CONFIG_NUTTSBI

  .global __start_s

__start_s:

#else

  .global __start

__start:

#endif

  /* Preserve a1 by not using it here as it contains DTB */

#ifndef CONFIG_BUILD_KERNEL
  /* Load mhartid (cpuid) */

  csrr a0, mhartid
#endif

  /* Set stack pointer to the idle thread stack */

  bnez a0, 1f
  la   sp, K230_IDLESTACK_TOP
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
  csrw CSR_IE, zero
  wfi

3:
  /* To get g_cpu_basestack[mhartid], must get g_cpu_basestack first */

  la   t0, g_cpu_basestack

  /* Offset = pointer width * hart id */

  slli t1, a0, 3
  add  t0, t0, t1

  /* Load idle stack base to sp */

  REGLOAD sp, 0(t0)

  /*
   * sp (stack top) = sp + idle stack size - XCPTCONTEXT_SIZE
   *
   * Note: Reserve some space used by up_initial_state since we are already
   * running and using the per CPU idle stack.
   */

  li   t0, STACK_ALIGN_UP(CONFIG_IDLETHREAD_STACKSIZE - XCPTCONTEXT_SIZE)
  add  sp, sp, t0

2:

  /* Disable all interrupts (i.e. timer, external) */

	csrw	CSR_IE, zero
  
  la   t0, __trap_vec        /* __trap_dump */
  csrw CSR_TVEC, t0
  
  /* Jump to k230_start, a0=mhartid, a1=dtb */

  jal  x1, k230_start

  /* We shouldn't return from _start */

  .global _init
  .global _fini

_init:
_fini:

  /* These don't have to do anything since we use init_array/fini_array. */

  ret
