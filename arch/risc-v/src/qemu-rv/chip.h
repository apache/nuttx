/****************************************************************************
 * arch/risc-v/src/qemu-rv/chip.h
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

#ifndef __ARCH_RISCV_SRC_QEMU_RV_CHIP_H
#define __ARCH_RISCV_SRC_QEMU_RV_CHIP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include the chip capabilities file */

#include <arch/qemu-rv/chip.h>

#include "qemu_rv_memorymap.h"

#include "hardware/qemu_rv_clint.h"
#include "hardware/qemu_rv_memorymap.h"
#include "hardware/qemu_rv_plic.h"
#include "hardware/qemu_rv_aplic.h"

#include "riscv_internal.h"
#include "riscv_percpu.h"

/****************************************************************************
 * Macro Definitions
 ****************************************************************************/

#ifdef __ASSEMBLY__

#if CONFIG_ARCH_INTERRUPTSTACK > 15

/****************************************************************************
 * Name: setintstack_bounds
 *
 * Description:
 *   Set sp to high if sp is outside [low, high], the bounds of the
 *   interrupt stack for the current CPU.
 *
 ****************************************************************************/

.macro  setintstack_bounds high, low

  /* Check if sp is below the low address of the interrupt stack
   * (outside the interrupt stack).
   */

  blt   sp, \low, 1f

  /* Check if sp is above the high address of the interrupt stack
   * (outside the interrupt stack)
   */

  bgt   sp, \high, 1f

  /* If sp is within the interrupt stack boundaries, no action is required */

  j     2f

1:
  /* Set sp to the high address of the interrupt stack (start of the
   * interrupt stack)
   */

  mv    sp, \high

2:
.endm

/****************************************************************************
 * Name: setintstack
 *
 * Description:
 *   Set the current stack pointer to the "top" of the correct interrupt
 *   stack for the current CPU.
 *
 ****************************************************************************/

#if defined(CONFIG_SMP)
.macro  setintstack tmp0, tmp1
  up_cpu_index \tmp0
  li    \tmp1, STACKFRAME_ALIGN_DOWN(CONFIG_ARCH_INTERRUPTSTACK)
  mul   \tmp1, \tmp0, \tmp1
  la    \tmp0, g_intstacktop

  /* tmp0 = g_intstacktop - (CONFIG_ARCH_INTERRUPTSTACK * HART_ID)
   * (high address of the interrupt stack)
   */

  sub   \tmp0, \tmp0, \tmp1
  li    \tmp1, STACKFRAME_ALIGN_DOWN(CONFIG_ARCH_INTERRUPTSTACK)

  /* tmp1 = tmp0 - CONFIG_ARCH_INTERRUPTSTACK
   * (low address of the interrupt stack)
   */

  sub   \tmp1, \tmp0, \tmp1

  setintstack_bounds \tmp0, \tmp1
.endm
#elif defined(CONFIG_ARCH_USE_S_MODE)
.macro  setintstack tmp0, tmp1
  csrr    \tmp0, CSR_SCRATCH
  REGLOAD \tmp0, RISCV_PERCPU_IRQSTACK(\tmp0)

  /* tmp0 = high address (top) of the interrupt stack for this hart */

  li      \tmp1, STACKFRAME_ALIGN_DOWN(CONFIG_ARCH_INTERRUPTSTACK)
  sub     \tmp1, \tmp0, \tmp1

  /* tmp1 = low address of the interrupt stack */

  setintstack_bounds \tmp0, \tmp1
.endm
#endif /* defined(CONFIG_SMP) */

#endif /* CONFIG_ARCH_INTERRUPTSTACK > 15 */

#endif /* __ASSEMBLY__  */
#endif /* __ARCH_RISCV_SRC_QEMU_RV_CHIP_H */
