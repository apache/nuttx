/****************************************************************************
 * arch/or1k/include/irq.h
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

/* This file should never be included directly but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_OR1K_INCLUDE_IRQ_H
#define __ARCH_OR1K_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/* Return the current value of the stack pointer */

static inline_function uint32_t up_getsp(void)
{
  uint32_t sp;

  __asm__
  (
    "\tmov %0, sp\n\t"
    : "=r"(sp)
  );

  return sp;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the
 * [get/set]_current_regs for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

EXTERN volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return the real core number regardless CONFIG_SMP setting
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_MULTICPU
int up_cpu_index(void) noinstrument_function;
#endif /* CONFIG_ARCH_HAVE_MULTICPU */

/****************************************************************************
 * Inline functions
 ****************************************************************************/

static inline_function uint32_t *up_current_regs(void)
{
#ifdef CONFIG_SMP
  return (uint32_t *)g_current_regs[up_cpu_index()];
#else
  return (uint32_t *)g_current_regs[0];
#endif
}

static inline_function void up_set_current_regs(uint32_t *regs)
{
#ifdef CONFIG_SMP
  g_current_regs[up_cpu_index()] = regs;
#else
  g_current_regs[0] = regs;
#endif
}

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in the interrupt
 *   handler context.
 *
 ****************************************************************************/

static inline_function bool up_interrupt_context(void)
{
#ifdef CONFIG_SMP
  irqstate_t flags = up_irq_save();
#endif

  bool ret = up_current_regs() != NULL;

#ifdef CONFIG_SMP
  up_irq_restore(flags);
#endif

  return ret;
}

/****************************************************************************
 * Name: up_getusrpc
 ****************************************************************************/

#define up_getusrpc(regs) \
    (((uint32_t *)((regs) ? (regs) : up_current_regs()))[REG_PC])

/****************************************************************************
 * Name: up_getusrsp
 ****************************************************************************/

#define up_getusrsp(regs) \
    ((uintptr_t)((uint32_t *)(regs))[REG_R13])

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_OR1K_INCLUDE_IRQ_H */
