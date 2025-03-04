/****************************************************************************
 * arch/tricore/include/irq.h
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

#ifndef __ARCH_TRICORE_INCLUDE_IRQ_H
#define __ARCH_TRICORE_INCLUDE_IRQ_H

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
 * Pre-processor Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * g_current_regs for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

EXTERN volatile uintptr_t *g_current_regs[CONFIG_SMP_NCPUS];

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
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts globally.
 *
 ****************************************************************************/

void up_irq_enable(void);

/****************************************************************************
 * Inline functions
 ****************************************************************************/

noinstrument_function static inline_function uintptr_t up_getsp(void)
{
#ifdef CONFIG_TRICORE_TOOLCHAIN_TASKING
  return (uintptr_t)__get_sp();
#else
  return __builtin_frame_address(0);
#endif
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupts and return the previous value of the mstatus register
 *
 ****************************************************************************/

noinstrument_function static inline_function irqstate_t up_irq_save(void)
{
  return __disable_and_save();
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore the value of the mstatus register
 *
 ****************************************************************************/

noinstrument_function static inline_function
void up_irq_restore(irqstate_t flags)
{
  __restore(flags);
}

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function uintptr_t *up_current_regs(void)
{
#ifdef CONFIG_SMP
  return (uintptr_t *)g_current_regs[up_cpu_index()];
#else
  return (uintptr_t *)g_current_regs[0];
#endif
}

static inline_function void up_set_current_regs(uintptr_t *regs)
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

noinstrument_function
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
 * Name: up_getusrsp
 ****************************************************************************/

static inline_function uintptr_t up_getusrsp(void *regs)
{
  uintptr_t *csa = regs;

  while (((uintptr_t)csa & PCXI_UL) == 0)
    {
      csa = tricore_csa2addr((uintptr_t)csa);
      csa = (uintptr_t *)csa[0];
    }

  csa = tricore_csa2addr((uintptr_t)csa);

  return csa[REG_SP];
}

#endif /* __ASSEMBLY__ */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_TRICORE_INCLUDE_IRQ_H */
