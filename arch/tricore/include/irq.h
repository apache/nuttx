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

#include <arch/arch.h>
#include <arch/chip/irq.h>

#if defined(CONFIG_ARCH_CHIP_FAMILY_TC3X)
#  include <arch/tc3x/irq.h>
#elif defined(CONFIG_ARCH_CHIP_FAMILY_TC4X)
#  include <arch/tc4x/irq.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#ifndef STACKFRAME_ALIGN
#  define STACKFRAME_ALIGN 8
#endif

/* FPU registers */

#ifdef CONFIG_ARCH_TC1V6
#  define FPU_SYNC_TRAP_REG  CPU_FPU_TRAP_CON
#  define FPU_ASYNC_TRAP_REG CPU_FPU_TRAP_CON
#else
#  define FPU_SYNC_TRAP_REG  CPU_FPU_SYNC_TRAP_CON
#  define FPU_ASYNC_TRAP_REG CPU_FPU_TRAP_CON
#endif
#define FPU_TRAP_FZE_SHIFT   20
#define FPU_TRAP_TCL_SHIFT   1

#ifndef __ASSEMBLY__

#ifdef __cplusplus

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: up_irq_enable
 *
 * Description:
 *   Enable interrupts globally.
 *
 ****************************************************************************/

void up_irq_enable(void);

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

noinstrument_function static inline_function uintptr_t up_getsp(void)
{
  return __builtin_frame_address(0);
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Disable interrupts and return the previous PSW.IE state.
 *
 ****************************************************************************/

#define TRICORE_IRQ_DISABLE_AND_SAVE(x) \
  __asm__ __volatile__("disable %0":"=d"(x))
#define TRICORE_IRQ_RESTORE(x) \
  __asm__ __volatile__("restore %0"::"d"(x))

noinstrument_function static inline_function irqstate_t up_irq_save(void)
{
  irqstate_t state;
  TRICORE_IRQ_DISABLE_AND_SAVE(state);
  return state;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore the previous PSW.IE state from up_irq_save().
 *
 ****************************************************************************/

noinstrument_function static inline_function
void up_irq_restore(irqstate_t flags)
{
  TRICORE_IRQ_RESTORE(flags);
}

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

static inline_function uintptr_t *up_current_regs(void)
{
  return (uintptr_t *)g_current_regs[0];
}

static inline_function void up_set_current_regs(uintptr_t *regs)
{
  g_current_regs[0] = regs;
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
  bool ret = up_current_regs() != NULL;

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

/****************************************************************************
 * Name: up_getusrpc
 ****************************************************************************/

#define up_getusrpc(regs) \
    (((uintptr_t *)((regs) ? (regs) : running_regs()))[REG_PC])

#endif /* __ASSEMBLY__ */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_TRICORE_INCLUDE_IRQ_H */
