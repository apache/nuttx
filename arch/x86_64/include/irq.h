/****************************************************************************
 * arch/x86_64/include/irq.h
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

#ifndef __ARCH_X86_64_INCLUDE_IRQ_H
#define __ARCH_X86_64_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include NuttX-specific IRQ definitions */

#include <nuttx/irq.h>

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/* Include architecture-specific IRQ definitions (including register save
 * structure and up_irq_save()/up_irq_restore() macros).
 */

#ifdef CONFIG_ARCH_INTEL64
#  include <arch/intel64/irq.h>
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* CPU private data */

struct intel64_cpu_s
{
  int     id;
  uint8_t loapic_id;
  bool    ready;

/* current_regs holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to current_regs must be through
 * up_current_regs() and up_set_current_regs() functions
 */

  uint64_t *current_regs;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: up_cpu_index
 *
 * Description:
 *   Return an index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An integer index in the range of 0 through (CONFIG_SMP_NCPUS-1) that
 *   corresponds to the currently executing CPU.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static inline_function int up_cpu_index(void)
{
  int cpu;

  __asm__ volatile(
    "\tmovl %%gs:(%c1), %0\n"
    : "=r" (cpu)
    : "i" (offsetof(struct intel64_cpu_s, id))
    :);

  return cpu;
}
#else
#  define up_cpu_index() (0)
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

static inline_function uint64_t *up_current_regs(void)
{
  uint64_t *regs;
  __asm__ volatile("movq %%gs:(%c1), %0"
                   : "=rm" (regs)
                   : "i" (offsetof(struct intel64_cpu_s, current_regs)));
  return regs;
}

static inline_function void up_set_current_regs(uint64_t *regs)
{
  __asm__ volatile("movq %0, %%gs:(%c1)"
                   :: "r" (regs), "i" (offsetof(struct intel64_cpu_s,
                                                current_regs)));
}

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in the interrupt handler
 *   context.
 *
 ****************************************************************************/

noinstrument_function
static inline_function bool up_interrupt_context(void)
{
  return up_current_regs() != NULL;
}

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_X86_64_INCLUDE_IRQ_H */

