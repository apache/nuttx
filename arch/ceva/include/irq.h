/****************************************************************************
 * arch/ceva/include/irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_CEVA_INCLUDE_IRQ_H
#define __ARCH_CEVA_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sys/types.h>
#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/* Include CEVA architecture-specific IRQ definitions (including register
 * save structure and up_irq_save()/up_irq_restore() functions)
 */

#if defined(CONFIG_ARCH_XC5)
#  include <arch/xc5/irq.h>
#elif defined(CONFIG_ARCH_XM6)
#  include <arch/xm6/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IRQ_VINT0           (IRQ_VINT_FIRST + 0)
#define IRQ_VINT1           (IRQ_VINT_FIRST + 1)
#define IRQ_VINT2           (IRQ_VINT_FIRST + 2)
#define IRQ_VINT3           (IRQ_VINT_FIRST + 3)
#define IRQ_VINT4           (IRQ_VINT_FIRST + 4)
#define IRQ_VINT5           (IRQ_VINT_FIRST + 5)
#define IRQ_VINT6           (IRQ_VINT_FIRST + 6)
#define IRQ_VINT7           (IRQ_VINT_FIRST + 7)
#define IRQ_VINT8           (IRQ_VINT_FIRST + 8)
#define IRQ_VINT9           (IRQ_VINT_FIRST + 9)
#define IRQ_VINT10          (IRQ_VINT_FIRST + 10)
#define IRQ_VINT11          (IRQ_VINT_FIRST + 11)
#define IRQ_VINT12          (IRQ_VINT_FIRST + 12)
#define IRQ_VINT13          (IRQ_VINT_FIRST + 13)
#define IRQ_VINT14          (IRQ_VINT_FIRST + 14)
#define IRQ_VINT15          (IRQ_VINT_FIRST + 15)
#define IRQ_VINT16          (IRQ_VINT_FIRST + 16)
#define IRQ_VINT17          (IRQ_VINT_FIRST + 17)
#define IRQ_VINT18          (IRQ_VINT_FIRST + 18)
#define IRQ_VINT19          (IRQ_VINT_FIRST + 19)
#define IRQ_VINT20          (IRQ_VINT_FIRST + 20)
#define IRQ_VINT21          (IRQ_VINT_FIRST + 21)
#define IRQ_VINT22          (IRQ_VINT_FIRST + 22)
#define IRQ_VINT23          (IRQ_VINT_FIRST + 23)
#define IRQ_VINT24          (IRQ_VINT_FIRST + 24)
#define IRQ_VINT25          (IRQ_VINT_FIRST + 25)
#define IRQ_VINT26          (IRQ_VINT_FIRST + 26)

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
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

EXTERN uint32_t *volatile g_current_regs[CONFIG_SMP_NCPUS];
#define CURRENT_REGS (g_current_regs[up_cpu_index()])

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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
int up_cpu_index(void);
#else
#  define up_cpu_index() (0)
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_interrupt_context
 *
 * Description:
 *   Return true is we are currently executing in the interrupt
 *   handler context.
 *
 ****************************************************************************/

static inline bool up_interrupt_context(void)
{
#ifdef CONFIG_SMP
  irqstate_t flags = up_irq_save();
#endif
  bool ret = CURRENT_REGS != NULL;

#ifdef CONFIG_SMP
  up_irq_restore(flags);
#endif

  return ret;
}

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_INCLUDE_IRQ_H */
