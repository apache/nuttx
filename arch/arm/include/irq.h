/****************************************************************************
 * arch/arm/include/irq.h
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

#ifndef __ARCH_ARM_INCLUDE_IRQ_H
#define __ARCH_ARM_INCLUDE_IRQ_H

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

/* Include ARM architecture-specific IRQ definitions (including register
 * save structure and up_irq_save()/up_irq_restore() functions)
 */

#if defined(CONFIG_ARCH_ARMV7A)
#  include <arch/armv7-a/irq.h>
#elif defined(CONFIG_ARCH_ARMV7R)
#  include <arch/armv7-r/irq.h>
#elif defined(CONFIG_ARCH_ARMV8R)
#  include <arch/armv8-r/irq.h>
#elif defined(CONFIG_ARCH_ARMV7M)
#  include <arch/armv7-m/irq.h>
#elif defined(CONFIG_ARCH_ARMV8M)
#  include <arch/armv8-m/irq.h>
#elif defined(CONFIG_ARCH_ARMV6M)
#  include <arch/armv6-m/irq.h>
#else
#  include <arch/arm/irq.h>
#endif

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

#define up_getsp() (uintptr_t)__builtin_frame_address(0)

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
 * CURRENT_REGS for portability.
 */

/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

EXTERN volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];
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
#endif /* __ASSEMBLY__ */

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_ARM_INCLUDE_IRQ_H */
