/****************************************************************************
 * include/nuttx/irq.h
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

#ifndef __INCLUDE_NUTTX_IRQ_H
#define __INCLUDE_NUTTX_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
# include <stdbool.h>
#endif

/* Now include architecture-specific types */

#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* IRQ detach is a convenience definition, it detach all the handlers
 * sharing the same IRQ. Detaching an interrupt handler is equivalent to
 * setting a NULL interrupt handler.
 */

#  define irq_detach(irq) irq_attach(irq, NULL, NULL)

/* Maximum/minimum values of IRQ integer types */

#  if NR_IRQS <= 256
#    define IRQT_MAX UINT8_MAX
#  elif NR_IRQS <= 65536
#    define IRQT_MAX UINT16_MAX
#  else
#    define IRQT_MAX UINT32_MAX
#  endif

#  ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
#    if CONFIG_ARCH_NUSER_INTERRUPTS <= 256
#      define IRQMAPPED_MAX UINT8_MAX
#    elif CONFIG_ARCH_NUSER_INTERRUPTS <= 65536
#      define IRQMAPPED_MAX UINT16_MAX
#    else
#      define IRQMAPPED_MAX UINT32_MAX
#   endif
#  endif

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* This type is an unsigned integer type large enough to hold the largest
 * IRQ number.
 */

#if NR_IRQS <= 256
typedef uint8_t irq_t;
#elif NR_IRQS <= 65536
typedef uint16_t irq_t;
#else
typedef uint32_t irq_t;
#endif

/* This type is an unsigned integer type large enough to hold the largest
 * mapped vector table index.
 */

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
#if CONFIG_ARCH_NUSER_INTERRUPTS <= 256
typedef uint8_t irq_mapped_t;
#elif CONFIG_ARCH_NUSER_INTERRUPTS <= 65536
typedef uint16_t irq_mapped_t;
#else
typedef uint32_t irq_mapped_t;
#endif
#endif /* CONFIG_ARCH_MINIMAL_VECTORTABLE */

/* This struct defines the form of an interrupt service routine */

typedef CODE int (*xcpt_t)(int irq, FAR void *context, FAR void *arg);
#endif /* __ASSEMBLY__ */

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

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
/* This is the interrupt vector mapping table.  This must be provided by
 * architecture specific logic if CONFIG_ARCH_MINIMAL_VECTORTABLE is define
 * in the configuration.
 *
 * REVISIT: Currently declared in sched/irq/irq.h.  This declaration here
 * introduces a circular dependency since it depends on NR_IRQS which is
 * defined in arch/irq.h but arch/irq.h includes nuttx/irq.h and we get
 * here with NR_IRQS undefined.
 */

/* EXTERN const irq_mapped_t g_irqmap[NR_IRQS]; */
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: irq_attach
 *
 * Description:
 *   Configure the IRQ subsystem so that IRQ number 'irq' is dispatched to
 *   'isr' with argument 'arg'
 *
 ****************************************************************************/

int irq_attach(int irq, xcpt_t isr, FAR void *arg);

#ifdef CONFIG_IRQCHAIN
int irqchain_detach(int irq, xcpt_t isr, FAR void *arg);
#else
#  define irqchain_detach(irq, isr, arg) irq_detach(irq)
#endif

/****************************************************************************
 * Name: enter_critical_section
 *
 * Description:
 *   If thread-specific IRQ counter is enabled (for SMP or other
 *   instrumentation):
 *
 *     Take the CPU IRQ lock and disable interrupts on all CPUs.  A thread-
 *     specific counter is incremented to indicate that the thread has IRQs
 *     disabled and to support nested calls to enter_critical_section().
 *
 *     NOTE: Most architectures do not support disabling all CPUs from one
 *     CPU.  ARM is an example.  In such cases, logic in
 *     enter_critical_section() will still manage entrance into the
 *     protected logic using spinlocks.
 *
 *   If thread-specific IRQ counter is not enabled:
 *
 *     This function is equivalent to up_irq_save().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to enter_critical_section();
 *
 ****************************************************************************/

#ifdef CONFIG_IRQCOUNT
irqstate_t enter_critical_section(void);
#else
#  define enter_critical_section() up_irq_save()
#endif

/****************************************************************************
 * Name: leave_critical_section
 *
 * Description:
 *   If thread-specific IRQ counter is enabled (for SMP or other
 *   instrumentation):
 *
 *     Decrement the IRQ lock count and if it decrements to zero then release
 *     the spinlock and restore the interrupt state as it was prior to the
 *     previous call to enter_critical_section().
 *
 *   If thread-specific IRQ counter is not enabled:
 *
 *     This function is equivalent to up_irq_restore().
 *
 * Input Parameters:
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to enter_critical_section();
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_IRQCOUNT
void leave_critical_section(irqstate_t flags);
#else
#  define leave_critical_section(f) up_irq_restore(f)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_IRQ_H */
