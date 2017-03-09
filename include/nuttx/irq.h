/****************************************************************************
 * include/nuttx/irq.h
 *
 *   Copyright (C) 2007-2011, 2013, 2016-2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
# include <assert.h>
# include <arch/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef __ASSEMBLY__
/* IRQ detach is a convenience definition.  Detaching an interrupt handler
 * is equivalent to setting a NULL interrupt handler.
 */

#  define irq_detach(isr) irq_attach(isr, NULL, NULL)

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

typedef int (*xcpt_t)(int irq, FAR void *context, FAR void *arg);
#endif

/* Now include architecture-specific types */

#include <arch/irq.h>

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

/****************************************************************************
 * Name: enter_critical_section
 *
 * Description:
 *   If SMP is enabled:
 *     Take the CPU IRQ lock and disable interrupts on all CPUs.  A thread-
 *     specific counter is increment to indicate that the thread has IRQs
 *     disabled and to support nested calls to enter_critical_section().
 *
 *     NOTE: Most architectures do not support disabling all CPUs from one
 *     CPU.  ARM is an example.  In such cases, logic in
 *     enter_critical_section() will still manage entrance into the
 *     protected logic using spinlocks.
 *
 *   If SMP is not enabled:
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

#if defined(CONFIG_SMP) || defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
irqstate_t enter_critical_section(void);
#else
#  define enter_critical_section(f) up_irq_save(f)
#endif

/****************************************************************************
 * Name: leave_critical_section
 *
 * Description:
 *   If SMP is enabled:
 *     Decrement the IRQ lock count and if it decrements to zero then release
 *     the spinlock and restore the interrupt state as it was prior to the
 *     previous call to enter_critical_section().
 *
 *   If SMP is not enabled:
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

#if defined(CONFIG_SMP) || defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)
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
