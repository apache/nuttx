/****************************************************************************
 * include/nuttx/irq.h
 *
 *   Copyright (C) 2007-2011, 2013, 2016-2017 Gregory Nutt. All rights
 *     reserved.
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
#endif

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
 *     specific counter is increment to indicate that the thread has IRQs
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

/****************************************************************************
 * Name: spin_lock_irqsave
 *
 * Description:
 *   If SMP and SPINLOCK_IRQ are enabled:
 *     Disable local interrupts and take the global spinlock (g_irq_spin)
 *     if the call counter (g_irq_spin_count[cpu]) equals to 0. Then the
 *     counter on the CPU is increment to allow nested call.
 *
 *     NOTE: This API is very simple to protect data (e.g. H/W register
 *     or internal data structure) in SMP mode. But do not use this API
 *     with kernel APIs which suspend a caller thread. (e.g. nxsem_wait)
 *
 *   If SMP and SPINLOCK_IRQ are not enabled:
 *     This function is equivalent to enter_critical_section().
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   An opaque, architecture-specific value that represents the state of
 *   the interrupts prior to the call to spin_lock_irqsave();
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && defined(CONFIG_SPINLOCK_IRQ) && \
    defined(CONFIG_ARCH_GLOBAL_IRQDISABLE)
irqstate_t spin_lock_irqsave(void);
#else
#  define spin_lock_irqsave() enter_critical_section()
#endif

/****************************************************************************
 * Name: spin_unlock_irqrestore
 *
 * Description:
 *   If SMP and SPINLOCK_IRQ are enabled:
 *     Decrement the call counter (g_irq_spin_count[cpu]) and if it
 *     decrements to zero then release the spinlock (g_irq_spin) and
 *     restore the interrupt state as it was prior to the previous call to
 *     spin_lock_irqsave().
 *
 *   If SMP and SPINLOCK_IRQ are not enabled:
 *     This function is equivalent to leave_critical_section().
 *
 * Input Parameters:
 *   flags - The architecture-specific value that represents the state of
 *           the interrupts prior to the call to spin_lock_irqsave();
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_SMP) && defined(CONFIG_SPINLOCK_IRQ) && \
    defined(CONFIG_ARCH_GLOBAL_IRQDISABLE)
void spin_unlock_irqrestore(irqstate_t flags);
#else
#  define spin_unlock_irqrestore(f) leave_critical_section(f)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_IRQ_H */
