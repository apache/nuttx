/****************************************************************************
 * sched/irq/irq.h
 *
 *   Copyright (C) 2007, 2008, 2013-2014, 2017 Gregory Nutt. All rights reserved.
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

#ifndef __SCHED_IRQ_IRQ_H
#define __SCHED_IRQ_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <sys/types.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARCH_MINIMAL_VECTORTABLE) && \
   !defined(CONFIG_ARCH_NUSER_INTERRUPTS)
#  error CONFIG_ARCH_NUSER_INTERRUPTS is not defined
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This is the type of the list of interrupt handlers, one for each IRQ.
 * This type provided all of the information necessary to irq_dispatch to
 * transfer control to interrupt handlers after the occurrence of an
 * interrupt.
 */

struct irq_info_s
{
  xcpt_t handler;  /* Address of the interrupt handler */
  FAR void *arg;   /* The argument provided to the interrupt handler. */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the list of interrupt handlers, one for each IRQ.  This is used
 * by irq_dispatch to transfer control to interrupt handlers after the
 * occurrence of an interrupt.
 */

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
extern struct irq_info_s g_irqvector[CONFIG_ARCH_NUSER_INTERRUPTS];
#else
extern struct irq_info_s g_irqvector[NR_IRQS];
#endif

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
/* This is the interrupt vector mapping table.  This must be provided by
 * architecture specific logic if CONFIG_ARCH_MINIMAL_VECTORTABLE is define
 * in the configuration.
 *
 * REVISIT: This should be declared in include/nuttx/irq.h.  The declaration
 * at that location, however, introduces a circular include dependency so the
 * declaration is here for the time being.
 */

extern const irq_mapped_t g_irqmap[NR_IRQS];
#endif

#ifdef CONFIG_SMP
/* This is the spinlock that enforces critical sections when interrupts are
 * disabled.
 */

extern volatile spinlock_t g_cpu_irqlock SP_SECTION;

/* Used to keep track of which CPU(s) hold the IRQ lock. */

extern volatile spinlock_t g_cpu_irqsetlock SP_SECTION;
extern volatile cpu_set_t g_cpu_irqset SP_SECTION;

/* Handles nested calls to enter_critical section from interrupt handlers */

extern volatile uint8_t g_cpu_nestcount[CONFIG_SMP_NCPUS];
#endif

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
 * Name: irq_initialize
 *
 * Description:
 *   Configure the IRQ subsystem
 *
 ****************************************************************************/

void weak_function irq_initialize(void);

/****************************************************************************
 * Name: irq_unexpected_isr
 *
 * Description:
 *   An interrupt has been received for an IRQ that was never registered
 *   with the system.
 *
 ****************************************************************************/

int irq_unexpected_isr(int irq, FAR void *context, FAR void *arg);

/****************************************************************************
 * Name:  irq_cpu_locked
 *
 * Description:
 *   Test if the IRQ lock set OR if this CPU holds the IRQ lock
 *   There is an interaction with pre-emption controls and IRQ locking:
 *   Even if the pre-emption is enabled, tasks will be forced to pend if
 *   the IRQ lock is also set UNLESS the CPU starting the task is the
 *   holder of the IRQ lock.
 *
 * Inputs:
 *   rtcb - Points to the blocked TCB that is ready-to-run
 *
 * Return Value:
 *   true  - IRQs are locked by a different CPU.
 *   false - IRQs are unlocked OR if they are locked BUT this CPU
 *           is the holder of the lock.
 *
 *   Warning: This values are volatile at only valid at the instance that
 *   the CPU set was queried.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
bool irq_cpu_locked(int cpu);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SCHED_IRQ_IRQ_H */
