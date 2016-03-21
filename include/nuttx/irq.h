/****************************************************************************
 * include/nuttx/irq.h
 *
 *   Copyright (C) 2007-2011, 2013, 2016 Gregory Nutt. All rights reserved.
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
# include <assert.h>
# include <arch/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* IRQ detach is a convenience definition.  Detaching an interrupt handler
 * is equivalent to setting a NULL interrupt handler.
 */

#ifndef __ASSEMBLY__
# define irq_detach(isr) irq_attach(isr, NULL)
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This struct defines the way the registers are stored */

#ifndef __ASSEMBLY__
typedef int (*xcpt_t)(int irq, FAR void *context);
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: irq_attach
 *
 * Description:
 *   Configure the IRQ subsystem so that IRQ number 'irq' is dispatched to
 *   'isr'
 *
 ****************************************************************************/

int irq_attach(int irq, xcpt_t isr);

/****************************************************************************
 * Name: enter_critical_section
 *
 * Description:
 *   If SMP is enabled:
 *     Take the CPU IRQ lock and disable interrupts on all CPUs.  A thread-
 *     specific counter is increment to indicate that the thread has IRQs
 *     disabled and to support nested calls to enter_critical_section().
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_save().
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
 *     the spinlock.
 *   If SMP is not enabled:
 *     This function is equivalent to up_irq_restore().
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
#endif

#endif /* __INCLUDE_NUTTX_IRQ_H */
