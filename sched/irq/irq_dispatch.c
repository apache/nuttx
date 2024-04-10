/****************************************************************************
 * sched/irq/irq_dispatch.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/mm/mm.h>
#include <nuttx/random.h>
#include <nuttx/sched_note.h>

#include "irq/irq.h"
#include "clock/clock.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
#  define NUSER_IRQS CONFIG_ARCH_NUSER_INTERRUPTS
#else
#  define NUSER_IRQS NR_IRQS
#endif

/* CALL_VECTOR - Call the interrupt service routine attached to this
 * interrupt request
 */

#ifndef CONFIG_SCHED_CRITMONITOR_MAXTIME_IRQ
#  define CONFIG_SCHED_CRITMONITOR_MAXTIME_IRQ 0
#endif

#ifdef CONFIG_SCHED_IRQMONITOR
#  define CALL_VECTOR(ndx, vector, irq, context, arg) \
     do \
       { \
         clock_t start; \
         clock_t elapsed; \
         start = perf_gettime(); \
         vector(irq, context, arg); \
         elapsed = perf_gettime() - start; \
         if (ndx < NUSER_IRQS) \
           { \
             g_irqvector[ndx].count++; \
             if (elapsed > g_irqvector[ndx].time) \
               { \
                 g_irqvector[ndx].time = elapsed; \
               } \
           } \
         if (CONFIG_SCHED_CRITMONITOR_MAXTIME_IRQ > 0 && \
             elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_IRQ) \
           { \
             CRITMONITOR_PANIC("IRQ %d(%p), execute time too long %ju\n", \
                               irq, vector, (uintmax_t)elapsed); \
           } \
       } \
     while (0)
#else
#  define CALL_VECTOR(ndx, vector, irq, context, arg) \
     vector(irq, context, arg)
#endif /* CONFIG_SCHED_IRQMONITOR */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: irq_dispatch
 *
 * Description:
 *   This function must be called from the architecture-specific logic in
 *   order to dispatch an interrupt to the appropriate, registered handling
 *   logic.
 *
 ****************************************************************************/

void irq_dispatch(int irq, FAR void *context)
{
#ifdef CONFIG_DEBUG_MM
  struct tcb_s *rtcb = this_task();
#endif
  xcpt_t vector = irq_unexpected_isr;
  FAR void *arg = NULL;
  unsigned int ndx = irq;

#if NR_IRQS > 0
  if ((unsigned)irq < NR_IRQS)
    {
#ifdef CONFIG_ARCH_MINIMAL_VECTORTABLE
      ndx = g_irqmap[irq];
      if (ndx < CONFIG_ARCH_NUSER_INTERRUPTS)
        {
          if (g_irqvector[ndx].handler)
            {
              vector = g_irqvector[ndx].handler;
              arg    = g_irqvector[ndx].arg;
            }
        }
#else
      if (g_irqvector[ndx].handler)
        {
          vector = g_irqvector[ndx].handler;
          arg    = g_irqvector[ndx].arg;
        }
#endif
    }
#endif

#ifdef CONFIG_CRYPTO_RANDOM_POOL_COLLECT_IRQ_RANDOMNESS
  /* Add interrupt timing randomness to entropy pool */

  add_irq_randomness(irq);
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  /* Notify that we are entering into the interrupt handler */

  sched_note_irqhandler(irq, vector, true);
#endif

  /* Then dispatch to the interrupt handler */

  CALL_VECTOR(ndx, vector, irq, context, arg);
  UNUSED(ndx);

#ifdef CONFIG_SCHED_INSTRUMENTATION_IRQHANDLER
  /* Notify that we are leaving from the interrupt handler */

  sched_note_irqhandler(irq, vector, false);
#endif

#ifdef CONFIG_DEBUG_MM
  if ((rtcb->flags & TCB_FLAG_HEAP_CHECK) ||
      (this_task()->flags & TCB_FLAG_HEAP_CHECK))
    {
      kmm_checkcorruption();
    }
#endif
}
