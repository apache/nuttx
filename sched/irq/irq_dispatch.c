/****************************************************************************
 * sched/irq/irq_dispatch.c
 *
 *   Copyright (C) 2007, 2008, 2017-2018 Gregory Nutt. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/random.h>

#include "irq/irq.h"
#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* INCR_COUNT - Increment the count of interrupts taken on this IRQ number */

#ifdef CONFIG_SCHED_IRQMONITOR
#  ifdef CONFIG_HAVE_LONG_LONG
#    define INCR_COUNT(ndx) \
       do \
         { \
           g_irqvector[ndx].count++; \
         } \
       while (0)
#  else
#    define INCR_COUNT(ndx) \
       do \
         { \
           if (++g_irqvector[ndx].lscount == 0) \
             { \
               g_irqvector[ndx].mscount++; \
             } \
         } \
       while (0)
#  endif
#else
#  define INCR_COUNT(ndx)
#endif

/* CALL_VECTOR - Call the interrupt service routine attached to this interrupt
 * request
 */

#if defined(CONFIG_SCHED_IRQMONITOR) && defined(CONFIG_SCHED_TICKLESS)
#  define CALL_VECTOR(ndx, vector, irq, context, arg) \
     do \
       { \
         struct timespec start; \
         struct timespec end; \
         struct timespec delta; \
         clock_systimespec(&start); \
         vector(irq, context, arg); \
         clock_systimespec(&end); \
         clock_timespec_subtract(&end, &start, &delta); \
         if (delta.tv_nsec > g_irqvector[ndx].time) \
           { \
             g_irqvector[ndx].time = delta.tv_nsec; \
           } \
       } \
     while (0)
#else
#  define CALL_VECTOR(ndx, vector, irq, context, arg) \
     vector(irq, context, arg)
#endif

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

          INCR_COUNT(ndx);
        }
#else
      if (g_irqvector[ndx].handler)
        {
          vector = g_irqvector[ndx].handler;
          arg    = g_irqvector[ndx].arg;
        }

      INCR_COUNT(ndx);
#endif
    }
#endif

#ifdef CONFIG_CRYPTO_RANDOM_POOL_COLLECT_IRQ_RANDOMNESS
  /* Add interrupt timing randomness to entropy pool */

  add_irq_randomness(irq);
#endif

  /* Then dispatch to the interrupt handler */

  CALL_VECTOR(ndx, vector, irq, context, arg);
  UNUSED(ndx);
}
