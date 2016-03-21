/****************************************************************************
 * sched/irq/irq_csection.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
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

#include <sys/types.h>

#include <nuttx/spinlock.h>
#include <nuttx/sched_note.h>
#include <arch/irq.h>

#include "sched/sched.h"
#include "irq/irq.h"

#if defined(CONFIG_SMP) || defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION)

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SMP
/* This is the spinlock that enforces critical sections when interrupts are
 * disabled.
 */

volatile spinlock_t g_cpu_irqlock = SP_UNLOCKED;

/* Used to keep track of which CPU(s) hold the IRQ lock. */

volatile spinlock_t g_cpu_irqsetlock;
volatile cpu_set_t g_cpu_irqset;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: enter_critical_section
 *
 * Description:
 *   Take the CPU IRQ lock and disable interrupts on all CPUs.  A thread-
 *   specific counter is increment to indicate that the thread has IRQs
 *   disabled and to support nested calls to enter_critical_section().
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
irqstate_t enter_critical_section(void)
{
  FAR struct tcb_s *rtcb;

  /* Do nothing if called from an interrupt handler */

  if (up_interrupt_context())
    {
      /* The value returned does not matter.  We assume only that it is a
       * scalar here.
       */

      return (irqstate_t)0;
    }

  /* Do we already have interrupts disabled? */

  rtcb = this_task();
  DEBUGASSERT(rtcb != NULL);

  if (rtcb->irqcount > 0)
    {
      /* Yes... make sure that the spinlock is set and increment the IRQ
       * lock count.
       */

      DEBUGASSERT(g_cpu_irqlock == SP_LOCKED && rtcb->irqcount < INT16_MAX);
      rtcb->irqcount++;
    }
  else
    {
      /* NO.. Take the spinlock to get exclusive access and set the lock
       * count to 1.
       *
       * We must avoid that case where a context occurs between taking the
       * g_cpu_irqlock and disabling interrupts.  Also interrupts disables
       * must follow a stacked order.  We cannot other context switches to
       * re-order the enabling/disabling of interrupts.
       *
       * The scheduler accomplishes this by treating the irqcount like
       * lockcount:  Both will disable pre-emption.
       */

      spin_setbit(&g_cpu_irqset, this_cpu(), &g_cpu_irqsetlock,
                  &g_cpu_irqlock);
      rtcb->irqcount = 1;

#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
      /* Note that we have entered the critical section */

      sched_note_csection(rtcb, true);
#endif
    }

  /* Then disable interrupts (they may already be disabled, be we need to
   * return valid interrupt status in any event).
   */

  return up_irq_save();
}
#else /* defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION) */
irqstate_t enter_critical_section(void)
{
  /* Check if we were called from an interrupt handler */

  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();
      DEBUGASSERT(rtcb != NULL);

      /* No.. note that we have entered the critical section */

      sched_note_csection(rtcb, true);
    }

  /* And disable interrupts */

  return up_irq_save();
}
#endif

/****************************************************************************
 * Name: leave_critical_section
 *
 * Description:
 *   Decrement the IRQ lock count and if it decrements to zero then release
 *   the spinlock.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
void leave_critical_section(irqstate_t flags)
{
  /* Do nothing if called from an interrupt handler */

  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();
      DEBUGASSERT(rtcb != 0 && rtcb->irqcount > 0);

      /* Will we still have interrupts disabled after decrementing the
       * count?
       */

      if (rtcb->irqcount > 1)
        {
          /* Yes... make sure that the spinlock is set */

          DEBUGASSERT(g_cpu_irqlock == SP_LOCKED);
          rtcb->irqcount--;
        }
      else
        {
#ifdef CONFIG_SCHED_INSTRUMENTATION_CSECTION
          /* No.. Note that we have entered the critical section */

          sched_note_csection(rtcb, false);
#endif
          /* Decrement our count on the lock.  If all CPUs have released,
           * then unlock the spinlock.
           */

          rtcb->irqcount = 0;
          spin_clrbit(&g_cpu_irqset, this_cpu(), &g_cpu_irqsetlock,
                      &g_cpu_irqlock);

          /* Have all CPUs release the lock? */

          if (!spin_islocked(&g_cpu_irqlock))
            {
              /* Check if there are pending tasks and that pre-emption is
               * also enabled.
               */

              if (g_pendingtasks.head != NULL && !spin_islocked(&g_cpu_schedlock))
                {
                  /* Release any ready-to-run tasks that have collected in
                   * g_pendingtasks if the scheduler is not locked.
                   *
                   * NOTE: This operation has a very high likelihood of causing
                   * this task to be switched out!
                   */

                  up_release_pending();
                }
            }
        }

      /* Restore the previous interrupt state which may still be interrupts
       * disabled (but we don't have a mechanism to verify that now)
       */

      up_irq_restore(flags);
    }
}
#else /* defined(CONFIG_SCHED_INSTRUMENTATION_CSECTION) */
void leave_critical_section(irqstate_t flags)
{
  /* Check if we were called from an interrupt handler */

  if (!up_interrupt_context())
    {
      FAR struct tcb_s *rtcb = this_task();
      DEBUGASSERT(rtcb != NULL);

      /* Note that we have left the critical section */

      sched_note_csection(rtcb, false);
    }

  /* Restore the previous interrupt state. */

  up_irq_restore(flags);
}
#endif

#endif /* CONFIG_SMP || CONFIG_SCHED_INSTRUMENTATION_CSECTION*/
