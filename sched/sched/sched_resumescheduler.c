/****************************************************************************
 * sched/sched/sched_resumescheduler.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/sched_note.h>

#include "irq/irq.h"
#include "sched/sched.h"

#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC) || \
    defined(CONFIG_SCHED_INSTRUMENTATION) || defined(CONFIG_SMP)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_resume_scheduler
 *
 * Description:
 *   Called by architecture specific implementations that block task
 *   execution.  This function prepares the scheduler for the thread that is
 *   about to be restarted.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread to be restarted.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void sched_resume_scheduler(FAR struct tcb_s *tcb)
{
#if CONFIG_RR_INTERVAL > 0
#ifdef CONFIG_SCHED_SPORADIC
  if ((tcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_RR)
#endif
    {
      /* Reset the task's timeslice. */

      tcb->timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);
    }
#endif

#ifdef CONFIG_SCHED_SPORADIC
#if CONFIG_RR_INTERVAL > 0
  else
#endif
  if ((tcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      /* Reset the replenishment cycle if it is appropriate to do so */

      DEBUGVERIFY(sched_sporadic_resume(tcb));
    }
#endif

#ifdef CONFIG_SCHED_INSTRUMENTATION
  /* Inidicate the task has been resumed */

  sched_note_resume(tcb);
#endif

#ifdef CONFIG_SMP
  /* NOTE: The following logic for adjusting global IRQ controls were
   * derived from sched_addreadytorun() and sched_removedreadytorun()
   * Here, we only handles clearing logic to defer unlocking IRQ lock
   * followed by context switching.
   */

  int me = this_cpu();

  /* Adjust global IRQ controls.  If irqcount is greater than zero,
   * then this task/this CPU holds the IRQ lock
   */

  if (tcb->irqcount > 0)
    {
      /* Do notihing here
       * NOTE: spin_setbit() is done in sched_addreadytorun()
       * and sched_removereadytorun()
       */
    }

  /* No.. This CPU will be relinquishing the lock.  But this works
   * differently if we are performing a context switch from an
   * interrupt handler and the interrupt handler has established
   * a critical section.  We can detect this case when
   * g_cpu_nestcount[me] > 0.
   */

  else if (g_cpu_nestcount[me] <= 0)
    {
      /* Release our hold on the IRQ lock. */

      spin_clrbit(&g_cpu_irqset, me, &g_cpu_irqsetlock,
                  &g_cpu_irqlock);
    }
#endif /* CONFIG_SMP */

}

#endif /* CONFIG_RR_INTERVAL > 0 || CONFIG_SCHED_SPORADIC || \
        * CONFIG_SCHED_INSTRUMENTATION || CONFIG_SMP */
