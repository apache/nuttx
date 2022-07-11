/****************************************************************************
 * sched/sched/sched_resumescheduler.c
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

#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/clock.h>
#include <nuttx/sched_note.h>

#include "irq/irq.h"
#include "sched/sched.h"

#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_RESUMESCHEDULER)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_resume_scheduler
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

void nxsched_resume_scheduler(FAR struct tcb_s *tcb)
{
#ifdef CONFIG_SCHED_SPORADIC
  if ((tcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      /* Reset the replenishment cycle if it is appropriate to do so */

      DEBUGVERIFY(nxsched_resume_sporadic(tcb));
    }
#endif

  /* Indicate the task has been resumed */

#ifdef CONFIG_SCHED_CRITMONITOR
  nxsched_resume_critmon(tcb);
#endif
#ifdef CONFIG_SCHED_INSTRUMENTATION
  sched_note_resume(tcb);
#endif

#ifdef CONFIG_SMP
  /* NOTE: The following logic for adjusting global IRQ controls were
   * derived from nxsched_add_readytorun() and sched_removedreadytorun()
   * Here, we only handles clearing logic to defer unlocking IRQ lock
   * followed by context switching.
   */

  int me = this_cpu();

  /* Adjust global IRQ controls.  If irqcount is greater than zero,
   * then this task/this CPU holds the IRQ lock
   */

  if (tcb->irqcount > 0)
    {
      /* Do nothing here
       * NOTE: spin_setbit() is done in nxsched_add_readytorun()
       * and nxsched_remove_readytorun()
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

      if ((g_cpu_irqset & (1 << me)) != 0)
        {
          spin_clrbit(&g_cpu_irqset, me, &g_cpu_irqsetlock,
                      &g_cpu_irqlock);
        }
    }
#endif /* CONFIG_SMP */
}

#endif /* CONFIG_RR_INTERVAL > 0 || CONFIG_SCHED_RESUMESCHEDULER */
