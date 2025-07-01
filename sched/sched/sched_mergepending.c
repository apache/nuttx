/****************************************************************************
 * sched/sched/sched_mergepending.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <stdbool.h>
#include <sched.h>
#include <assert.h>

#include <nuttx/queue.h>

#ifdef CONFIG_SMP
#  include <nuttx/spinlock.h>
#endif

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ALL_CPUS ((1 << CONFIG_SMP_NCPUS) - 1)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_merge_pending
 *
 * Description:
 *   This function merges the prioritized g_pendingtasks list into the
 *   prioritized ready-to-run task list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if the head of the ready-to-run task list has changed indicating
 *     a context switch is needed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function.
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run task list is changed.
 *
 ****************************************************************************/

#ifndef CONFIG_SMP
bool nxsched_merge_pending(void)
{
  FAR struct tcb_s *ptcb;
  FAR struct tcb_s *pnext;
  FAR struct tcb_s *rtcb;
  FAR struct tcb_s *rprev;
  bool ret = false;

  /* Initialize the inner search loop */

  rtcb = this_task();

  /* Process every TCB in the g_pendingtasks list
   *
   * Do nothing if pre-emption is still disabled
   */

  if (!nxsched_islocked_tcb(rtcb))
    {
      for (ptcb = (FAR struct tcb_s *)list_pendingtasks()->head;
           ptcb;
           ptcb = pnext)
        {
          pnext = ptcb->flink;

          /* REVISIT:  Why don't we just remove the ptcb from pending task
           * list and call nxsched_add_readytorun?
           */

          /* Search the ready-to-run list to find the location to insert the
           * new ptcb. Each is list is maintained in ascending sched_priority
           * order.
           */

          for (;
               (rtcb && ptcb->sched_priority <= rtcb->sched_priority);
               rtcb = rtcb->flink)
            {
            }

          /* Add the ptcb to the spot found in the list.  Check if the
           * ptcb goes at the ends of the ready-to-run list. This would be
           * error condition since the idle test must always be at the end of
           * the ready-to-run list!
           */

          DEBUGASSERT(rtcb);

          /* The ptcb goes just before rtcb */

          rprev = rtcb->blink;
          if (rprev == NULL)
            {
              /* Special case: Inserting ptcb at the head of the list */

              ptcb->flink       = rtcb;
              ptcb->blink       = NULL;
              rtcb->blink       = ptcb;
              list_readytorun()->head
                                = (FAR dq_entry_t *)ptcb;
              rtcb->task_state  = TSTATE_TASK_READYTORUN;
              ptcb->task_state  = TSTATE_TASK_RUNNING;
              up_update_task(ptcb);
              ret               = true;
            }
          else
            {
              /* Insert in the middle of the list */

              ptcb->flink       = rtcb;
              ptcb->blink       = rprev;
              rprev->flink      = ptcb;
              rtcb->blink       = ptcb;
              ptcb->task_state  = TSTATE_TASK_READYTORUN;
            }

          /* Set up for the next time through */

          rtcb = ptcb;
        }

      /* Mark the input list empty */

      list_pendingtasks()->head = NULL;
      list_pendingtasks()->tail = NULL;
    }

  return ret;
}
#endif /* !CONFIG_SMP */

/****************************************************************************
 * Name: nxsched_merge_pending
 *
 * Description:
 *   This function merges the prioritized g_pendingtasks list into the
 *   prioritized ready-to-run task list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   true if the head of the ready-to-run task list has changed indicating
 *     a context switch is needed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function.
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run task list is changed.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP

bool nxsched_merge_pending(void)
{
  FAR struct tcb_s *rtcb;
  FAR struct tcb_s *ptcb;
  FAR struct tcb_s *next;
  bool ret = false;
  int cpu;
  int minprio;

  ptcb = (FAR struct tcb_s *)dq_peek(list_readytorun());
  if (ptcb == NULL)
    {
      /* The readytorun task list is empty. */

      return false;
    }

  /* Find the CPU that is executing the lowest priority task. */

  cpu = nxsched_select_cpu(ALL_CPUS);
  rtcb = current_delivered(cpu);
  minprio = rtcb->sched_priority;

  /* Loop while there is a higher priority task in the ready-to-run list
   * than in the lowest executing task.
   *
   * Normally, this loop should execute no more than CONFIG_SMP_NCPUS
   * times.  That number could be larger, however, if the CPU affinity
   * sets do not include all CPUs. In that case, the excess TCBs will
   * be left in the g_readytorun list.
   */

  while (ptcb && ptcb->sched_priority > minprio)
    {
      /* If the ptcb is not allowed to run on all CPU's re-select the
       * CPU. This is unlikely, so not worth doing on every cycle.
       */

      if (ptcb->affinity != ALL_CPUS)
        {
          cpu  = nxsched_select_cpu(ptcb->affinity);
          rtcb = current_delivered(cpu);
        }

      next = dq_next(ptcb);

      if (ptcb->sched_priority > rtcb->sched_priority)
        {
          /* Remove the task from the readytorun task list. */

          dq_rem((dq_entry_t *)ptcb, list_readytorun());

          /* Add the task again to the correct list. */

          ret |= nxsched_add_readytorun(ptcb);
        }

      /* Re-check the minimum priority. */

      cpu = nxsched_select_cpu(ALL_CPUS);
      rtcb = current_delivered(cpu);
      minprio = rtcb->sched_priority;

      ptcb = next;
    }

  return ret;
}
#endif /* CONFIG_SMP */
