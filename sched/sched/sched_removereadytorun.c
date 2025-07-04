/****************************************************************************
 * sched/sched/sched_removereadytorun.c
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
#include <assert.h>

#include <nuttx/sched_note.h>

#include "irq/irq.h"
#include "sched/queue.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_remove_readytorun
 *
 * Description:
 *   This function removes a TCB from the ready to run list.
 *
 * Input Parameters:
 *   rtcb - Points to the TCB that is ready-to-run
 *
 * Returned Value:
 *   true if the currently active task (the head of the ready-to-run list)
 *     has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run list is changed.
 *
 ****************************************************************************/

#ifndef CONFIG_SMP
bool nxsched_remove_readytorun(FAR struct tcb_s *rtcb)
{
  FAR dq_queue_t *tasklist;
  bool doswitch = false;

  tasklist = TLIST_HEAD(rtcb);

  /* Check if the TCB to be removed is at the head of the ready to run list.
   * There is only one list, g_readytorun, and it always contains the
   * currently running task.  If we are removing the head of this list,
   * then we are removing the currently active task.
   */

  if (rtcb->blink == NULL && TLIST_ISRUNNABLE(rtcb->task_state))
    {
      /* There must always be at least one task in the list (the IDLE task)
       * after the TCB being removed.
       */

      FAR struct tcb_s *nxttcb = (FAR struct tcb_s *)rtcb->flink;
      DEBUGASSERT(nxttcb != NULL);

      nxttcb->task_state = TSTATE_TASK_RUNNING;
      up_update_task(nxttcb);
      doswitch = true;
    }

  /* Remove the TCB from the ready-to-run list.  In the non-SMP case, this
   * is always the g_readytorun list.
   */

  dq_rem((FAR dq_entry_t *)rtcb, tasklist);

  /* Since the TCB is not in any list, it is now invalid */

  rtcb->task_state = TSTATE_TASK_INVALID;

  return doswitch;
}

void nxsched_remove_self(FAR struct tcb_s *tcb)
{
  nxsched_remove_readytorun(tcb);
  if (list_pendingtasks()->head)
    {
      nxsched_merge_pending();
    }
}
#endif /* !CONFIG_SMP */

/****************************************************************************
 * Name: nxsched_remove_readytorun
 *
 * Description:
 *   This function removes a TCB from the ready to run list.
 *
 * Input Parameters:
 *   rtcb - Points to the TCB that is ready-to-run
 *
 * Returned Value:
 *   true if the currently active task (the head of the ready-to-run list)
 *     has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run list is changed.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static void nxsched_remove_running(FAR struct tcb_s *tcb)
{
  FAR dq_queue_t *tasklist;
  FAR struct tcb_s *nxttcb;
  int cpu;

  /* Which CPU (if any) is the task running on?  Which task list holds the
   * TCB?
   */

  DEBUGASSERT(tcb->cpu == this_cpu() &&
              tcb->task_state == TSTATE_TASK_RUNNING);

  cpu = tcb->cpu;
  tasklist = &g_assignedtasks[cpu];

  /* Check if the TCB to be removed is at the head of a running list.
   * For the case of SMP, there are two lists involved:  (1) the
   * g_readytorun list that holds non-running tasks that have not been
   * assigned to a CPU, and (2) and the g_assignedtasks[] lists which hold
   * tasks assigned a CPU, including the task that is currently running on
   * that CPU.  Only this latter list contains the currently active task
   * only removing the head of that list can result in a context switch.
   *
   * tcb->blink == NULL will tell us if the TCB is at the head of the
   * running list and, hence, a candidate for the new running task.
   *
   * If so, then the tasklist RUNNABLE attribute will inform us if the list
   * holds the currently executing task and, hence, if a context switch
   * should occur.
   */

  DEBUGASSERT(tcb->blink == NULL);
  DEBUGASSERT(TLIST_ISRUNNABLE(tcb->task_state));

  /* There must always be at least one task in the list (the IDLE task)
   * after the TCB being removed.
   */

  nxttcb = tcb->flink;
  DEBUGASSERT(nxttcb != NULL && is_idle_task(nxttcb));

  /* The task is running but the CPU that it was running on has been
   * paused.  We can now safely remove its TCB from the running
   * task list.
   */

  dq_remfirst(tasklist);

  /* Since the TCB is no longer in any list, it is now invalid */

  tcb->task_state = TSTATE_TASK_INVALID;

  /* Activate the idle task */

  nxttcb->task_state = TSTATE_TASK_RUNNING;
  up_update_task(nxttcb);
}

void nxsched_remove_self(FAR struct tcb_s *tcb)
{
  nxsched_remove_running(tcb);
  nxsched_switch_running(tcb->cpu, false);
}

bool nxsched_remove_readytorun(FAR struct tcb_s *tcb)
{
  if (tcb->task_state == TSTATE_TASK_RUNNING)
    {
      nxsched_remove_running(tcb);
      return true;
    }
  else
    {
      FAR dq_queue_t *tasklist;

      tasklist = TLIST_HEAD(tcb, tcb->cpu);

      /* The task is not running.  Just remove its TCB from the task
       * list.  In the SMP case this may be either the g_readytorun() or the
       * g_assignedtasks[cpu] list.
       */

      dq_rem((FAR dq_entry_t *)tcb, tasklist);

      /* Since the TCB is no longer in any list, it is now invalid */

      tcb->task_state = TSTATE_TASK_INVALID;
    }

  return false;
}
#endif /* CONFIG_SMP */
