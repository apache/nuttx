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
 *   merge - Merge pending list or not
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
bool nxsched_remove_readytorun(FAR struct tcb_s *rtcb, bool merge)
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
      doswitch = true;
    }

  /* Remove the TCB from the ready-to-run list.  In the non-SMP case, this
   * is always the g_readytorun list.
   */

  dq_rem((FAR dq_entry_t *)rtcb, tasklist);

  /* Since the TCB is not in any list, it is now invalid */

  rtcb->task_state = TSTATE_TASK_INVALID;

  if (list_pendingtasks()->head && merge)
    {
      doswitch |= nxsched_merge_pending();
    }

  return doswitch;
}

void nxsched_remove_self(FAR struct tcb_s *tcb)
{
  nxsched_remove_readytorun(tcb, true);
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
 *   merge - Merge pending list or not
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
void nxsched_remove_running(FAR struct tcb_s *tcb)
{
  FAR dq_queue_t *tasklist;
  FAR struct tcb_s *nxttcb;
  FAR struct tcb_s *rtrtcb = NULL;
  int cpu;

  /* Which CPU (if any) is the task running on?  Which task list holds the
   * TCB?
   */

  DEBUGASSERT(tcb->task_state == TSTATE_TASK_RUNNING);
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
  DEBUGASSERT(nxttcb != NULL);

  /* The task is running but the CPU that it was running on has been
   * paused.  We can now safely remove its TCB from the running
   * task list.  In the SMP case this may be either the g_readytorun()
   * or the g_assignedtasks[cpu] list.
   */

  dq_rem_head((FAR dq_entry_t *)tcb, tasklist);

  /* Find the highest priority non-running tasks in the g_assignedtasks
   * list of other CPUs, and also non-idle tasks, place them in the
   * g_readytorun list. so as to find the task with the highest priority,
   * globally
   */

  for (int i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      if (i == cpu)
        {
          /* The highest priority task of the current
           * CPU has been found, which is nxttcb.
           */

          continue;
        }

      for (rtrtcb = (FAR struct tcb_s *)g_assignedtasks[i].head;
                !is_idle_task(rtrtcb); rtrtcb = rtrtcb->flink)
        {
          if (rtrtcb->task_state != TSTATE_TASK_RUNNING &&
              CPU_ISSET(cpu, &rtrtcb->affinity))
            {
              /* We have found the task with the highest priority whose
               * CPU index is i. Since this task must be between the two
               * tasks, we can use the dq_rem_mid macro to delete it.
               */

              dq_rem_mid(rtrtcb);
              rtrtcb->task_state = TSTATE_TASK_READYTORUN;

              /* Add rtrtcb to g_readytorun to find
               * the task with the highest global priority
               */

              nxsched_add_prioritized(rtrtcb, &g_readytorun);
              break;
            }
        }
    }

  /* Which task will go at the head of the list?  It will be either the
   * next tcb in the assigned task list (nxttcb) or a TCB in the
   * g_readytorun list.  We can only select a task from that list if
   * the affinity mask includes the current CPU.
   */

  /* Search for the highest priority task that can run on this
   * CPU.
   */

  for (rtrtcb = (FAR struct tcb_s *)g_readytorun.head;
        rtrtcb != NULL && !CPU_ISSET(cpu, &rtrtcb->affinity);
        rtrtcb = rtrtcb->flink);

  /* Did we find a task in the g_readytorun list?  Which task should
   * we use?  We decide strictly by the priority of the two tasks:
   * Either (1) the task currently at the head of the
   * g_assignedtasks[cpu] list (nexttcb) or (2) the highest priority
   * task from the g_readytorun list with matching affinity (rtrtcb).
   */

  if (rtrtcb != NULL && rtrtcb->sched_priority >= nxttcb->sched_priority)
    {
      /* The TCB rtrtcb has the higher priority and it can be run on
       * target CPU. Remove that task (rtrtcb) from the g_readytorun
       * list and add to the head of the g_assignedtasks[cpu] list.
       */

      dq_rem((FAR dq_entry_t *)rtrtcb, &g_readytorun);
      dq_addfirst_nonempty((FAR dq_entry_t *)rtrtcb, tasklist);

      rtrtcb->cpu = cpu;
      nxttcb = rtrtcb;
    }

  /* Will pre-emption be disabled after the switch?  If the lockcount is
   * greater than zero, then this task/this CPU holds the scheduler lock.
   */

  if (nxttcb->lockcount > 0)
    {
      /* Yes... make sure that scheduling logic knows about this */

      g_cpu_lockset |= (1 << cpu);
    }
  else
    {
      /* No.. we may need to perform release our hold on the lock. */

      g_cpu_lockset &= ~(1 << cpu);
    }

  /* NOTE: If the task runs on another CPU(cpu), adjusting global IRQ
   * controls will be done in the pause handler on the new CPU(cpu).
   * If the task is scheduled on this CPU(me), do nothing because
   * this CPU already has a critical section
   */

  nxttcb->task_state = TSTATE_TASK_RUNNING;

  /* Since the TCB is no longer in any list, it is now invalid */

  tcb->task_state = TSTATE_TASK_INVALID;
}

void nxsched_remove_self(FAR struct tcb_s *tcb)
{
  nxsched_remove_running(tcb);
  if (g_pendingtasks.head)
    {
      nxsched_merge_pending();
    }
}

bool nxsched_remove_readytorun(FAR struct tcb_s *tcb, bool merge)
{
  bool doswitch = false;

  if (tcb->task_state == TSTATE_TASK_RUNNING)
    {
      int me = this_cpu();
      int cpu = tcb->cpu;
      if (cpu != me)
        {
          up_cpu_pause(tcb->cpu);
          nxsched_remove_running(tcb);
          up_cpu_resume(tcb->cpu);
        }
      else
        {
          nxsched_remove_running(tcb);
          doswitch = true;
        }
    }
  else
    {
      FAR dq_queue_t *tasklist;
      int i;

      /* if tcb == g_delivertasks[i] we set NULL to g_delivertasks[i] */

      for (i = 0; i < CONFIG_SMP_NCPUS; i++)
        {
          if (tcb == g_delivertasks[i])
            {
              g_delivertasks[i] = NULL;
              tcb->task_state = TSTATE_TASK_INVALID;
              goto finish;
            }
        }

      tasklist = TLIST_HEAD(tcb, tcb->cpu);

      DEBUGASSERT(tcb->task_state != TSTATE_TASK_RUNNING);

      /* The task is not running.  Just remove its TCB from the task
       * list.  In the SMP case this may be either the g_readytorun() or the
       * g_assignedtasks[cpu] list.
       */

      dq_rem((FAR dq_entry_t *)tcb, tasklist);

      /* Since the TCB is no longer in any list, it is now invalid */

      tcb->task_state = TSTATE_TASK_INVALID;
    }

finish:
  if (list_pendingtasks()->head && merge)
    {
      doswitch |= nxsched_merge_pending();
    }

  return doswitch;
}
#endif /* CONFIG_SMP */
