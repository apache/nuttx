/****************************************************************************
 * sched/sched/sched_addreadytorun.c
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

#include "irq/irq.h"
#include "sched/queue.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_add_readytorun
 *
 * Description:
 *   This function adds a TCB to the ready to run list.  If the currently
 *   active task has preemption disabled and the new TCB would cause this
 *   task to be pre-empted, the new task is added to the g_pendingtasks list
 *   instead.  The pending tasks will be made ready-to-run when preemption is
 *   unlocked.
 *
 * Input Parameters:
 *   btcb - Points to the blocked TCB that is ready-to-run
 *
 * Returned Value:
 *   true if the currently active task (the head of the ready-to-run list)
 *   has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 * - The caller has already removed the input rtcb from whatever list it
 *   was in.
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run list is changed.
 *
 ****************************************************************************/

#ifndef CONFIG_SMP
bool nxsched_add_readytorun(FAR struct tcb_s *btcb)
{
  FAR struct tcb_s *rtcb = this_task();
  bool ret;

  /* Check if pre-emption is disabled for the current running task and if
   * the new ready-to-run task would cause the current running task to be
   * pre-empted.  NOTE that IRQs disabled implies that pre-emption is
   * also disabled.
   */

  if (rtcb->lockcount > 0 && rtcb->sched_priority < btcb->sched_priority)
    {
      /* Yes.  Preemption would occur!  Add the new ready-to-run task to the
       * g_pendingtasks task list for now.
       */

      nxsched_add_prioritized(btcb, list_pendingtasks());
      btcb->task_state = TSTATE_TASK_PENDING;
      ret = false;
    }

  /* Otherwise, add the new task to the ready-to-run task list */

  else if (nxsched_add_prioritized(btcb, list_readytorun()))
    {
      /* The new btcb was added at the head of the ready-to-run list.  It
       * is now the new active task!
       */

      DEBUGASSERT(rtcb->lockcount == 0 && !is_idle_task(btcb));

      btcb->task_state = TSTATE_TASK_RUNNING;
      btcb->flink->task_state = TSTATE_TASK_READYTORUN;
      ret = true;
    }
  else
    {
      /* The new btcb was added in the middle of the ready-to-run list */

      btcb->task_state = TSTATE_TASK_READYTORUN;
      ret = false;
    }

  return ret;
}
#endif /* !CONFIG_SMP */

/****************************************************************************
 * Name:  nxsched_add_readytorun
 *
 * Description:
 *   This function adds a TCB to one of the ready to run lists.  That might
 *   be:
 *
 *   1. The g_readytorun list if the task is ready-to-run but not running
 *      and not assigned to a CPU.
 *   2. The g_assignedtask[cpu] list if the task is running or if has been
 *      assigned to a CPU.
 *
 *   If the currently active task has preemption disabled and the new TCB
 *   would cause this task to be pre-empted, the new task is added to the
 *   g_pendingtasks list instead.  The pending tasks will be made
 *   ready-to-run when preemption isunlocked.
 *
 * Input Parameters:
 *   btcb - Points to the blocked TCB that is ready-to-run
 *
 * Returned Value:
 *   true if the currently active task (the head of the ready-to-run list)
 *   has changed.
 *
 * Assumptions:
 * - The caller has established a critical section before calling this
 *   function (calling sched_lock() first is NOT a good idea -- use
 *   enter_critical_section()).
 * - The caller has already removed the input rtcb from whatever list it
 *   was in.
 * - The caller handles the condition that occurs if the head of the
 *   ready-to-run list has changed.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
bool nxsched_add_readytorun(FAR struct tcb_s *btcb)
{
  FAR struct tcb_s *rtcb;
  FAR struct tcb_s *headtcb;
  FAR dq_queue_t *tasklist;
  bool doswitch;
  int task_state;
  int cpu;
  int me;

  cpu = nxsched_select_cpu(btcb->affinity);

  /* Get the task currently running on the CPU (may be the IDLE task) */

  rtcb = current_task(cpu);

  /* Determine the desired new task state.  First, if the new task priority
   * is higher then the priority of the lowest priority, running task, then
   * the new task will be running and a context switch switch will be
   * required.
   */

  if (rtcb->sched_priority < btcb->sched_priority)
    {
      task_state = TSTATE_TASK_RUNNING;
    }
  else
    {
      task_state = TSTATE_TASK_READYTORUN;
    }

  /* If the selected state is TSTATE_TASK_RUNNING, then we would like to
   * start running the task.  Be we cannot do that if pre-emption is
   * disabled.  If the selected state is TSTATE_TASK_READYTORUN, then it
   * should also go to the pending task list so that it will have a chance
   * to be restarted when the scheduler is unlocked.
   *
   * There is an interaction here with IRQ locking.  Even if the pre-
   * emption is enabled, tasks will be forced to pend if the IRQ lock
   * is also set UNLESS the CPU starting the thread is also the holder of
   * the IRQ lock.  irq_cpu_locked() performs an atomic check for that
   * situation.
   */

  if (nxsched_islocked_global())
    {
      /* Add the new ready-to-run task to the g_pendingtasks task list for
       * now.
       */

      nxsched_add_prioritized(btcb, list_pendingtasks());
      btcb->task_state = TSTATE_TASK_PENDING;
      doswitch         = false;
    }
  else if (task_state == TSTATE_TASK_READYTORUN)
    {
      /* The new btcb was added either (1) in the middle of the assigned
       * task list (the btcb->cpu field is already valid) or (2) was
       * added to the ready-to-run list (the btcb->cpu field does not
       * matter).  Either way, it won't be running.
       *
       * Add the task to the ready-to-run (but not running) task list
       */

      nxsched_add_prioritized(btcb, list_readytorun());

      btcb->task_state = TSTATE_TASK_READYTORUN;
      doswitch         = false;
    }
  else /* (task_state == TSTATE_TASK_RUNNING) */
    {
      /* If we are modifying some assigned task list other than our own, we
       * will need to switch that CPU.
       */

      me = this_cpu();
      if (cpu != me)
        {
          if (g_delivertasks[cpu] == NULL)
            {
              g_delivertasks[cpu] = btcb;
              btcb->cpu = cpu;
              btcb->task_state = TSTATE_TASK_ASSIGNED;
              up_cpu_pause_async(cpu);
            }
          else
            {
              rtcb = g_delivertasks[cpu];
              if (rtcb->sched_priority < btcb->sched_priority)
                {
                  g_delivertasks[cpu] = btcb;
                  btcb->cpu = cpu;
                  btcb->task_state = TSTATE_TASK_ASSIGNED;
                  nxsched_add_prioritized(rtcb, &g_readytorun);
                  rtcb->task_state = TSTATE_TASK_READYTORUN;
                }
              else
                {
                  nxsched_add_prioritized(btcb, &g_readytorun);
                  btcb->task_state = TSTATE_TASK_READYTORUN;
                }
            }

          return false;
        }

      tasklist = &g_assignedtasks[cpu];

      /* Change "head" from TSTATE_TASK_RUNNING to TSTATE_TASK_ASSIGNED */

      headtcb = (FAR struct tcb_s *)tasklist->head;
      DEBUGASSERT(headtcb->task_state = TSTATE_TASK_RUNNING);
      headtcb->task_state = TSTATE_TASK_ASSIGNED;

      /* Add btcb to the head of the g_assignedtasks
       * task list and mark it as running
       */

      dq_addfirst_nonempty((FAR dq_entry_t *)btcb, tasklist);

      DEBUGASSERT(task_state == TSTATE_TASK_RUNNING);
      btcb->cpu        = cpu;
      btcb->task_state = TSTATE_TASK_RUNNING;

      doswitch = true;

      /* Resume scheduling lock */

      DEBUGASSERT(g_cpu_lockset == 0);
      if (btcb->lockcount > 0)
        {
          g_cpu_lockset |= (1 << cpu);
        }
    }

  return doswitch;
}

#endif /* CONFIG_SMP */
