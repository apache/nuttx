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
 *   task to be preempted, the new task is added to the g_pendingtasks list
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
   * preempted.  NOTE that IRQs disabled implies that pre-emption is
   * also disabled.
   */

  if (nxsched_islocked_tcb(rtcb) &&
      rtcb->sched_priority < btcb->sched_priority)
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

      DEBUGASSERT(!nxsched_islocked_tcb(rtcb) && !is_idle_task(btcb));

      btcb->task_state = TSTATE_TASK_RUNNING;
      btcb->flink->task_state = TSTATE_TASK_READYTORUN;
      up_update_task(btcb);
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

#else /* !CONFIG_SMP */

/****************************************************************************
 * Name:  nxsched_switch_running
 *
 * Description:
 *   This function switches the head of the current CPU's assigned tasks
 *   list to the TCB given as parameter. The idle task is not switched out.
 *   If the running task can't be swapped out, the btcb is pushed to
 *   the ready-to-run list.
 *
 * Input Parameters:
 *   cpu          - Always this_cpu(). Given as argument only for
 *                  optimization
 *   switch_equal - When true, switch away a task of equal priority compared
 *                  to the pending one
 *
 * Returned Value:
 *   true if the currently active task is switched
 *
 * Assumptions:
 * - The caller has established a critical section
 * - The caller has already removed the input rtcb from whatever list it
 *   was in.
 * - The caller handles the condition that occurs if the head of the
 *   assigned tasks list has changed.
 *
 ****************************************************************************/

bool nxsched_switch_running(int cpu, bool switch_equal)
{
  FAR struct tcb_s *rtcb = current_task(cpu);
  int sched_priority = rtcb->sched_priority;
  FAR struct tcb_s *btcb;
  bool ret = false;

  DEBUGASSERT(cpu == this_cpu());

  if (nxsched_islocked_tcb(rtcb))
    {
      return false;
    }

  if (switch_equal)
    {
      sched_priority--;
    }

  /* If there is a task in readytorun list, which is eglible to run on this
   * CPU, and has higher priority than the current task,
   * switch the current task to that one.
   */

  for (btcb = (FAR struct tcb_s *)dq_peek(list_readytorun());
       btcb && btcb->sched_priority > sched_priority;
       btcb = btcb->flink)
    {
      /* Check if the task found in ready-to-run list is allowed to run on
       * this CPU. TCB_FLAG_CPU_LOCKED may be used to override affinity. If
       * the flag is set, assume that btcb->cpu is valid, and it is the only
       * CPU on which the btcb can run.
       */

      if (CPU_ISSET(cpu, &btcb->affinity) &&
          ((btcb->flags & TCB_FLAG_CPU_LOCKED) == 0 || btcb->cpu == cpu))
        {
          /* Found a task, remove it from ready-to-run list */

          dq_rem((FAR struct dq_entry_s *)btcb, list_readytorun());

          if (!is_idle_task(rtcb))
            {
              /* Put currently running task back to ready-to-run list */

              rtcb->task_state = TSTATE_TASK_READYTORUN;
              nxsched_add_prioritized(rtcb, list_readytorun());
            }
          else
            {
              rtcb->task_state = TSTATE_TASK_ASSIGNED;
            }

          g_assignedtasks[cpu] = btcb;
          up_update_task(btcb);

          btcb->cpu = cpu;
          btcb->task_state = TSTATE_TASK_RUNNING;
          ret = true;
          break;
        }
    }

  return ret;
}

/****************************************************************************
 * Name:  nxsched_add_readytorun
 *
 * Description:
 *   This function adds a TCB to one of the ready to run lists. The list
 *   will be:
 *
 *   1. The g_readytorun list if the task is ready-to-run but not running
 *      and not assigned to a CPU.
 *   2. The g_assignedtask[cpu] list if the task is running or if has been
 *      assigned to a CPU.
 *
 *   If the currently active task has preemption disabled and the new TCB
 *   would cause this task to be preempted, the new task is added to the
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

bool nxsched_add_readytorun(FAR struct tcb_s *btcb)
{
  bool doswitch = false;
  int target_cpu = btcb->flags & TCB_FLAG_CPU_LOCKED ? btcb->cpu :
    nxsched_select_cpu(btcb->affinity);

  /* Add the btcb to the ready to run list, and try to run it on the target
   * CPU
   */

  btcb->task_state = TSTATE_TASK_READYTORUN;
  nxsched_add_prioritized(btcb, list_readytorun());

  if (target_cpu < CONFIG_SMP_NCPUS)
    {
      FAR struct tcb_s *tcb = current_task(target_cpu);

      if (tcb->sched_priority < btcb->sched_priority)
        {
          doswitch = nxsched_deliver_task(this_cpu(), target_cpu,
                                          SWITCH_HIGHER);
        }
    }

  return doswitch;
}

#endif /* CONFIG_SMP */
