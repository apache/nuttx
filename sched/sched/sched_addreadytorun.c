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
 *   btcb - Points to the TCB that is ready-to-run
 *   cpu  - Always this_cpu(). Given as argument only for optimization
 *
 * Returned Value:
 *   true if the currently active task is switched to the btcb
 *
 * Assumptions:
 * - The caller has established a critical section
 * - The caller has already removed the input rtcb from whatever list it
 *   was in.
 * - The caller handles the condition that occurs if the head of the
 *   assigned tasks list has changed.
 *
 ****************************************************************************/

bool nxsched_switch_running(FAR struct tcb_s *btcb, int cpu)
{
  FAR struct tcb_s *rtcb = current_task(cpu);
  bool ret = true;

  DEBUGASSERT(cpu == this_cpu());
  DEBUGASSERT(btcb->task_state != TSTATE_TASK_RUNNING);

  /* If the currently running task is locked, or is on higher priority than
   * the requested one. Just add the btcb into ready-to-run list (not
   * running) and return false
   */

  if (nxsched_islocked_tcb(rtcb) ||
      btcb->sched_priority <= rtcb->sched_priority ||
      ((1 << cpu) & btcb->affinity) == 0)
    {
      nxsched_add_prioritized(btcb, list_readytorun());
      btcb->task_state = TSTATE_TASK_READYTORUN;
      ret = false;
    }
  else
    {
      FAR dq_queue_t *tasklist = list_assignedtasks(cpu);

      /* Only the idle task and one other are kept in g_assignedtasks.
       * return the other tasks to unassigned list, so that they can be
       * picked up by other CPUs
       */

      if (!is_idle_task(rtcb))
        {
          rtcb->task_state = TSTATE_TASK_READYTORUN;
          dq_remfirst(tasklist);
          nxsched_add_prioritized(rtcb, list_readytorun());

          /* We should now have only the idle task assigned */

          DEBUGASSERT(is_idle_task((FAR struct tcb_s *)dq_peek(tasklist)));
        }
      else
        {
          rtcb->task_state = TSTATE_TASK_ASSIGNED;
        }

      dq_addfirst((FAR dq_entry_t *)btcb, tasklist);
      up_update_task(btcb);

      btcb->cpu = cpu;
      btcb->task_state = TSTATE_TASK_RUNNING;
    }

  return ret;
}

/****************************************************************************
 * Name:  nxsched_add_readytorun/nxsched_add_readytorun_cpu
 *
 * Description:
 *   This function adds a TCB to one of the ready to run lists. The _cpu
 *   variant adds the TCB to the specified CPU's list. The list will be:
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
  /* Get the task currently running on the CPU (may be the IDLE task) */

  int cpu = nxsched_select_cpu(btcb->affinity);
  return nxsched_add_readytorun_cpu(btcb, cpu);
}

bool nxsched_add_readytorun_cpu(FAR struct tcb_s *btcb,
                                int cpu)
{
  bool doswitch;

  /* Determine the desired new task state.  First, if the new task priority
   * is higher then the priority of the lowest priority, running task, then
   * the new task will be running and a context switch switch will be
   * required.
   */

  if (cpu >= CONFIG_SMP_NCPUS ||
      btcb->sched_priority <= current_delivered(cpu)->sched_priority)
    {
      /* There is no CPU available to run the task, or the priority is
       * lower or equal to the currently running task.
       * The btcb won't be running, add it to the ready-to-run task list
       */

      nxsched_add_prioritized(btcb, list_readytorun());

      btcb->task_state = TSTATE_TASK_READYTORUN;
      doswitch         = false;
    }
  else
    {
      /* The btcb needs to run */

      if (cpu != this_cpu())
        {
          /* The btcb is running on another CPU, request that CPU to switch */

          if (g_delivertasks[cpu] == NULL)
            {
              g_delivertasks[cpu] = btcb;
              btcb->cpu = cpu;
              btcb->task_state = TSTATE_TASK_INVALID;
              up_send_smp_sched(cpu);
            }
          else
            {
              FAR struct tcb_s *rtcb = g_delivertasks[cpu];
              if (rtcb->sched_priority < btcb->sched_priority)
                {
                  g_delivertasks[cpu] = btcb;
                  btcb->cpu = cpu;
                  btcb->task_state = TSTATE_TASK_INVALID;
                  nxsched_add_prioritized(rtcb, &g_readytorun);
                  rtcb->task_state = TSTATE_TASK_READYTORUN;
                }
              else
                {
                  nxsched_add_prioritized(btcb, &g_readytorun);
                  btcb->task_state = TSTATE_TASK_READYTORUN;
                }
            }

          doswitch = false;
        }
      else
        {
          /* Running on this CPU - try to switch now */

          doswitch = nxsched_switch_running(btcb, cpu);
        }
    }

  return doswitch;
}

#endif /* CONFIG_SMP */
