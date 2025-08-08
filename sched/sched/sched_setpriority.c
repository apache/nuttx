/****************************************************************************
 * sched/sched/sched_setpriority.c
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

#include <stdint.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "irq/irq.h"
#include "sched/sched.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_running_setpriority
 *
 * Description:
 *   This function sets the priority of a running task.  This does nothing
 *   if we are increasing the priority of a running task.  If we are dropping
 *   the priority of a running task, then this could cause then next lower
 *   priority task to run,
 *
 *   NOTE: Setting a task's priority to the same value has a similar effect
 *   to sched_yield() -- The task will be moved to after all other tasks
 *   with the same priority.
 *
 * Input Parameters:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void nxsched_running_setpriority(FAR struct tcb_s *tcb,
                                               int sched_priority)
{
  FAR struct tcb_s *nxttcb;

  /* Get the TCB of the next highest priority, ready to run task */

#ifdef CONFIG_SMP
  nxttcb = (FAR struct tcb_s *)dq_peek(list_readytorun());
#else
  nxttcb = tcb->flink;
#endif

  /* A context switch will occur if the new priority of the running
   * task becomes less than OR EQUAL TO the next highest priority
   * ready to run task.
   */

  if (nxttcb && sched_priority <= nxttcb->sched_priority)
    {
#ifdef CONFIG_SMP
      tcb->sched_priority = (uint8_t)sched_priority;
      if (nxsched_deliver_task(this_cpu(), tcb->cpu, SWITCH_EQUAL))
        {
          up_switch_context(this_task(), tcb);
        }
#else
      FAR struct tcb_s *rtcb = this_task();

      if (nxsched_islocked_tcb(rtcb))
        {
          /* Move all tasks with the higher priority from the ready-to-run
           * list to the pending list.
           */

          do
            {
              bool check = nxsched_remove_readytorun(nxttcb);
              DEBUGASSERT(check == false);
              UNUSED(check);

              nxsched_add_prioritized(nxttcb, list_pendingtasks());
              nxttcb->task_state = TSTATE_TASK_PENDING;

              nxttcb = tcb->flink;
            }
          while (sched_priority < nxttcb->sched_priority);

          /* Change the task priority */

          tcb->sched_priority = (uint8_t)sched_priority;
        }
      else
        {
          /* A context switch will occur. */

          if (nxsched_reprioritize_rtr(tcb, sched_priority))
            {
              up_switch_context(this_task(), rtcb);
            }
        }
#endif
    }

  /* Otherwise, we can just change priority since it has no effect */

  else
    {
      /* Change the task priority */

      tcb->sched_priority = (uint8_t)sched_priority;
    }
}

/****************************************************************************
 * Name:  nxsched_readytorun_setpriority
 *
 * Description:
 *   This function sets the priority of a ready-to-run task.  This may alter
 *   the position of the task in the ready-to-run list and if the priority
 *   is increased, may cause the task to become running.
 *
 * Input Parameters:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxsched_readytorun_setpriority(FAR struct tcb_s *tcb,
                                           int sched_priority)
{
  FAR struct tcb_s *rtcb;

  rtcb = this_task();

#ifdef CONFIG_SMP
  dq_rem((FAR struct dq_entry_s *)tcb, list_readytorun());
  tcb->sched_priority = sched_priority;
  if (nxsched_add_readytorun(tcb))
#else
  if (nxsched_reprioritize_rtr(tcb, sched_priority))
#endif
    {
      up_switch_context(this_task(), rtcb);
    }
}

/****************************************************************************
 * Name:  nxsched_blocked_setpriority
 *
 * Description:
 *   Change the priority of a blocked tasks.  The only issue here is that
 *   the task may like in a prioritized or an non-prioritized queue.
 *
 * Input Parameters:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void nxsched_blocked_setpriority(FAR struct tcb_s *tcb,
                                               int sched_priority)
{
  FAR dq_queue_t *tasklist;
  tstate_t task_state = (tstate_t)tcb->task_state;

  /* CASE 3a. The task resides in a prioritized list. */

  tasklist = TLIST_BLOCKED(tcb);
  if (TLIST_ISPRIORITIZED(task_state))
    {
      /* Remove the TCB from the prioritized task list */

      dq_rem((FAR dq_entry_t *)tcb, tasklist);

      /* Change the task priority */

      tcb->sched_priority = (uint8_t)sched_priority;

      /* Put it back into the prioritized list at the correct position. */

      nxsched_add_prioritized(tcb, tasklist);
    }

  /* CASE 3b. The task resides in a non-prioritized list. */

  else
    {
      /* Just change the task's priority */

      tcb->sched_priority = (uint8_t)sched_priority;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_set_priority
 *
 * Description:
 *   This function sets the priority of a specified task.
 *
 *   NOTE: Setting a task's priority to the same value has a similar effect
 *   to sched_yield() -- The task will be moved to after all other tasks
 *   with the same priority.
 *
 * Input Parameters:
 *   tcb - the TCB of task to reprioritize.
 *   sched_priority - The new task priority
 *
 * Returned Value:
 *   On success, nxsched_set_priority() returns 0 (OK). On error, a negated
 *   errno value is returned.
 *
 *  EINVAL The parameter 'param' is invalid or does not make sense for the
 *         current scheduling policy.
 *  EPERM  The calling task does not have appropriate privileges.
 *  ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int nxsched_set_priority(FAR struct tcb_s *tcb, int sched_priority)
{
  irqstate_t flags;

  /* Verify that the requested priority is in the valid range */

  if (sched_priority < SCHED_PRIORITY_MIN ||
      sched_priority > SCHED_PRIORITY_MAX)
    {
      return -EINVAL;
    }

  /* We need to assure that there there is no interrupt activity while
   * performing the following.
   */

  flags = enter_critical_section();

  /* There are three major cases (and two sub-cases) that must be
   * considered:
   */

  switch (tcb->task_state)
    {
      /* CASE 1. The task is running and a context switch may be caused by
       * the re-prioritization
       */

      case TSTATE_TASK_RUNNING:
        nxsched_running_setpriority(tcb, sched_priority);
        break;

      /* CASE 2. The task is ready-to-run (but not running) and a context
       * switch may be caused by the re-prioritization
       */

      case TSTATE_TASK_READYTORUN:
        nxsched_readytorun_setpriority(tcb, sched_priority);
        break;

      /* CASE 3. The task is not in the ready to run list.  Changing its
       * Priority cannot effect the currently executing task.
       */

      default:
        nxsched_blocked_setpriority(tcb, sched_priority);
        break;
    }

  leave_critical_section(flags);
  return OK;
}
