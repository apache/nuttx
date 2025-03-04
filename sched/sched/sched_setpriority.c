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
 * Private Type Declarations
 ****************************************************************************/

#ifdef CONFIG_SMP
struct reprioritize_arg_s
{
  pid_t pid;
  cpu_set_t saved_affinity;
  int  sched_priority;
  bool need_restore;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int reprioritize_handler(FAR void *cookie)
{
  FAR struct reprioritize_arg_s *arg = cookie;
  FAR struct tcb_s *rtcb = this_task();
  FAR struct tcb_s *tcb;
  irqstate_t flags;

  flags = enter_critical_section();

  tcb = nxsched_get_tcb(arg->pid);

  if (!tcb || tcb->task_state == TSTATE_TASK_INVALID ||
      (tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      leave_critical_section(flags);
      return OK;
    }

  if (arg->need_restore)
    {
      tcb->affinity = arg->saved_affinity;
      tcb->flags &= ~TCB_FLAG_CPU_LOCKED;
    }

  if (nxsched_reprioritize_rtr(tcb, arg->sched_priority))
    {
      up_switch_context(this_task(), rtcb);
    }

  leave_critical_section(flags);
  return OK;
}
#endif

/****************************************************************************
 * Name: nxsched_nexttcb
 *
 * Description:
 *   Get the next highest priority ready-to-run task.
 *
 * Input Parameters:
 *   tcb - the TCB of task to reprioritize.
 *
 * Returned Value:
 *   TCB of the next highest priority ready-to-run task.
 *
 ****************************************************************************/

#ifdef CONFIG_SMP
static FAR struct tcb_s *nxsched_nexttcb(FAR struct tcb_s *tcb)
{
  FAR struct tcb_s *nxttcb = tcb->flink;
  FAR struct tcb_s *rtrtcb;

  /* Which task should run next?  It will be either the next tcb in the
   * assigned task list (nxttcb) or a TCB in the g_readytorun list.  We can
   * only select a task from that list if the affinity mask includes the
   * tcb->cpu.
   *
   * If pre-emption is locked or another CPU is in a critical section,
   * then use the 'nxttcb' which will probably be the IDLE thread.
   */

  if (!nxsched_islocked_tcb(this_task()))
    {
      /* Search for the highest priority task that can run on tcb->cpu. */

      for (rtrtcb = (FAR struct tcb_s *)list_readytorun()->head;
           rtrtcb != NULL && !CPU_ISSET(tcb->cpu, &rtrtcb->affinity);
           rtrtcb = rtrtcb->flink);

      /* Return the TCB from the readyt-to-run list if it is the next
       * highest priority task.
       */

      if (rtrtcb != NULL &&
          rtrtcb->sched_priority >= nxttcb->sched_priority)
        {
          return rtrtcb;
        }
    }

  /* Otherwise, return the next TCB in the g_assignedtasks[] list...
   * probably the TCB of the IDLE thread.
   * REVISIT:  What if it is not the IDLE thread?
   */

  return nxttcb;
}
#endif

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
  nxttcb = nxsched_nexttcb(tcb);
#else
  nxttcb = tcb->flink;
#endif

  DEBUGASSERT(nxttcb != NULL);

  /* A context switch will occur if the new priority of the running
   * task becomes less than OR EQUAL TO the next highest priority
   * ready to run task.
   */

  if (sched_priority <= nxttcb->sched_priority)
    {
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

#ifdef CONFIG_SMP
              nxttcb = nxsched_nexttcb(tcb);
#else
              nxttcb = tcb->flink;
#endif
            }
          while (sched_priority < nxttcb->sched_priority);

          /* Change the task priority */

          tcb->sched_priority = (uint8_t)sched_priority;
        }
      else
        {
          /* A context switch will occur. */

#ifdef CONFIG_SMP
          if (tcb->cpu != this_cpu() &&
              tcb->task_state == TSTATE_TASK_RUNNING)
            {
              struct reprioritize_arg_s arg;

              if ((tcb->flags & TCB_FLAG_CPU_LOCKED) != 0)
                {
                  arg.pid = tcb->pid;
                  arg.need_restore = false;
                }
              else
                {
                  arg.pid = tcb->pid;
                  arg.saved_affinity = tcb->affinity;
                  arg.need_restore = true;

                  tcb->flags |= TCB_FLAG_CPU_LOCKED;
                  CPU_SET(tcb->cpu, &tcb->affinity);
                }

              arg.sched_priority = sched_priority;
              nxsched_smp_call_single(tcb->cpu, reprioritize_handler, &arg);
            }
          else
#endif
          if (nxsched_reprioritize_rtr(tcb, sched_priority))
            {
              up_switch_context(this_task(), rtcb);
            }
        }
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

  /* A context switch will occur. */

  if (nxsched_reprioritize_rtr(tcb, sched_priority))
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
#ifdef CONFIG_SMP
      case TSTATE_TASK_ASSIGNED:
#endif
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
