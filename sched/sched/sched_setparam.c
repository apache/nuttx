/****************************************************************************
 * sched/sched/sched_setparam.c
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

#include <sys/types.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "clock/clock.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_set_param
 *
 * Description:
 *   This function sets the priority of a specified task.  It is identical
 *   to the function sched_setparam(), differing only in its return value:
 *   This function does not modify the errno variable.
 *
 *   NOTE: Setting a task's priority to the same value has a similar effect
 *   to sched_yield() -- The task will be moved to  after all other tasks
 *   with the same priority.
 *
 *   This is a non-standard, internal OS function and is not intended for
 *   use by application logic.  Applications should use the standard
 *   sched_setparam().
 *
 * Input Parameters:
 *   pid - the task ID of the task to reprioritize.  If pid is zero, the
 *      priority of the calling task is changed.
 *   param - A structure whose member sched_priority is the integer priority.
 *      The range of valid priority numbers is from SCHED_PRIORITY_MIN
 *      through SCHED_PRIORITY_MAX.
 *
 * Returned Value:
 *   0 (OK) if successful, otherwise a negated errno value is returned to
 *   indicate the nature of the failure..
 *
 *   EINVAL The parameter 'param' is invalid or does not make sense for the
 *          current scheduling policy.
 *   EPERM  The calling task does not have appropriate privileges.
 *   ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int nxsched_set_param(pid_t pid, FAR const struct sched_param *param)
{
  FAR struct tcb_s *rtcb;
  FAR struct tcb_s *tcb;
  int ret;

  /* Verify that the requested priority is in the valid range */

  if (param == NULL)
    {
      return -EINVAL;
    }

  /* Prohibit modifications to the head of the ready-to-run task
   * list while adjusting the priority
   */

  sched_lock();

  /* Check if the task to reprioritize is the calling task */

  rtcb = this_task();
  if (pid == 0 || pid == rtcb->pid)
    {
      tcb = rtcb;
    }

  /* The PID is not the calling task, we will have to search for it */

  else
    {
      tcb = nxsched_get_tcb(pid);
      if (!tcb)
        {
          /* No task with this PID was found */

          ret = -ESRCH;
          goto errout_with_lock;
        }
    }

#ifdef CONFIG_SCHED_SPORADIC
  /* Update parameters associated with SCHED_SPORADIC */

  if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      FAR struct sporadic_s *sporadic;
      irqstate_t flags;
      sclock_t repl_ticks;
      sclock_t budget_ticks;

      if (param->sched_ss_max_repl < 1 ||
          param->sched_ss_max_repl > CONFIG_SCHED_SPORADIC_MAXREPL)
        {
          ret = -EINVAL;
          goto errout_with_lock;
        }

      /* Convert timespec values to system clock ticks */

      clock_time2ticks(&param->sched_ss_repl_period, &repl_ticks);
      clock_time2ticks(&param->sched_ss_init_budget, &budget_ticks);

      /* Avoid zero/negative times */

      if (repl_ticks < 1)
        {
          repl_ticks = 1;
        }

      if (budget_ticks < 1)
        {
          budget_ticks = 1;
        }

      /* The replenishment period must be greater than or equal to the
       * budget period.
       */

#if 1
      /* REVISIT: In the current implementation, the budget cannot exceed
       * half the duty.
       */

      if (repl_ticks < (2 * budget_ticks))
#else
      if (repl_ticks < budget_ticks)
#endif
        {
          ret = -EINVAL;
          goto errout_with_lock;
        }

      /* Stop/reset current sporadic scheduling */

      flags = enter_critical_section();
      ret = nxsched_reset_sporadic(tcb);
      if (ret >= 0)
        {
          /* Save the sporadic scheduling parameters and reset to the
           * beginning to the replenishment interval.
           */

          tcb->timeslice         = budget_ticks;

          sporadic = rtcb->sporadic;
          DEBUGASSERT(sporadic != NULL);

          sporadic->hi_priority  = param->sched_priority;
          sporadic->low_priority = param->sched_ss_low_priority;
          sporadic->max_repl     = param->sched_ss_max_repl;
          sporadic->repl_period  = repl_ticks;
          sporadic->budget       = budget_ticks;

          /* And restart at the next replenishment interval */

          ret = nxsched_start_sporadic(tcb);
        }

      /* Restore interrupts and handler errors */

      leave_critical_section(flags);
      if (ret < 0)
        {
          goto errout_with_lock;
        }
    }
#endif

  /* Then perform the reprioritization */

  ret = nxsched_reprioritize(tcb, param->sched_priority);

errout_with_lock:
  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name:  sched_setparam
 *
 * Description:
 *   This function sets the priority of a specified task.  This function is
 *   a simply wrapper around nxsched_set_param() that sets the errno value in
 *   the event of an error.
 *
 *   NOTE: Setting a task's priority to the same value has a similar effect
 *   to sched_yield() -- The task will be moved to  after all other tasks
 *   with the same priority.
 *
 * Input Parameters:
 *   pid - the task ID of the task to reprioritize.  If pid is zero, the
 *      priority of the calling task is changed.
 *   param - A structure whose member sched_priority is the integer priority.
 *      The range of valid priority numbers is from SCHED_PRIORITY_MIN
 *      through SCHED_PRIORITY_MAX.
 *
 * Returned Value:
 *   On success, sched_setparam() returns 0 (OK). On error, -1 (ERROR) is
 *   returned, and errno is set appropriately.
 *
 *  EINVAL The parameter 'param' is invalid or does not make sense for the
 *         current scheduling policy.
 *  EPERM  The calling task does not have appropriate privileges.
 *  ESRCH  The task whose ID is pid could not be found.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_setparam(pid_t pid, FAR const struct sched_param *param)
{
  int ret = nxsched_set_param(pid, param);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
