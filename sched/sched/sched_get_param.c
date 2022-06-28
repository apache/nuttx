/****************************************************************************
 * sched/sched/sched_get_param.c
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

#include "clock/clock.h"
#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_param
 *
 * Description:
 *   This function gets the scheduling priority of the task specified by
 *   pid.  It is identical to the function sched_getparam(), differing only
 *   in its return value:  This function does not modify the errno variable.
 *
 *   This is a non-standard, internal OS function and is not intended for
 *   use by application logic.  Applications should use the standard
 *   sched_getparam().
 *
 * Input Parameters:
 *   pid - the task ID of the task.  If pid is zero, the priority
 *     of the calling task is returned.
 *   param - A structure whose member sched_priority is the integer
 *     priority.  The task's priority is copied to the sched_priority
 *     element of this structure.
 *
 * Returned Value:
 *   0 (OK) if successful, otherwise a negated errno value is returned to
 *   indicate the nature of the failure..
 *
 *   This function can fail if param is null (EINVAL) or if pid does
 *   not correspond to any task (ESRCH).
 *
 ****************************************************************************/

int nxsched_get_param(pid_t pid, FAR struct sched_param *param)
{
  FAR struct tcb_s *rtcb;
  FAR struct tcb_s *tcb;
  int ret = OK;

  if (param == NULL)
    {
      return -EINVAL;
    }

  /* Check if the task to restart is the calling task */

  rtcb = this_task();
  if (pid == 0 || pid == rtcb->pid)
    {
      /* Return the priority if the calling task. */

      param->sched_priority = (int)rtcb->sched_priority;
    }

  /* This PID is not for the calling task, we will have to look it up */

  else
    {
      /* Get the TCB associated with this PID */

      sched_lock();
      tcb = nxsched_get_tcb(pid);
      if (!tcb)
        {
          /* This PID does not correspond to any known task */

          ret = -ESRCH;
        }
      else
        {
#ifdef CONFIG_SCHED_SPORADIC
#endif
          /* Return the priority of the task */

          param->sched_priority = (int)tcb->sched_priority;

#ifdef CONFIG_SCHED_SPORADIC
          if ((tcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
            {
              FAR struct sporadic_s *sporadic = tcb->sporadic;
              DEBUGASSERT(sporadic != NULL);

              /* Return parameters associated with SCHED_SPORADIC */

              param->sched_ss_low_priority = (int)sporadic->low_priority;
              param->sched_ss_max_repl     = (int)sporadic->max_repl;

              clock_ticks2time((sclock_t)sporadic->repl_period,
                               &param->sched_ss_repl_period);
              clock_ticks2time((sclock_t)sporadic->budget,
                               &param->sched_ss_init_budget);
            }
          else
            {
              param->sched_ss_low_priority        = 0;
              param->sched_ss_max_repl            = 0;
              param->sched_ss_repl_period.tv_sec  = 0;
              param->sched_ss_repl_period.tv_nsec = 0;
              param->sched_ss_init_budget.tv_sec  = 0;
              param->sched_ss_init_budget.tv_nsec = 0;
            }
#endif
        }

      sched_unlock();
    }

  return ret;
}
