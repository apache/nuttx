/****************************************************************************
 * sched/sched/sched_getscheduler.c
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
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_get_scheduler
 *
 * Description:
 *   sched_getscheduler() returns the scheduling policy currently
 *   applied to the task identified by pid.  If pid equals zero, the
 *   policy of the calling task will be retrieved.
 *
 *   This functions is identical to the function sched_getscheduler(),
 *   differing only in its return value:  This function does not modify
 *   the errno variable.
 *
 *   This is a non-standard, internal OS function and is not intended for
 *   use by application logic.  Applications should use the standard
 *   sched_getscheduler().
 *
 * Input Parameters:
 *   pid - the task ID of the task to query.  If pid is zero, the
 *     calling task is queried.
 *
 * Returned Value:
 *    On success, sched_getscheduler() returns the policy for the task
 *    (either SCHED_FIFO or SCHED_RR).  On error,  a negated errno value
 *    returned:
 *
 *      ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int nxsched_get_scheduler(pid_t pid)
{
  FAR struct tcb_s *tcb;
  int policy;

  /* Verify that the PID corresponds to a real task */

  if (pid == 0)
    {
      tcb = this_task();
    }
  else
    {
      tcb = nxsched_get_tcb(pid);
    }

  if (tcb == NULL)
    {
      return -ESRCH;
    }

  /* Return the scheduling policy from the TCB.  NOTE that the user-
   * interpretable values are 1 based; the TCB values are zero-based.
   */

  policy = (tcb->flags & TCB_FLAG_POLICY_MASK) >> TCB_FLAG_POLICY_SHIFT;
  return policy + 1;
}

/****************************************************************************
 * Name: sched_getscheduler
 *
 * Description:
 *   sched_getscheduler() returns the scheduling policy currently
 *   applied to the task identified by pid. If pid equals zero, the
 *   policy of the calling task will be retrieved.
 *
 *   sched_getscheduler() is a simply wrapper around nxsched_get_scheduler()
 *   that sets the errno value in the event of an error.
 *
 * Input Parameters:
 *   pid - the task ID of the task to query.  If pid is zero, the
 *     calling task is queried.
 *
 * Returned Value:
 *    On success, sched_getscheduler() returns the policy for the task
 *    (either SCHED_FIFO or SCHED_RR).  On error,  ERROR (-1) is
 *    returned, and errno is set appropriately:
 *
 *      ESRCH  The task whose ID is pid could not be found.
 *
 ****************************************************************************/

int sched_getscheduler(pid_t pid)
{
  int ret = nxsched_get_scheduler(pid);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
