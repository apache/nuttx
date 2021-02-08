/****************************************************************************
 * sched/sched/sched_rrgetinterval.c
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

#include <nuttx/arch.h>

#include "sched/sched.h"
#include "clock/clock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_rr_get_interval
 *
 * Description:
 *   sched_rr_get_interval()  writes  the timeslice interval for task
 *   identified by 'pid' into  the timespec structure pointed to by
 *   'interval.'  If pid is zero, the timeslice for the calling process
 *   is written into 'interval.  The identified process should be running
 *   under the SCHED_RRscheduling policy.'
 *
 * Input Parameters:
 *   pid - the task ID of the task.  If pid is zero, the priority of the
 *         calling task is returned.
 *   interval - a structure used to return the time slice
 *
 * Returned Value:
 *   On success, sched_rr_get_interval() returns OK (0).  On error,
 *   ERROR (-1) is returned, and errno is set to:
 *
 *   EFAULT -- Cannot copy to interval
 *   EINVAL Invalid pid.
 *   ENOSYS The system call is not yet implemented.
 *   ESRCH  The process whose ID is pid could not be found.
 *
 * Assumptions:
 *
 ****************************************************************************/

int sched_rr_get_interval(pid_t pid, struct timespec *interval)
{
  FAR struct tcb_s *rrtcb;

  /* If pid is zero, the timeslice for the calling process is written
   * into 'interval.'
   */

  if (pid == 0)
    {
      rrtcb = this_task();
    }

  /* Return a special error code on invalid PID */

  else if (pid < 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  /* Otherwise, lookup the TCB associated with this PID */

  else
    {
      rrtcb = nxsched_get_tcb(pid);
      if (rrtcb == NULL)
        {
          set_errno(ESRCH);
          return ERROR;
        }
    }

  if (interval == NULL)
    {
      set_errno(EFAULT);
      return ERROR;
    }

#if CONFIG_RR_INTERVAL > 0
  /* The thread has a timeslice ONLY if it is configured for round-robin
   * scheduling.
   */

  if ((rrtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_RR)
    {
      /* Convert the timeslice value from ticks to a timespec */

      interval->tv_sec  =  CONFIG_RR_INTERVAL / MSEC_PER_SEC;
      interval->tv_nsec = (CONFIG_RR_INTERVAL % MSEC_PER_SEC) *
                          NSEC_PER_MSEC;
    }
  else
#endif
    {
      /* Return {0,0} meaning that the time slice is indefinite */

      interval->tv_sec  = 0;
      interval->tv_nsec = 0;
    }

  return OK;
}
