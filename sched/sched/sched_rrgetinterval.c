/****************************************************************************
 * sched/sched/sched_rrgetinterval.c
 *
 *   Copyright (C) 2007, 2009, 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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
