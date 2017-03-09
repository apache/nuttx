/****************************************************************************
 * sched/wqueue/work_signal.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
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

#include <signal.h>
#include <errno.h>

#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_signal
 *
 * Description:
 *   Signal the worker thread to process the work queue now.  This function
 *   is used internally by the work logic but could also be used by the
 *   user to force an immediate re-assessment of pending work.
 *
 * Input parameters:
 *   qid    - The work queue ID
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

int work_signal(int qid)
{
  pid_t pid;
  int ret;

  /* Get the process ID of the worker thread */

#ifdef CONFIG_SCHED_HPWORK
  if (qid == HPWORK)
    {
      pid = g_hpwork.worker[0].pid;
    }
  else
#endif
#ifdef CONFIG_SCHED_LPWORK
  if (qid == LPWORK)
    {
      int i;

      /* Find an IDLE worker thread */

      for (i = 0; i < CONFIG_SCHED_LPNTHREADS; i++)
        {
          /* Is this worker thread busy? */

          if (!g_lpwork.worker[i].busy)
            {
              /* No.. select this thread */

              break;
            }
        }

      /* If all of the IDLE threads are busy, then just return successfully */

      if (i >= CONFIG_SCHED_LPNTHREADS)
        {
          return OK;
        }

      /* Otherwise, signal the first IDLE thread found */

      pid = g_lpwork.worker[i].pid;
    }
  else
#endif
    {
      return -EINVAL;
    }

  /* Signal the worker thread */

  ret = kill(pid, SIGWORK);
  if (ret < 0)
    {
      int errcode = errno;
      return -errcode;
    }

  return OK;
}

#endif /* CONFIG_SCHED_WORKQUEUE */
