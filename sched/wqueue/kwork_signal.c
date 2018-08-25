/****************************************************************************
 * sched/wqueue/work_signal.c
 *
 *   Copyright (C) 2014, 2016-2018 Gregory Nutt. All rights reserved.
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
#include <nuttx/signal.h>

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
 * Input Parameters:
 *   qid    - The work queue ID
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno value on failure
 *
 ****************************************************************************/

int work_signal(int qid)
{
  FAR struct kwork_wqueue_s *work;
  int threads;
  int i;

  /* Get the process ID of the worker thread */

#ifdef CONFIG_SCHED_HPWORK
  if (qid == HPWORK)
    {
      work = (FAR struct kwork_wqueue_s *)&g_hpwork;
      threads = CONFIG_SCHED_HPNTHREADS;
    }
  else
#endif
#ifdef CONFIG_SCHED_LPWORK
  if (qid == LPWORK)
    {
      work = (FAR struct kwork_wqueue_s *)&g_lpwork;
      threads = CONFIG_SCHED_LPNTHREADS;
    }
  else
#endif
    {
      return -EINVAL;
    }

  /* Find an IDLE worker thread */

  for (i = 0; i < threads; i++)
    {
      /* Is this worker thread busy? */

      if (!work->worker[i].busy)
        {
          /* No.. select this thread */

          break;
        }
    }

  /* If all of the IDLE threads are busy, then just return successfully */

  if (i >= threads)
    {
      return OK;
    }

  /* Otherwise, signal the first IDLE thread found */

  return nxsig_kill(work->worker[i].pid, SIGWORK);
}

#endif /* CONFIG_SCHED_WORKQUEUE */
