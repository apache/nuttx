/****************************************************************************
 * sched/wqueue/work_signal.c
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
