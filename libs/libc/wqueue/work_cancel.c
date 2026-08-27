/****************************************************************************
 * libs/libc/wqueue/work_cancel.c
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

#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#if defined(CONFIG_LIBC_USRWORK) && !defined(__KERNEL__)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_qcancel
 *
 * Description:
 *   Cancel previously queued work.  This removes work from the work queue.
 *   After work has been cancelled, it may be requeued by calling
 *   work_queue() again.
 *
 * Input Parameters:
 *   wqueue - The work queue
 *   work   - The previously queued work structure to cancel
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno on failure.  This error may be
 *   reported:
 *
 *   -EINVAL - An invalid work queue was specified
 *
 ****************************************************************************/

static int work_qcancel(FAR struct usr_wqueue_s *wqueue, bool sync,
                        FAR struct work_s *work)
{
  pid_t self = sync ? gettid() : 0;
  int ret;

  if (wqueue == NULL || work == NULL)
    {
      return -EINVAL;
    }

  /* A work structure becomes available for requeue after it is dequeued,
   * before its callback returns.  Multiple workers can therefore execute
   * callbacks using the same work structure.  Find one such worker and
   * repeat after it finishes until no callback remains.  Exclude the
   * calling worker to avoid self-deadlock.
   */

  for (; ; )
    {
      FAR sem_t *sync_wait = NULL;
      int wndx;

      /* Get exclusive access to the work queue */

      do
        {
          ret = nxmutex_lock(&wqueue->lock);
        }
      while (ret < 0);

      /* Remove a pending instance from the queue. */

      if (work->worker != NULL)
        {
          if (work_remove(wqueue, work))
            {
              work_wake(wqueue);
            }
        }

      if (sync)
        {
          for (wndx = 0; wndx < wqueue->nthreads; wndx++)
            {
              FAR struct usr_worker_s *worker = &wqueue->worker[wndx];

              if (worker->work == work && self != worker->tid)
                {
                  worker->wait_count++;
                  sync_wait = &worker->wait;
                  break;
                }
            }
        }

      nxmutex_unlock(&wqueue->lock);

      if (sync_wait == NULL)
        {
          return OK;
        }

      nxsem_wait_uninterruptible(sync_wait);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_cancel
 *
 * Description:
 *   Cancel previously queued user-mode work.  This removes work from the
 *   user mode work queue.  After work has been cancelled, it may be
 *   requeued by calling work_queue() again.
 *
 * Input Parameters:
 *   qid    - The work queue ID (must be USRWORK)
 *   work   - The previously queued work structure to cancel
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno on failure.  This error may be
 *   reported:
 *
 *   -EINVAL - An invalid work queue was specified
 *
 ****************************************************************************/

int work_cancel(int qid, FAR struct work_s *work)
{
  if (qid == USRWORK)
    {
      return work_qcancel(&g_usrwork, false, work);
    }
  else
    {
      return -EINVAL;
    }
}

#ifndef CONFIG_DISABLE_PTHREAD
int work_cancel_wq(FAR struct kwork_wqueue_s *handle,
                   FAR struct work_s *work)
{
  return work_qcancel((FAR struct usr_wqueue_s *)handle, false, work);
}
#endif

int work_cancel_sync(int qid, FAR struct work_s *work)
{
  if (qid == USRWORK)
    {
      return work_qcancel(&g_usrwork, true, work);
    }

  return -EINVAL;
}

#ifndef CONFIG_DISABLE_PTHREAD
int work_cancel_sync_wq(FAR struct kwork_wqueue_s *handle,
                        FAR struct work_s *work)
{
  return work_qcancel((FAR struct usr_wqueue_s *)handle, true, work);
}
#endif

#endif /* CONFIG_LIBC_USRWORK && !__KERNEL__ */
