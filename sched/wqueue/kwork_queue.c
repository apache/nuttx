/****************************************************************************
 * sched/wqueue/kwork_queue.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/queue.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define queue_work(wqueue, work) \
  do \
    { \
      dq_addlast((FAR dq_entry_t *)(work), &(wqueue)->q); \
      if ((wqueue)->wait_count > 0) /* There are threads waiting for sem. */ \
        { \
          (wqueue)->wait_count--; \
          nxsem_post(&(wqueue)->sem); \
        } \
    } \
  while (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_timer_expiry
 ****************************************************************************/

static void work_timer_expiry(wdparm_t arg)
{
  FAR struct work_s *work = (FAR struct work_s *)arg;

  irqstate_t flags = spin_lock_irqsave(&work->wq->lock);
  sched_lock();

  /* We have being canceled */

  if (work->worker != NULL)
    {
      queue_work(work->wq, work);
    }

  spin_unlock_irqrestore(&work->wq->lock, flags);
  sched_unlock();
}

static bool work_is_canceling(FAR struct kworker_s *kworkers, int nthreads,
                              FAR struct work_s *work)
{
  int wndx;

  for (wndx = 0; wndx < nthreads; wndx++)
    {
      if (kworkers[wndx].work == work)
        {
          if (kworkers[wndx].wait_count > 0)
            {
              return true;
            }
        }
    }

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_queue/work_queue_wq
 *
 * Description:
 *   Queue work to be performed at a later time.  All queued work will be
 *   performed on the worker thread of execution (not the caller's).
 *
 *   The work structure is allocated and must be initialized to all zero by
 *   the caller.  Otherwise, the work structure is completely managed by the
 *   work queue logic.  The caller should never modify the contents of the
 *   work queue structure directly.  If work_queue() is called before the
 *   previous work has been performed and removed from the queue, then any
 *   pending work will be canceled and lost.
 *
 * Input Parameters:
 *   qid    - The work queue ID (must be HPWORK or LPWORK)
 *   wqueue - The work queue handle
 *   work   - The work structure to queue
 *   worker - The worker callback to be invoked.  The callback will be
 *            invoked on the worker thread of execution.
 *   arg    - The argument that will be passed to the worker callback when
 *            it is invoked.
 *   delay  - Delay (in clock ticks) from the time queue until the worker
 *            is invoked. Zero means to perform the work immediately.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

int work_queue_wq(FAR struct kwork_wqueue_s *wqueue,
                  FAR struct work_s *work, worker_t worker,
                  FAR void *arg, clock_t delay)
{
  irqstate_t flags;

  if (wqueue == NULL || work == NULL || worker == NULL)
    {
      return -EINVAL;
    }

  /* Interrupts are disabled so that this logic can be called from with
   * task logic or from interrupt handling logic.
   */

  flags = spin_lock_irqsave(&wqueue->lock);

  /* Remove the entry from the timer and work queue. */

  if (work->worker != NULL)
    {
      /* Remove the entry from the work queue and make sure that it is
       * marked as available (i.e., the worker field is nullified).
       */

      work->worker = NULL;
      wd_cancel(&work->u.timer);
      if (dq_inqueue((FAR dq_entry_t *)work, &wqueue->q))
        {
          dq_rem((FAR dq_entry_t *)work, &wqueue->q);
        }
    }

  if (work_is_canceling(wqueue->worker, wqueue->nthreads, work))
    {
      spin_unlock_irqrestore(&wqueue->lock, flags);
      return 0;
    }

  /* Initialize the work structure. */

  work->worker = worker;           /* Work callback. non-NULL means queued */
  work->arg    = arg;              /* Callback argument */
  work->wq     = wqueue;           /* Work queue */

  /* Queue the new work */

  if (!delay)
    {
      sched_lock();
      queue_work(wqueue, work);
      spin_unlock_irqrestore(&wqueue->lock, flags);
      sched_unlock();
    }
  else
    {
      wd_start(&work->u.timer, delay, work_timer_expiry, (wdparm_t)work);
      spin_unlock_irqrestore(&wqueue->lock, flags);
    }

  return 0;
}

int work_queue(int qid, FAR struct work_s *work, worker_t worker,
               FAR void *arg, clock_t delay)
{
  return work_queue_wq(work_qid2wq(qid), work, worker, arg, delay);
}

#endif /* CONFIG_SCHED_WORKQUEUE */
