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
#include <nuttx/list.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_queue_next/work_queue_next_wq
 *
 * Description:
 *   Queue work to be performed at a later time based on the last expiration
 *   time. This function must be called in the workqueue callback.
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

int work_queue_next_wq(FAR struct kwork_wqueue_s *wqueue,
                       FAR struct work_s *work, worker_t worker,
                       FAR void *arg, clock_t delay)
{
  irqstate_t flags;

  if (wqueue == NULL || work == NULL || worker == NULL ||
      delay > WDOG_MAX_DELAY)
    {
      return -EINVAL;
    }

  /* Initialize the work structure. */

  work->worker = worker; /* Work callback. non-NULL means queued */
  work->arg    = arg;    /* Callback argument */
  work->qtime += delay;  /* Expected time based on last expiration time */

  flags = spin_lock_irqsave(&wqueue->lock);

  if (delay)
    {
      /* Insert to the pending list of the wqueue. */

      if (work_insert_pending(wqueue, work))
        {
          /* Start the timer if the work is the earliest expired work. */

          wd_start_abstick(&wqueue->timer, work->qtime,
                           work_timer_expired, (wdparm_t)wqueue);
        }
    }
  else
    {
      /* Insert to the expired list of the wqueue. */

      list_add_tail(&wqueue->expired, &work->node);
    }

  spin_unlock_irqrestore(&wqueue->lock, flags);

  if (!delay)
    {
      /* Immediately wake up the worker thread. */

      nxsem_post(&wqueue->sem);
    }

  return 0;
}

int work_queue_next(int qid, FAR struct work_s *work, worker_t worker,
                    FAR void *arg, clock_t delay)
{
  return work_queue_next_wq(work_qid2wq(qid), work, worker, arg, delay);
}

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
  clock_t expected;
  bool retimer;

  if (wqueue == NULL || work == NULL || worker == NULL ||
      delay > WDOG_MAX_DELAY)
    {
      return -EINVAL;
    }

  expected = clock_delay2abstick(delay);

  /* Interrupts are disabled so that this logic can be called from with
   * task logic or from interrupt handling logic.
   */

  flags = spin_lock_irqsave(&wqueue->lock);

  /* Ensure the work has been removed. */

  retimer = work_available(work) ? false : work_remove(wqueue, work);

  /* Initialize the work structure. */

  work->worker = worker;   /* Work callback. non-NULL means queued */
  work->arg    = arg;      /* Callback argument */
  work->qtime  = expected; /* Expected time */

  if (delay)
    {
      /* Insert to the pending list of the wqueue. */

      if (work_insert_pending(wqueue, work))
        {
          /* Start the timer if the work is the earliest expired work. */

          retimer = false;
          wd_start_abstick(&wqueue->timer, work->qtime,
                           work_timer_expired, (wdparm_t)wqueue);
        }
    }
  else
    {
      /* Insert to the expired list of the wqueue. */

      list_add_tail(&wqueue->expired, &work->node);
    }

  if (retimer)
    {
      work_timer_reset(wqueue);
    }

  spin_unlock_irqrestore(&wqueue->lock, flags);

  if (!delay)
    {
      /* Immediately wake up the worker thread. */

      nxsem_post(&wqueue->sem);
    }

  return 0;
}

int work_queue(int qid, FAR struct work_s *work, worker_t worker,
               FAR void *arg, clock_t delay)
{
  return work_queue_wq(work_qid2wq(qid), work, worker, arg, delay);
}

#endif /* CONFIG_SCHED_WORKQUEUE */
