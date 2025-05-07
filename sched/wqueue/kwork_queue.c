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
 * Name: work_queue_period/work_queue_period_wq
 *
 * Description:
 *   Queue work to be performed periodically.  All queued work will be
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
 *   period - Period (in clock ticks).
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

int work_queue_period_wq(FAR struct kwork_wqueue_s *wqueue,
                         FAR struct work_s *work, worker_t worker,
                         FAR void *arg, clock_t delay, clock_t period)
{
  irqstate_t flags;
  clock_t    expected;
  bool wake = false;
  int  ret  = OK;

  if (wqueue == NULL || work == NULL || worker == NULL)
    {
      return -EINVAL;
    }

  /* Ensure the work has been canceled. */

  work_cancel_wq(wqueue, work);

  /* delay+1 is to prevent the insufficient sleep time if we are
   * currently near the boundary to the next tick.
   * | current_tick | current_tick + 1 | current_tick + 2 | .... |
   * |           ^ Here we get the current tick
   * In this case we delay 1 tick, timer will be triggered at
   * current_tick + 1, which is not enough for at least 1 tick.
   */

  expected = clock_systime_ticks() + delay + 1;

  /* Interrupts are disabled so that this logic can be called from with
   * task logic or from interrupt handling logic.
   */

  flags = spin_lock_irqsave(&wqueue->lock);

  /* Initialize the work structure. */

  work->worker = worker;   /* Work callback. non-NULL means queued */
  work->arg    = arg;      /* Callback argument */
  work->qtime  = expected; /* Expected time */
  work->period = period;   /* Periodical delay */

  /* Insert to the pending list of the wqueue. */

  if (delay)
    {
      if (work_insert_pending(wqueue, work))
        {
          /* Start the timer if the work is the earliest expired work. */

          ret = wd_start_abstick(&wqueue->timer, work->qtime,
                                 work_timer_expired, (wdparm_t)wqueue);
        }
    }
  else
    {
      list_add_tail(&wqueue->expired, &work->node);
      wake = true;
    }

  spin_unlock_irqrestore(&wqueue->lock, flags);

  if (wake)
    {
      nxsem_post(&wqueue->sem);
    }

  return ret;
}

int work_queue_period(int qid, FAR struct work_s *work, worker_t worker,
                      FAR void *arg, clock_t delay, clock_t period)
{
  return work_queue_period_wq(work_qid2wq(qid), work, worker,
                              arg, delay, period);
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
  return work_queue_period_wq(wqueue, work, worker, arg, delay, 0);
}

int work_queue(int qid, FAR struct work_s *work, worker_t worker,
               FAR void *arg, clock_t delay)
{
  return work_queue_wq(work_qid2wq(qid), work, worker, arg, delay);
}

#endif /* CONFIG_SCHED_WORKQUEUE */
