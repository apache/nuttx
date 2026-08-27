/****************************************************************************
 * libs/libc/wqueue/work_queue.c
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
#include <signal.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/list.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#if defined(CONFIG_LIBC_USRWORK) && !defined(__KERNEL__)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_qqueue
 *
 * Description:
 *   Queue work to be performed at a later time.  All queued work will be
 *   performed on the worker thread of execution (not the caller's).
 *
 *   The work structure is allocated by caller, but completely managed by
 *   the work queue logic.  The caller should never modify the contents of
 *   the work queue structure.  Calling work_qqueue() while the work is
 *   pending on the same queue cancels and replaces the pending instance.
 *
 * Input Parameters:
 *   wqueue - The work queue
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

static int work_qqueue(FAR struct usr_wqueue_s *wqueue,
                       FAR struct work_s *work, worker_t worker,
                       FAR void *arg, clock_t delay, bool period)
{
  FAR struct work_s *curr;
  FAR struct work_s *head;
  bool wake = false;
  int ret;

  if (wqueue == NULL || work == NULL || worker == NULL ||
      delay < 0 || delay > WDOG_MAX_DELAY)
    {
      return -EINVAL;
    }

  /* Get exclusive access to the work queue */

  do
    {
      ret = nxmutex_lock(&wqueue->lock);
    }
  while (ret < 0);

  if (wqueue->exit)
    {
      nxmutex_unlock(&wqueue->lock);
      return -ESHUTDOWN;
    }

  /* Remove a previous pending instance before requeueing it. */

  if (work->worker != NULL)
    {
      wake = work_remove(wqueue, work);
    }

  /* Initialize the work structure */

  work->worker = worker; /* Work callback. non-NULL means queued */
  work->arg    = arg;    /* Callback argument */

  if (period)
    {
      work->qtime += delay;
    }
  else if (delay > 0)
    {
      work->qtime = clock() + delay + 1;
    }
  else
    {
      work->qtime = clock();
    }

  /* Insert the work into the wait queue sorted by the expired time. */

  head = list_first_entry(&wqueue->q, struct work_s, node);

  list_for_every_entry(&wqueue->q, curr, struct work_s, node)
    {
      if (!clock_compare(curr->qtime, work->qtime))
        {
          break;
        }
    }

  /* After the insertion, we do not violate the invariant that
   * the wait queue is sorted by the expired time. Because
   * curr->qtime > work->qtime.
   * In the case of the wqueue is empty, we insert
   * the work at the head of the wait queue.
   */

  list_add_before(&curr->node, &work->node);

  /* Wake if this work becomes the new head.  Immediate work may be queued
   * behind other ready work, so wake another worker in the pool as well.
   */

  if (wake || delay == 0 || curr == head)
    {
      work_wake(wqueue);
    }

  nxmutex_unlock(&wqueue->lock);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_queue
 *
 * Description:
 *   Queue user-mode work to be performed at a later time.  All queued work
 *   will be performed on the worker thread of execution (not the caller's).
 *
 *   The work structure is allocated and must be initialized to all zero by
 *   the caller.  Otherwise, the work structure is completely managed by the
 *   work queue logic.  The caller should never modify the contents of the
 *   work queue structure directly.  If work_queue() is called before the
 *   previous work has been performed and removed from the queue, then any
 *   pending work will be canceled and lost.
 *
 * Input Parameters:
 *   qid    - The work queue ID (index)
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

int work_queue(int qid, FAR struct work_s *work, worker_t worker,
               FAR void *arg, clock_t delay)
{
  if (qid == USRWORK)
    {
      return work_qqueue(&g_usrwork, work, worker, arg, delay, false);
    }
  else
    {
      return -EINVAL;
    }
}

/****************************************************************************
 * Name: work_queue_wq
 *
 * Description:
 *   Queue work on a user-mode custom work queue.  This function must only
 *   be called from task context.
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PTHREAD
int work_queue_wq(FAR struct kwork_wqueue_s *handle,
                  FAR struct work_s *work, worker_t worker,
                  FAR void *arg, clock_t delay)
{
  return work_qqueue((FAR struct usr_wqueue_s *)handle, work,
                     worker, arg, delay, false);
}
#endif

/****************************************************************************
 * Name: work_queue_next/work_queue_next_wq
 ****************************************************************************/

int work_queue_next(int qid, FAR struct work_s *work, worker_t worker,
                    FAR void *arg, clock_t delay)
{
  if (qid == USRWORK)
    {
      return work_qqueue(&g_usrwork, work, worker, arg, delay, true);
    }

  return -EINVAL;
}

#ifndef CONFIG_DISABLE_PTHREAD
int work_queue_next_wq(FAR struct kwork_wqueue_s *handle,
                       FAR struct work_s *work, worker_t worker,
                       FAR void *arg, clock_t delay)
{
  return work_qqueue((FAR struct usr_wqueue_s *)handle, work,
                     worker, arg, delay, true);
}
#endif

#endif /* CONFIG_LIBC_USRWORK && !__KERNEL__ */
