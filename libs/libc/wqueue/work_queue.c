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
 *   the work queue structure; the caller should not call work_qqueue()
 *   again until either (1) the previous work has been performed and removed
 *   from the queue, or (2) work_cancel() has been called to cancel the work
 *   and remove it from the work queue.
 *
 * Input Parameters:
 *   wqueue - The work queue
 *   work   - The work structure to queue
 *   worker - The worker callback to be invoked.  The callback will be
 *            invoked on the worker thread of execution.
 *   arg    - The argument that will be passed to the worker callback when
 *            int is invoked.
 *   delay  - Delay (in clock ticks) from the time queue until the worker
 *            is invoked. Zero means to perform the work immediately.
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 ****************************************************************************/

static int work_qqueue(FAR struct usr_wqueue_s *wqueue,
                       FAR struct work_s *work, worker_t worker,
                       FAR void *arg, clock_t delay)
{
  FAR struct work_s *curr;
  FAR struct work_s *head;
  int semcount;

  /* Get exclusive access to the work queue */

  while (nxmutex_lock(&wqueue->lock) < 0);

  /* Initialize the work structure */

  work->worker = worker;          /* Work callback. non-NULL means queued */
  work->arg    = arg;             /* Callback argument */
  work->qtime  = clock() + delay; /* Delay until work performed */

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

  /* If the current work is the head of the wait queue.
   * We should wake up the worker thread.
   */

  if (curr == head)
    {
      nxsem_get_value(&wqueue->wake, &semcount);
      if (semcount < 1)
        {
          nxsem_post(&wqueue->wake);
        }
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
 *            int is invoked.
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
      /* Is there already pending work? */

      work_cancel(qid, work);

      return work_qqueue(&g_usrwork, work, worker, arg, delay);
    }
  else
    {
      return -EINVAL;
    }
}

#endif /* CONFIG_LIBC_USRWORK && !__KERNEL__ */
