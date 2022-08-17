/****************************************************************************
 * libs/libc/wqueue/work_queue.c
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
#include <queue.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/semaphore.h>

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
  FAR dq_entry_t *prev = NULL;
  FAR dq_entry_t *curr;
  sclock_t delta;
  int semcount;

  /* Get exclusive access to the work queue */

  while (_SEM_WAIT(&wqueue->lock) < 0);

  /* Initialize the work structure */

  work->worker = worker;             /* Work callback. non-NULL means queued */
  work->arg    = arg;                /* Callback argument */
  work->u.s.qtime = clock() + delay; /* Delay until work performed */

  /* Do the easy case first -- when the work queue is empty. */

  if (wqueue->q.head == NULL)
    {
      /* Add the watchdog to the head == tail of the queue. */

      dq_addfirst(&work->u.s.dq, &wqueue->q);
      _SEM_POST(&wqueue->wake);
    }

  /* There are other active watchdogs in the timer queue */

  else
    {
      curr = wqueue->q.head;

      /* Check if the new work must be inserted before the curr. */

      do
        {
          delta = work->u.s.qtime - ((FAR struct work_s *)curr)->u.s.qtime;
          if (delta < 0)
            {
              break;
            }

          prev = curr;
          curr = curr->flink;
        }
      while (curr != NULL);

      /* Insert the new watchdog in the list */

      if (prev == NULL)
        {
          /* Insert the watchdog at the head of the list */

          dq_addfirst(&work->u.s.dq, &wqueue->q);
          _SEM_GETVALUE(&wqueue->wake, &semcount);
          if (semcount < 1)
            {
              _SEM_POST(&wqueue->wake);
            }
        }
      else
        {
          /* Insert the watchdog in mid- or end-of-queue */

          dq_addafter(prev, &work->u.s.dq, &wqueue->q);
        }
    }

  _SEM_POST(&wqueue->lock);
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
