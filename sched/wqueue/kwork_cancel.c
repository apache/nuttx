/****************************************************************************
 * sched/wqueue/kwork_cancel.c
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

#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/list.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#ifdef CONFIG_SCHED_WORKQUEUE

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int work_qcancel(FAR struct kwork_wqueue_s *wqueue, bool sync,
                        FAR struct work_s *work)
{
  irqstate_t flags;
  int        ret = OK;

  if (wqueue == NULL || work == NULL)
    {
      return -EINVAL;
    }

  /* Cancelling the work is simply a matter of removing the work structure
   * from the work queue.  This must be done with interrupts disabled because
   * new work is typically added to the work queue from interrupt handlers.
   */

  flags = spin_lock_irqsave(&wqueue->lock);

  /* Check whether we own the work structure. */

  if (!work_available(work))
    {
      bool is_head = list_is_head(&wqueue->pending, &work->node);

      /* Seize the ownership from the work thread. */

      work->worker = NULL;

      list_delete(&work->node);

      /* If the head of the pending queue has changed, we should reset
       * the wqueue timer.
       */

      if (is_head)
        {
          if (!list_is_empty(&wqueue->pending))
            {
              work = list_first_entry(&wqueue->pending, struct work_s, node);

              ret = wd_start_abstick(&wqueue->timer, work->qtime,
                                     work_timer_expired, (wdparm_t)wqueue);
            }
          else
            {
              wd_cancel(&wqueue->timer);
            }
        }
    }
  else if (!up_interrupt_context() && !sched_idletask() && sync)
    {
      int wndx;

      for (wndx = 0; wndx < wqueue->nthreads; wndx++)
        {
          if (wqueue->worker[wndx].work == work &&
              wqueue->worker[wndx].pid != nxsched_gettid())
            {
              wqueue->worker[wndx].wait_count++;
              spin_unlock_irqrestore(&wqueue->lock, flags);
              nxsem_wait_uninterruptible(&wqueue->worker[wndx].wait);
              return 1;
            }
        }
    }

  spin_unlock_irqrestore(&wqueue->lock, flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: work_cancel/work_cancel_wq
 *
 * Description:
 *   Cancel previously queued work.  This removes work from the work queue.
 *   After work has been cancelled, it may be requeued by calling
 *   work_queue() again.
 *
 * Input Parameters:
 *   qid    - The work queue ID (must be HPWORK or LPWORK)
 *   wqueue - The work queue handle
 *   work   - The previously queued work structure to cancel
 *
 * Returned Value:
 *   Zero on success, a negated errno on failure
 *
 *   -ENOENT - There is no such work queued.
 *   -EINVAL - An invalid work queue was specified
 *
 ****************************************************************************/

int work_cancel(int qid, FAR struct work_s *work)
{
  return work_qcancel(work_qid2wq(qid), false, work);
}

int work_cancel_wq(FAR struct kwork_wqueue_s *wqueue,
                   FAR struct work_s *work)
{
  return work_qcancel(wqueue, false, work);
}

/****************************************************************************
 * Name: work_cancel_sync/work_cancel_sync_wq
 *
 * Description:
 *   Blocked cancel previously queued user-mode work.  This removes work
 *   from the user mode work queue.  After work has been cancelled, it may
 *   be requeued by calling work_queue() again.
 *
 * Input Parameters:
 *   qid    - The work queue ID (must be HPWORK or LPWORK)
 *   wqueue - The work queue handle
 *   work   - The previously queued work structure to cancel
 *
 * Returned Value:
 *   Zero means the work was successfully cancelled.
 *   One means the work was not cancelled because it is currently being
 *   processed by work thread, but wait for it to finish.
 *   A negated errno value is returned on any failure:
 *
 *   -ENOENT - There is no such work queued.
 *   -EINVAL - An invalid work queue was specified
 *
 ****************************************************************************/

int work_cancel_sync(int qid, FAR struct work_s *work)
{
  return work_qcancel(work_qid2wq(qid), true, work);
}

int work_cancel_sync_wq(FAR struct kwork_wqueue_s *wqueue,
                        FAR struct work_s *work)
{
  return work_qcancel(wqueue, true, work);
}

#endif /* CONFIG_SCHED_WORKQUEUE */
