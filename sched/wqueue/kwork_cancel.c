/****************************************************************************
 * sched/wqueue/kwork_cancel.c
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

#include <queue.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>

#include "wqueue/wqueue.h"

#ifdef CONFIG_SCHED_WORKQUEUE

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
 *   qid    - The work queue ID
 *   work   - The previously queued work structure to cancel
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno on failure.  This error may be
 *   reported:
 *
 *   -ENOENT - There is no such work queued.
 *   -EINVAL - An invalid work queue was specified
 *
 ****************************************************************************/

static int work_qcancel(FAR struct kwork_wqueue_s *wqueue,
                        FAR struct work_s *work)
{
  irqstate_t flags;
  int ret = -ENOENT;

  DEBUGASSERT(work != NULL);

  /* Cancelling the work is simply a matter of removing the work structure
   * from the work queue.  This must be done with interrupts disabled because
   * new work is typically added to the work queue from interrupt handlers.
   */

  flags = enter_critical_section();
  if (work->worker != NULL)
    {
      /* Remove the entry from the work queue and make sure that it is
       * marked as available (i.e., the worker field is nullified).
       */

      if (WDOG_ISACTIVE(&work->u.timer))
        {
          wd_cancel(&work->u.timer);
        }
      else
        {
          dq_rem((FAR dq_entry_t *)work, &wqueue->q);
        }

      work->worker = NULL;
      ret = OK;
    }

  leave_critical_section(flags);
  return ret;
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
 *   qid    - The work queue ID (must be HPWORK or LPWORK)
 *   work   - The previously queued work structure to cancel
 *
 * Returned Value:
 *   Zero (OK) on success, a negated errno on failure.  This error may be
 *   reported:
 *
 *   -ENOENT - There is no such work queued.
 *   -EINVAL - An invalid work queue was specified
 *
 ****************************************************************************/

int work_cancel(int qid, FAR struct work_s *work)
{
#ifdef CONFIG_SCHED_HPWORK
  if (qid == HPWORK)
    {
      /* Cancel high priority work */

      return work_qcancel((FAR struct kwork_wqueue_s *)&g_hpwork, work);
    }
  else
#endif
#ifdef CONFIG_SCHED_LPWORK
  if (qid == LPWORK)
    {
      /* Cancel low priority work */

      return work_qcancel((FAR struct kwork_wqueue_s *)&g_lpwork, work);
    }
  else
#endif
    {
      return -EINVAL;
    }
}

#endif /* CONFIG_SCHED_WORKQUEUE */
