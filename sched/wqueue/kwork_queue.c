/****************************************************************************
 * sched/wqueue/kwork_queue.c
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
      int sem_count; \
      dq_addlast((FAR dq_entry_t *)(work), &(wqueue)->q); \
      nxsem_get_value(&(wqueue)->sem, &sem_count); \
      if (sem_count < 0) /* There are threads waiting for sem. */ \
        { \
          nxsem_post(&(wqueue)->sem); \
        } \
    } \
  while (0)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hp_work_timer_expiry
 ****************************************************************************/

#ifdef CONFIG_SCHED_HPWORK
static void hp_work_timer_expiry(wdparm_t arg)
{
  irqstate_t flags = enter_critical_section();
  queue_work(&g_hpwork, arg);
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: lp_work_timer_expiry
 ****************************************************************************/

#ifdef CONFIG_SCHED_LPWORK
static void lp_work_timer_expiry(wdparm_t arg)
{
  irqstate_t flags = enter_critical_section();
  queue_work(&g_lpwork, arg);
  leave_critical_section(flags);
}
#endif

static bool work_is_canceling(FAR struct kworker_s *kworkers, int nthreads,
                              FAR struct work_s *work)
{
  int semcount;

  for (int wndx = 0; wndx < nthreads; wndx++)
    {
      if (kworkers[wndx].work == work)
        {
          nxsem_get_value(&kworkers[wndx].wait, &semcount);
          if (semcount < 0)
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
 * Name: work_queue
 *
 * Description:
 *   Queue kernel-mode work to be performed at a later time.  All queued
 *   work will be performed on the worker thread of execution (not the
 *   caller's).
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
  FAR struct kwork_wqueue_s * wqueue;
  wdentry_t expiry;
  irqstate_t flags;
  int nthreads;
  int ret = OK;

  /* Interrupts are disabled so that this logic can be called from with
   * task logic or from interrupt handling logic.
   */

  flags = enter_critical_section();

  /* Remove the entry from the timer and work queue. */

  if (work->worker != NULL)
    {
      work_cancel(qid, work);
    }

#ifdef CONFIG_SCHED_HPWORK
  if (qid == HPWORK)
    {
      wqueue = (FAR struct kwork_wqueue_s *)&g_hpwork;
      expiry = hp_work_timer_expiry;
      nthreads = CONFIG_SCHED_HPNTHREADS;
    }
  else
#endif
#ifdef CONFIG_SCHED_LPWORK
  if (qid == LPWORK)
    {
      wqueue = (FAR struct kwork_wqueue_s *)&g_lpwork;
      expiry = lp_work_timer_expiry;
      nthreads = CONFIG_SCHED_LPNTHREADS;
    }
  else
#endif
    {
      ret = -EINVAL;
      goto out;
    }

  if (work_is_canceling(wqueue->worker, nthreads, work))
    {
      goto out;
    }

  /* Initialize the work structure. */

  work->worker = worker;           /* Work callback. non-NULL means queued */
  work->arg = arg;                 /* Callback argument */

  /* Queue the new work */

  if (!delay)
    {
      queue_work(wqueue, work);
    }
  else
    {
      wd_start(&work->u.timer, delay, expiry,
               (wdparm_t)work);
    }

out:
  leave_critical_section(flags);
  return ret;
}

#endif /* CONFIG_SCHED_WORKQUEUE */
