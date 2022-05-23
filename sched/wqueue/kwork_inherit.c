/****************************************************************************
 * sched/wqueue/kwork_inherit.c
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

#include <sched.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/wqueue.h>

#include "sched/sched.h"
#include "wqueue/wqueue.h"

#if defined(CONFIG_SCHED_WORKQUEUE) && defined(CONFIG_SCHED_LPWORK) && \
    defined(CONFIG_PRIORITY_INHERITANCE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpwork_boostworker
 *
 * Description:
 *   Called by the work queue client to assure that the priority of the low-
 *   priority worker thread is at least at the requested level, reqprio. This
 *   function would normally be called just before calling work_queue().
 *
 * Input Parameters:
 *   reqprio - Requested minimum worker thread priority
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpwork_boostworker(pid_t wpid, uint8_t reqprio)
{
  FAR struct tcb_s *wtcb;

  /* Get the TCB of the low priority worker thread from the process ID. */

  wtcb = nxsched_get_tcb(wpid);
  DEBUGASSERT(wtcb);

  /* REVISIT: Priority multi-boost is not supported */

  DEBUGASSERT(wtcb->boost_priority == 0);

  /* If the priority of the client thread that is greater than the base
   * priority of the worker thread, then we may need to adjust the worker
   * thread's priority now or later to that priority.
   */

  if (reqprio > wtcb->base_priority)
    {
      /* Save boost priority value as it might be needed in case of multiple
       * re-prioritisations happen, then the priority of the thread can't go
       * below the boost priority value until priority boost is canceled.
       */

      wtcb->boost_priority = reqprio;

      /* If the priority of the client thread that is less than of equal to
       * the priority of the worker thread, then do nothing because the
       * thread is already running at a sufficient priority.
       */

      if (reqprio > wtcb->sched_priority)
        {
          /* Raise the priority of the worker thread.  This cannot cause
           * context switch because we have preemption disabled.  The task
           * will be marked "pending" and the switch will occur during
           * sched_unlock() processing.
           */

          nxsched_set_priority(wtcb, reqprio);
        }
    }
}

/****************************************************************************
 * Name: lpwork_restoreworker
 *
 * Description:
 *   This function is called to restore the priority after it was previously
 *   boosted.  This is often done by client logic on the worker thread when
 *   the scheduled work completes.  It will check if we need to drop the
 *   priority of the worker thread.
 *
 * Input Parameters:
 *   reqprio - Previously requested minimum worker thread priority to be
 *     "unboosted"
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void lpwork_restoreworker(pid_t wpid, uint8_t reqprio)
{
  FAR struct tcb_s *wtcb;

  /* Get the TCB of the low priority worker thread from the process ID. */

  wtcb = nxsched_get_tcb(wpid);
  DEBUGASSERT(wtcb);

  /* REVISIT: Priority multi-boost is not supported. */

  DEBUGASSERT(wtcb->boost_priority == reqprio);

  /* Clear the threat boost priority. */

  wtcb->boost_priority = 0;

  /* Was the priority of the worker thread boosted? If so, then drop its
   * priority back to the correct level.  What is the correct level?
   */

  if (wtcb->sched_priority != wtcb->base_priority)
    {
      FAR struct semholder_s *pholder;
      uint8_t wpriority;

      /* We attempt to restore task priority to its base priority.  If there
       * is any task with the higher priority waiting for the semaphore
       * held by wtcb then this value will be overwritten.
       */

      wpriority = wtcb->base_priority;

      /* Try to find the highest priority across all the tasks that are
       * waiting for any semaphore held by wtcb.
       */

      for (pholder = wtcb->holdsem; pholder != NULL;
           pholder = pholder->tlink)
        {
          FAR struct tcb_s *stcb;

          stcb = (FAR struct tcb_s *)dq_peek(SEM_WAITLIST(pholder->sem));

          if (stcb != NULL && stcb->sched_priority > wpriority)
            {
              wpriority = stcb->sched_priority;
            }
        }

      /* Apply the selected priority to the worker thread (hopefully back
       * to the threads base_priority).
       */

      nxsched_set_priority(wtcb, wpriority);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpwork_boostpriority
 *
 * Description:
 *   Called by the work queue client to assure that the priority of the low-
 *   priority worker thread is at least at the requested level, reqprio. This
 *   function would normally be called just before calling work_queue().
 *
 * Input Parameters:
 *   reqprio - Requested minimum worker thread priority
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpwork_boostpriority(uint8_t reqprio)
{
  irqstate_t flags;
  int wndx;

  /* Clip to the configured maximum priority */

  if (reqprio > CONFIG_SCHED_LPWORKPRIOMAX)
    {
      reqprio = CONFIG_SCHED_LPWORKPRIOMAX;
    }

  /* Prevent context switches until we get the priorities right */

  flags = enter_critical_section();
  sched_lock();

  /* Adjust the priority of every worker thread */

  for (wndx = 0; wndx < CONFIG_SCHED_LPNTHREADS; wndx++)
    {
      lpwork_boostworker(g_lpwork.worker[wndx].pid, reqprio);
    }

  sched_unlock();
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: lpwork_restorepriority
 *
 * Description:
 *   This function is called to restore the priority after it was previously
 *   boosted.  This is often done by client logic on the worker thread when
 *   the scheduled work completes.  It will check if we need to drop the
 *   priority of the worker thread.
 *
 * Input Parameters:
 *   reqprio - Previously requested minimum worker thread priority to be
 *     "unboosted"
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void lpwork_restorepriority(uint8_t reqprio)
{
  irqstate_t flags;
  int wndx;

  /* Clip to the configured maximum priority */

  if (reqprio > CONFIG_SCHED_LPWORKPRIOMAX)
    {
      reqprio = CONFIG_SCHED_LPWORKPRIOMAX;
    }

  /* Prevent context switches until we get the priorities right */

  flags = enter_critical_section();
  sched_lock();

  /* Adjust the priority of every worker thread */

  for (wndx = 0; wndx < CONFIG_SCHED_LPNTHREADS; wndx++)
    {
      lpwork_restoreworker(g_lpwork.worker[wndx].pid, reqprio);
    }

  sched_unlock();
  leave_critical_section(flags);
}

#endif /* CONFIG_SCHED_WORKQUEUE && CONFIG_SCHED_LPWORK && \
        * CONFIG_PRIORITY_INHERITANCE */
