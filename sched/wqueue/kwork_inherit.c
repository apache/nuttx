/****************************************************************************
 * sched/work/work_inherit.c
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

#if CONFIG_SEM_NNESTPRIO > 0
  /* If the priority of the client thread that is greater than the base
   * priority of the worker thread, then we may need to adjust the worker
   * thread's priority now or later to that priority.
   */

  if (reqprio > wtcb->base_priority)
    {
      /* If the new priority is greater than the current, possibly already
       * boosted priority of the worker thread, then we will have to raise
       * the worker thread's priority now.
       */

      if (reqprio > wtcb->sched_priority)
        {
          /* If the current priority of worker thread has already been
           * boosted, then add the boost priority to the list of restoration
           * priorities.  When the higher priority waiter thread gets its
           * count, then we need to revert the worker thread to this saved
           * priority (not to its base priority).
           */

          if (wtcb->sched_priority > wtcb->base_priority)
            {
              /* Save the current, boosted priority of the worker thread. */

              if (wtcb->npend_reprio < CONFIG_SEM_NNESTPRIO)
                {
                  wtcb->pend_reprios[wtcb->npend_reprio] =
                    wtcb->sched_priority;
                  wtcb->npend_reprio++;
                }
              else
                {
                  serr("ERROR: CONFIG_SEM_NNESTPRIO exceeded\n");
                  DEBUGASSERT(wtcb->npend_reprio < CONFIG_SEM_NNESTPRIO);
                }
            }

          /* Raise the priority of the worker.  This cannot cause a context
           * switch because we have preemption disabled.  The worker thread
           * may be marked "pending" and the switch may occur during
           * sched_unblock() processing.
           */

          nxsched_set_priority(wtcb, reqprio);
        }
      else
        {
          /* The new priority is above the base priority of the worker,
           * but not as high as its current working priority.  Just put it
           * in the list of pending restoration priorities so that when the
           * higher priority thread gets its count, we can revert to this
           * saved priority and not to the base priority.
           */

          if (wtcb->npend_reprio < CONFIG_SEM_NNESTPRIO)
            {
              wtcb->pend_reprios[wtcb->npend_reprio] = reqprio;
              wtcb->npend_reprio++;
            }
          else
            {
              serr("ERROR: CONFIG_SEM_NNESTPRIO exceeded\n");
              DEBUGASSERT(wtcb->npend_reprio < CONFIG_SEM_NNESTPRIO);
            }
        }
    }
#else
  /* If the priority of the client thread that is less than of equal to the
   * priority of the worker thread, then do nothing because the thread is
   * already running at a sufficient priority.
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
#endif
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
#if CONFIG_SEM_NNESTPRIO > 0
  uint8_t wpriority;
  int index;
  int selected;
#endif

  /* Get the TCB of the low priority worker thread from the process ID. */

  wtcb = nxsched_get_tcb(wpid);
  DEBUGASSERT(wtcb);

  /* Was the priority of the worker thread boosted? If so, then drop its
   * priority back to the correct level.  What is the correct level?
   */

  if (wtcb->sched_priority != wtcb->base_priority)
    {
#if CONFIG_SEM_NNESTPRIO > 0
      /* Are there other, pending priority levels to revert to? */

      if (wtcb->npend_reprio < 1)
        {
          /* No... the worker thread has only been boosted once.
           * npend_reprio should be 0 and the boosted priority should be the
           * priority of the client task (reqprio)
           *
           * That latter assumption may not be true if the client's priority
           * was also boosted so that it no longer matches the wtcb's
           * sched_priority.  Or if CONFIG_SEM_NNESTPRIO is too small (so
           * that we do not have a proper record of the reprioritizations).
           */

          DEBUGASSERT(/* wtcb->sched_priority == reqprio && */
                      wtcb->npend_reprio == 0);

          /* Reset the worker's priority back to the base priority. */

          nxsched_reprioritize(wtcb, wtcb->base_priority);
        }

      /* There are multiple pending priority levels. The worker thread's
       * "boosted" priority could greater than or equal to "reqprio" (it
       * could be greater if its priority we boosted because it also holds
       * some semaphore).
       */

      else if (wtcb->sched_priority <= reqprio)
        {
          /* The worker thread has been boosted to the same priority as the
           * waiter thread that just received the count.  We will simply
           * reprioritize to the next highest pending priority.
           */

          /* Find the highest pending priority and remove it from the list */

          for (index = 1, selected = 0; index < wtcb->npend_reprio; index++)
            {
              if (wtcb->pend_reprios[index] > wtcb->pend_reprios[selected])
                {
                  selected = index;
                }
            }

          /* Remove the highest priority pending priority from the list */

          wpriority = wtcb->pend_reprios[selected];
          index = wtcb->npend_reprio - 1;
          if (index > 0)
            {
              wtcb->pend_reprios[selected] = wtcb->pend_reprios[index];
            }

          wtcb->npend_reprio = index;

          /* And apply that priority to the thread (while retaining the
           * base_priority)
           */

          nxsched_set_priority(wtcb, wpriority);
        }
      else
        {
          /* The worker thread has been boosted to a higher priority than the
           * waiter task.  The pending priority should be in the list (unless
           * it was lost because of list overflow or because the worker was
           * reprioritized again unbeknownst to the priority inheritance
           * logic).
           *
           * Search the list for the matching priority.
           */

          for (index = 0; index < wtcb->npend_reprio; index++)
            {
              /* Does this pending priority match the priority of the thread
               * that just received the count?
               */

              if (wtcb->pend_reprios[index] == reqprio)
                {
                  /* Yes, remove it from the list */

                  selected = wtcb->npend_reprio - 1;
                  if (selected > 0)
                    {
                      wtcb->pend_reprios[index] =
                        wtcb->pend_reprios[selected];
                    }

                  wtcb->npend_reprio = selected;
                  break;
                }
            }
        }
#else
      /* There is no alternative restore priorities, drop the priority
       * of the worker thread all the way back to the threads "base"
       * priority.
       */

      nxsched_reprioritize(wtcb, wtcb->base_priority);
#endif
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
