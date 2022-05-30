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

static sem_t g_kwork_dummy_sem;

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
  /* If the priority of the thread that is waiting for a count is greater
   * than the base priority of the thread holding a count, then we add this
   * priority for the semaphore to the list of boosting semaphores
   */

  if (reqprio > wtcb->base_priority)
    {
      if (wtcb->nsem_boosts < CONFIG_SEM_NNESTPRIO)
        {
          /* Store this boost in the list of active boosts */

          struct semboost_s *boost = &wtcb->sem_boosts[wtcb->nsem_boosts];
          wtcb->nsem_boosts++;

          /* Store reference to dummy semaphore to record boost */

          boost->sem = &g_kwork_dummy_sem;
          boost->priority = reqprio;
          /* If the boost we just received is a new maximum. We need to boost
           * ourselves
           */

          if (boost->priority > wtcb->sched_priority)
            {
              nxsched_set_priority(wtcb, boost->priority);
            }
        }
      else
        {
          serr("ERROR: TCB %p out of priority boost slots.", wtcb);
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

  /* Get the TCB of the low priority worker thread from the process ID. */

  wtcb = nxsched_get_tcb(wpid);
  DEBUGASSERT(wtcb);

  /* Was the priority of the worker thread boosted? If so, then drop its
   * priority back to the correct level.  What is the correct level?
   */

  if (wtcb->sched_priority != wtcb->base_priority)
    {
#if CONFIG_SEM_NNESTPRIO > 0
      /* Priority is supposed to go back to what it was before.
       * We can remove the highest boost for kwork boosts from our
       * list of boosts, and re-evaluate what is the now the highest
       * priority still waiting
       */

      int max_boost_index = -1;
      uint8_t max_boost_priority = 0;
      for (int i = 0; i < wtcb->nsem_boosts; i++)
        {
          if (wtcb->sem_boosts[i].sem == &g_kwork_dummy_sem)
            {
              if (max_boost_priority < wtcb->sem_boosts[i].priority)
                {
                  max_boost_priority = wtcb->sem_boosts[i].priority;
                  max_boost_index = i;
                }
            }
        }

      if (max_boost_index >= 0)
        {
          /* We found the maximum boost for kwork boost on this task.
           * Remove this, as this is no longer required
           * remove (replace max with last, decrease count)
           */

          wtcb->sem_boosts[max_boost_index] =
            wtcb->sem_boosts[wtcb->nsem_boosts - 1];
          wtcb->nsem_boosts--;
        }

      /* Find new max priority by going through the boosts still present */

      uint8_t new_priority = wtcb->base_priority;
      for (int i = 0; i < wtcb->nsem_boosts; i++)
        {
          new_priority = (wtcb->sem_boosts[i].priority > new_priority) ?
              wtcb->sem_boosts[i].priority : new_priority;
        }

      if (new_priority != wtcb->sched_priority)
        {
          nxsched_set_priority(wtcb, new_priority);
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
