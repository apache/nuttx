/****************************************************************************
 * sched/paging/pg_worker.c
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
#include <stdbool.h>
#include <unistd.h>
#include <queue.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>
#include <nuttx/page.h>
#include <nuttx/clock.h>
#include <nuttx/signal.h>

#include "sched/sched.h"
#include "paging/paging.h"

#ifdef CONFIG_PAGING

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* This is the task ID of the page fill worker thread.  This value was set in
 * nx_start when the page fill worker thread was started.
 */

pid_t g_pgworker;

/* The page fill worker thread maintains a static variable called g_pftcb.
 * If no fill is in progress, g_pftcb will be NULL. Otherwise, g_pftcb will
 * point to the TCB of the task which is receiving the fill that is in
 * progress.
 *
 * NOTE: I think that this is the only state in which a TCB does not reside
 * in some list.  Here is it in limbo, outside of the normally queuing while
 * the page file is in progress.  Where here, it will be marked with
 * TSTATE_TASK_INVALID.
 */

FAR struct tcb_s *g_pftcb;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_PAGING_BLOCKINGFILL

/* When a page fill completes, the result of the fill is stored here.  The
 * value -EBUSY means that the page fill callback has not yet been received.
 */

static int g_fillresult;

/* A configurable timeout period (in clock ticks) may be select to detect
 * page fill failures.
 */

#ifdef CONFIG_PAGING_TIMEOUT_TICKS
static clock_t g_starttime;
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pg_callback
 *
 * Description:
 *   This function is called from the architecture-specific, page fill logic
 *   when the page fill completes (with or without an error).  A reference to
 *   this function was provided to up_fillpage().  The driver will provide
 *   the result of the fill as an argument.
 *
 *   NOTE: pg_callback() must also be locked in memory.
 *
 * When pg_callback() is called, it will perform the following operations:
 *
 * - Verify that g_pftcb is non-NULL.
 * - Find the higher priority between the task waiting for the fill to
 *   complete in g_pftcb and the task waiting at the head of the
 *   g_waitingforfill list.  That will be the priority of he highest priority
 *   task waiting for a fill.
 * - If this higher priority is higher than current page fill worker thread,
 *   then boost worker thread's priority to that level. Thus, the page fill
 *   worker thread will always run at the priority of the highest priority
 *   task that is waiting for a fill.
 * - Signal the page fill worker thread.
 *
 * Input Parameters:
 *   tcb    - The TCB of the task that just received the fill.
 *   result - The result of the page fill operation.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Possibly executing in the context of a driver interrupt handler???
 *
 ****************************************************************************/

#ifndef CONFIG_PAGING_BLOCKINGFILL
static void pg_callback(FAR struct tcb_s *tcb, int result)
{
  /* Verify that g_pftcb is non-NULL */

  pginfo("g_pftcb: %p\n", g_pftcb);
  if (g_pftcb)
    {
      FAR struct tcb_s *htcb = (FAR struct tcb_s *)g_waitingforfill.head;
      FAR struct tcb_s *wtcb = nxsched_get_tcb(g_pgworker);

      /* Find the higher priority between the task waiting for the fill to
       * complete in g_pftcb and the task waiting at the head of the
       * g_waitingforfill list.  That will be the priority of he highest
       * priority task waiting for a fill.
       */

      int priority = g_pftcb->sched_priority;
      if (htcb && priority < htcb->sched_priority)
        {
          priority = htcb->sched_priority;
        }

      /* If this higher priority is higher than current page fill worker
       * thread, then boost worker thread's priority to that level. Thus,
       * the page fill worker thread will always run at the priority of
       * the highest priority task that is waiting for a fill.
       */

      if (priority > wtcb->sched_priority)
        {
          pginfo("New worker priority. %d->%d\n",
                 wtcb->sched_priority, priority);
          nxsched_set_priority(wtcb, priority);
        }

      /* Save the page fill result (don't permit the value -EBUSY) */

      if (result == -EBUSY)
        {
          result = -ENOSYS;
        }

      g_fillresult = result;
    }

  /* Signal the page fill worker thread (in any event) */

  pginfo("Signaling worker. PID: %d\n", g_pgworker);
  nxsig_kill(g_pgworker, SIGWORK);
}
#endif

/****************************************************************************
 * Name: pg_dequeue
 *
 * Description:
 *   Dequeue the next, highest priority TCB from the g_waitingforfill task
 *   list.  Call up_checkmapping() see if the still needs to be performed
 *   for that task. In certain conditions, the page fault may occur on
 *   several threads for the same page and be queued multiple times. In this
 *   corner case, the blocked task will simply be restarted.
 *
 *   This function will continue to examine g_waitingforfill until either
 *   (1) a task is found that still needs the page fill, or (2) the
 *   g_waitingforfill task list becomes empty.
 *
 *   The result (NULL or a TCB pointer) will be returned in the global
 *   variable, g_pftcb.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   If there are no further queue page fill operations to be performed,
 *   pg_dequeue() will return false.  Otherwise, it will return
 *   true to that the full is in process (any errors will result in
 *   assertions and this function will not return).
 *
 * Assumptions:
 *   Executing in the context of the page fill worker thread with all
 *   interrupts disabled.
 *
 ****************************************************************************/

static inline bool pg_dequeue(void)
{
  /* Loop until either (1) the TCB of a task that requires a fill is found,
   * OR (2) the g_watingforfill list becomes empty.
   */

  do
    {
      /* Remove the TCB from the head of the list (if any) */

      g_pftcb = (FAR struct tcb_s *)
        dq_remfirst((FAR dq_queue_t *)&g_waitingforfill);
      pginfo("g_pftcb: %p\n", g_pftcb);
      if (g_pftcb != NULL)
        {
          /* Call the architecture-specific function up_checkmapping() to see
           * if the page fill still needs to be performed. In certain
           * conditions, the page fault may occur on several threads for the
           * same page and be queues multiple times. In this corner case, the
           * blocked task will simply be restarted.
           */

          if (!up_checkmapping(g_pftcb))
            {
              /* This page needs to be filled.  pg_miss bumps up
               * the priority of the page fill worker thread as each
               * TCB is added to the g_waitingforfill list.  So we
               * may need to also drop the priority of the worker
               * thread as the next TCB comes off of the list.
               *
               * If wtcb->sched_priority > CONFIG_PAGING_DEFPRIO,
               * then the page fill worker thread is executing at
               * an elevated priority that may be reduced.
               *
               * If wtcb->sched_priority > g_pftcb->sched_priority
               * then the page fill worker thread is executing at
               * a higher priority than is appropriate for this
               * fill (this priority can get re-boosted by pg_miss()
               * if a new higher priority fill is required).
               */

              FAR struct tcb_s *wtcb = this_task();
              if (wtcb->sched_priority > CONFIG_PAGING_DEFPRIO &&
                  wtcb->sched_priority > g_pftcb->sched_priority)
                {
                  /* Don't reduce the priority of the page fill
                   * worker thread lower than the configured
                   * minimum.
                   */

                  int priority = g_pftcb->sched_priority;
                  if (priority < CONFIG_PAGING_DEFPRIO)
                    {
                      priority = CONFIG_PAGING_DEFPRIO;
                    }

                  /* Reduce the priority of the page fill worker thread */

                  pginfo("New worker priority. %d->%d\n",
                         wtcb->sched_priority, priority);
                  nxsched_set_priority(wtcb, priority);
                }

              /* Return with g_pftcb holding the pointer to
               * the TCB associated with task that requires the page fill.
               */

              return true;
            }

          /* The page need by this task has already been mapped into the
           * virtual address space -- just restart it.
           */

          pginfo("Restarting TCB: %p\n", g_pftcb);
          up_unblock_task(g_pftcb);
        }
    }
  while (g_pftcb != NULL);

  return false;
}

/****************************************************************************
 * Name: pg_startfill
 *
 * Description:
 *   Start a page fill operation on the thread whose TCB is at the head of
 *   of the g_waitingforfill task list.  That is a prioritized list so that
 *   will be the highest priority task waiting for a page fill (in the event
 *   that are multiple tasks waiting for a page fill).
 *
 *   This function may be called either (1) when the page fill worker thread
 *   is notified that there is a new page fill TCB in the g_waitingforfill
 *   prioritized list, or (2) when a page fill completes and there are more
 *   pages to be filled in g_waitingforfill list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   If there are no further queue page fill operations to be performed,
 *   pg_startfill() will return false.  Otherwise, it will return
 *   true to that the full is in process (any errors will result in
 *   assertions and this function will not return).
 *
 * Assumptions:
 *   Executing in the context of the page fill worker thread with all
 *   interrupts disabled.
 *
 ****************************************************************************/

static inline bool pg_startfill(void)
{
  FAR void *vpage;
  int result;

  /* Remove the TCB at the head of the g_waitfor fill list and check if there
   * is any task waiting for a page fill. pg_dequeue will handle this (plus
   * some corner cases) and will true if the next page TCB was successfully
   * dequeued.
   */

  if (pg_dequeue())
    {
      /* Call up_allocpage(tcb, &vpage). This architecture-specific function
       * will set aside page in memory and map to virtual address (vpage). If
       * all available pages are in-use (the typical case), this function
       * will select a page in-use, un-map it, and make it available.
       */

      pginfo("Call up_allocpage(%p)\n", g_pftcb);
      result = up_allocpage(g_pftcb, &vpage);
      DEBUGASSERT(result == OK);

      /* Start the fill.  The exact way that the fill is started depends upon
       * the nature of the architecture-specific up_fillpage() function -- Is
       * it a blocking or a non-blocking call?
       */

#ifdef CONFIG_PAGING_BLOCKINGFILL
      /* If CONFIG_PAGING_BLOCKINGFILL is defined, then up_fillpage is
       * blocking call. In this case, up_fillpage() will accept only (1) a
       * reference to the TCB that requires the fill. Architecture-specific
       * context information within the TCB will be sufficient to perform the
       * fill. And (2) the (virtual) address of the allocated page to be
       * filled. The resulting status of the fill will be provided by return
       * value from up_fillpage().
       */

      pginfo("Call up_fillpage(%p)\n", g_pftcb);
      result = up_fillpage(g_pftcb, vpage);
      DEBUGASSERT(result == OK);
#else
      /* If CONFIG_PAGING_BLOCKINGFILL is defined, then up_fillpage is non-
       * blocking call. In this case up_fillpage() will accept an additional
       * argument: The page fill worker thread will provide a callback
       * function, pg_callback.
       * up_fillpage will start an asynchronous page fill. pg_callback
       * will be called when the page fill is finished (or an error occurs).
       * This callback will probably from interrupt level.
       */

      pginfo("Call up_fillpage(%p)\n", g_pftcb);
      result = up_fillpage(g_pftcb, vpage, pg_callback);
      DEBUGASSERT(result == OK);

      /* Save the time that the fill was started.  These will be used to
       * check for timeouts.
       */

#ifdef CONFIG_PAGING_TIMEOUT_TICKS
      g_starttime = clock_systime_ticks();
#endif

      /* Return and wait to be signaled for the next event -- the fill
       * completion event. While the fill is in progress, other tasks may
       * execute. If another page fault occurs during this time, the faulting
       * task will be blocked, its TCB will be added (in priority order) to
       * g_waitingforfill and the priority of the page worker task may be
       * boosted. But no action will be taken until the current page fill
       * completes. NOTE: The IDLE task must also be fully locked in memory.
       * The IDLE task cannot be blocked. It the case where all tasks are
       * blocked waiting for a page fill, the IDLE task must still be
       * available to run.
       */

#endif /* CONFIG_PAGING_BLOCKINGFILL */

      return true;
    }

  pginfo("Queue empty\n");
  UNUSED(result);
  return false;
}

/****************************************************************************
 * Name: pg_alldone
 *
 * Description:
 *   Called by the page fill worker thread when all pending page fill
 *   operations have been completed and the g_waitingforfill list is empty.
 *
 *   This function will perform the following operations:
 *
 *   - Set g_pftcb to NULL.
 *   - Restore the default priority of the page fill worker thread.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Executing in the context of the page fill worker thread with interrupts
 *   disabled.
 *
 ****************************************************************************/

static inline void pg_alldone(void)
{
  FAR struct tcb_s *wtcb = this_task();
  g_pftcb = NULL;
  pginfo("New worker priority. %d->%d\n",
         wtcb->sched_priority, CONFIG_PAGING_DEFPRIO);
  nxsched_set_priority(wtcb, CONFIG_PAGING_DEFPRIO);
}

/****************************************************************************
 * Name: pg_fillcomplete
 *
 * Description:
 *   Called by the page fill worker thread when a page fill completes.
 *   Either (1) in the non-blocking up_fillpage(), after the architecture-
 *   specific driver call the pg_callback() to wake up the page fill worker
 *   thread, or (2) after the blocking up_fillpage() returns (when
 *   CONFIG_PAGING_BLOCKINGFILL is defined).
 *
 *   This function is just a dumb wrapper around up_unblocktask().  This
 *   function simply makes the task that just received the fill ready-to-run.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Executing in the context of the page fill worker thread with interrupts
 *   disabled.
 *
 ****************************************************************************/

static inline void pg_fillcomplete(void)
{
  /* Call up_unblocktask(g_pftcb) to make the task that just
   * received the fill ready-to-run.
   */

  pginfo("Restarting TCB: %p\n", g_pftcb);
  up_unblock_task(g_pftcb);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pg_worker
 *
 * Description:
 *   This is the page fill worker thread that performs pages fills for tasks
 *   that have received a pag fault and are blocked in the g_waitingforfill
 *   task queue.
 *
 *   The page fill worker thread will be awakened on one of three conditions:
 *   - When signaled by pg_miss(), the page fill worker thread will be
 *     awakened, or
 *   - if CONFIG_PAGING_BLOCKINGFILL is not defined, from pg_callback()
 *     after completing a page fill.
 *   - A configurable timeout with no activity.
 *
 * Input Parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

int pg_worker(int argc, char *argv[])
{
  /* Loop forever -- Notice that interrupts will be disabled at all times
   * that this thread runs.  That is so that we can't lose signals or have
   * asynchronous page faults.
   *
   * All interrupt logic as well as all page fill worker thread logic must
   * be locked in memory.  Therefore, keeping interrupts disabled here
   * should prevent any concurrent page faults.  Any page faults or page
   * fill completions should occur while this thread sleeps.
   */

  pginfo("Started\n");
  up_irq_save();
  for (; ; )
    {
      /* Wait awhile.  We will wait here until either the configurable
       * timeout elapses or until we are awakened by a signal (which
       * terminates the nxsig_usleep with an EINTR error).  Note that
       * interrupts will be re- enabled while this task sleeps.
       *
       * The timeout is a failsafe that will handle any cases where a single
       * is lost (that would really be a bug and shouldn't happen!) and also
       * supports timeouts for case of non-blocking, asynchronous fills.
       */

      nxsig_usleep(CONFIG_PAGING_WORKPERIOD);

      /* The page fill worker thread will be awakened on one of 3 conditions:
       *
       *   - When signaled by pg_miss(), the page fill worker thread will be
       *     awakened,
       *   - if CONFIG_PAGING_BLOCKINGFILL is not defined, from pg_callback()
       *     after completing a page fill, or
       *   - On a configurable timeout expires with no activity.
       *
       * Interrupts are still disabled.
       */

#ifndef CONFIG_PAGING_BLOCKINGFILL
      /* For the non-blocking up_fillpage(), the page fill worker thread will
       * detect that the page fill is complete when it is awakened with
       * g_pftcb non-NULL and fill completion status from pg_callback.
       */

      if (g_pftcb != NULL)
        {
          /* If it is a real page fill completion event, then the result of
           * the page fill will be in g_fillresult and will not be equal to
           * -EBUSY.
           */

          if (g_fillresult != -EBUSY)
            {
              /* Any value other than OK, brings the system down */

              DEBUGASSERT(g_fillresult == OK);

              /* Handle the successful page fill complete event by restarting
               * the task that was blocked waiting for this page fill.
               */

              pginfo("Restarting TCB: %p\n", g_pftcb);
              up_unblock_task(g_pftcb);

              /* Yes .. Start the next asynchronous fill.  Check the return
               * value to see a fill was actually started (false means that
               * no fill was started).
               */

              pginfo("Calling pg_startfill\n");
              if (!pg_startfill())
                {
                  /* No fill was started.  This can mean only that all queued
                   * page fill actions have and been completed and there is
                   * nothing more to do.
                   */

                  pginfo("Call pg_alldone()\n");
                  pg_alldone();
                }
            }

          /* If a configurable timeout period expires with no page fill
           * completion event, then declare a failure.
           */

#ifdef CONFIG_PAGING_TIMEOUT_TICKS
          else
            {
              pgerr("ERROR: Timeout!\n");
              DEBUGASSERT(clock_systime_ticks() - g_starttime <
                          CONFIG_PAGING_TIMEOUT_TICKS);
            }
#endif
        }

      /* Otherwise, this might be a page fill initiation event.  When
       * awakened from pg_miss(), no fill will be in progress and
       * g_pftcb will be NULL.
       */

      else
        {
          /* Are there tasks blocked and waiting for a fill?  If so,
           * pg_startfill() will start the asynchronous fill (and set
           * g_pftcb).
           */

          pginfo("Calling pg_startfill\n");
          pg_startfill();
        }
#else
      /* Are there tasks blocked and waiting for a fill?  Loop until all
       * pending fills have been processed.
       */

      for (; ; )
        {
          /* Yes .. Start the fill and block until the fill completes.
           * Check the return value to see a fill was actually performed.
           * (false means that no fill was performed).
           */

          pginfo("Calling pg_startfill\n");
          if (!pg_startfill())
            {
              /* Break out of the loop -- there is nothing more to do */

              break;
            }

          /* Handle the page fill complete event by restarting the
           * task that was blocked waiting for this page fill. In the
           * non-blocking fill case, the page fill worker thread will
           * know that the page fill is  complete when pg_startfill()
           * returns true.
           */

          pginfo("Restarting TCB: %p\n", g_pftcb);
          up_unblock_task(g_pftcb);
        }

      /* All queued fills have been processed */

      pginfo("Call pg_alldone()\n");
      pg_alldone();
#endif
    }

  return OK; /* To keep some compilers happy */
}
#endif /* CONFIG_PAGING */
