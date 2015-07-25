/************************************************************************
 * sched/sched/sched_sporadic.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>

#include <arch/irq.h>

#include "sched/sched.h"

#ifdef CONFIG_SCHED_SPORADIC

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Name: sched_sporadic_replenish_start
 *
 * Description:
 *   Start the next replenishment cycle, increasing the priority of the
 *   thread to the high priority.  This is normally a pretty trivial
 *   operation.  But we do have to take a few precautions is priority
 *   inheritance is enabled.
 *
 * Parameters:
 *   tcb - TCB of the thread whose priority is being boosted.
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ************************************************************************/

static int sched_sporadic_replenish_start(FAR struct tcb_s *tcb)
{
  int ret;

  /* Start the next replenishment interval */

  tcb->timeslice = tcb->budget;
#ifdef __REVISIT_REPLENISHMENTS
  tcb->nrepl = tcb->max_repl;
#endif

#ifdef CONFIG_PRIORITY_INHERITANCE
  /* If the priority was boosted above the higher priority, than just
   * reset the base priority.
   */

  if (tcb->sched_priority > tcb->base_priority)
    {
      /* Boosted... Do we still need to reprioritize? */

      if (tcb->hi_priority < tcb->base_priority)
        {
          /* No.. the current execution priority is lower than the
           * boosted priority.  Just reset the base priority.
           */

          tcb->base_priority = tcb->hi_priority;
          return OK;
        }
    }

  /* The thread priority has not been boosted or it has been boosted to a
   * lower priority than the high priority.  So, in either case, we need to
   * reset the priority.
   */

#endif

  /* Then reprioritize to the higher priority */

  ret = sched_reprioritize(tcb, tcb->hi_priority);
  if (ret < 0)
    {
      return -get_errno();
    }

  return OK;
}

/************************************************************************
 * Name: sched_sporadic_expire
 *
 * Description:
 *   Handles the expiration of a replenishment interval by starting the
 *   next replenishment interval.
 *
 * Parameters:
 *   Standard watchdog parameters
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The thread is still running and is still using the sporadic
 *   scheduling policy.
 *
 ************************************************************************/

static void sched_sporadic_expire(int argc, wdparm_t arg1, ...)
{
  FAR struct tcb_s *tcb = (FAR struct tcb_s *)arg1;

  DEBUGASSERT(argc == 1 && tcb != NULL);

  /* Start the next replenishment interval */

  DEBUGVERIFY(sched_sporadic_replenish_start(tcb));
}

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: sched_sporadic_start
 *
 * Description:
 *   Called to initialize sporadic scheduling on a given thread.  This
 *   function is called in the following circumstances:
 *
 *     - When starting a pthread with sporadic scheduling specified in
 *       the pthread attributes.
 *     - When establishing sporadic scheduling policy via 
 *       sched_setscheduler()
 *     - When the sporadic scheduling parameters are changed via
 *       sched_setparam().
 *
 * Parameters:
 *   tcb - The TCB of the thread that is beginning sporadic scheduling.
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - All sporadic scheduling parameters in the TCB are valid
 *   - The thread is not currently using the sporadic scheduliing policy.
 *
 ************************************************************************/

int sched_sporadic_start(FAR struct tcb_s *tcb)
{
  DEBUGASSERT(tcb);

  /* Cancel and pending low-priority interval timing and re-initialize
   * the watchdog timer.
   */

  wd_cancel(&tcb->low_dog);
  memset(&tcb->low_dog, 0, sizeof(struct wdog_s));

  /* Then start the first replenishment interval */

  return sched_sporadic_replenish_start(tcb);
}

/************************************************************************
 * Name: sched_sporadic_stop
 *
 * Description:
 *   Called to terminate sporadic scheduling on a given thread.  This
 *   function is called in the following circumstances:
 *
 *     - When any thread exits with sporadic scheduling active.
 *     - When any thread using sporadic scheduling is changed to use
 *       some other scheduling policy via sched_setscheduler()
 *     - When the sporadic scheduling parameters are changed via
 *       sched_setparam().
 *
 * Parameters:
 *   tcb - The TCB of the thread that is beginning sporadic scheduling.
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - All sporadic scheduling parameters in the TCB are valid
 *   - The thread is currently using the sporadic scheduling policy.
 *
 ************************************************************************/

int sched_sporadic_stop(FAR struct tcb_s *tcb)
{
  DEBUGASSERT(tcb);

  /* Cancel and pending low-priority interval timing and re-initialize
   * the watchdog timer.
   */

  wd_cancel(&tcb->low_dog);
  memset(&tcb->low_dog, 0, sizeof(struct wdog_s));

  /* Reset sporadic scheduling parameters */

  tcb->hi_priority  = 0;
  tcb->low_priority = 0;
#ifdef __REVISIT_REPLENISHMENTS
  tcb->max_repl     = 0;
  tcb->nrepl        = 0;
#endif
  tcb->timeslice    = 0;
  tcb->repl_period  = 0;
  tcb->budget       = 0;
  return OK;
}

/************************************************************************
 * Name: sched_sporadic_resume
 *
 * Description:
 *   Called to start the next replenishment interval.  This function is
 *   called in the following circumstances:
 *
 *     - From up_unblocktask() via sched_resume_scheduler() when a task
 *       using the sporadic scheduling policy.
 *
 *   This function does nothing if the budget phase as already elapsed or
 *   the maximum numer of replenishments have already been performed.
 *
 * Parameters:
 *   tcb - The TCB of the thread that is beginning sporadic scheduling.
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 * Assumptions:
 *  - Interrupts are disabled
 *  - All sporadic scheduling parameters in the TCB are valid
 *  - The low priority interval timer is not running
 *
 ************************************************************************/

int sched_sporadic_resume(FAR struct tcb_s *tcb)
{
  DEBUGASSERT(tcb);

  /* REVISIT: This logic is wrong.  In order to correctly implement
   * replenishments, we would need to add:  (1) logic to keep more
   * accurate accounting of the expended budget execution time, and (2)
   * multiple timers to handle the nested replenishment intervals.
   *
   * The logic here works as is but effective max_repl == 1.
   */

#ifdef __REVISIT_REPLENISHMENTS
  /* Make sure that we are in the budget portion of the replenishment
   * interval.  We know this is the case if the current timeslice is
   * non-zero.  Do not exceed the maximum number of replenishments.
   */

  if (tcb->timeslice > 0 && tcb->nrepl > 0)
    {
      tcb->timeslice = tcb->budget;
      tcb->nrepl--;
    }
#endif

  return OK;
}

/************************************************************************
 * Name: sched_sporadic_process
 *
 * Description:
 *   Process the elapsed time interval. Called from this context:
 *
 *   - From the timer interrupt handler while the thread with sporadic
 *     scheduling is running.
 *
 * Parameters:
 *   tcb - The TCB of the thread that is beginning sporadic scheduling.
 *   ticks - The number of elapsed ticks since the last time this
 *   function was called.
 *
 * Returned Value:
 *   The number if ticks remaining until the budget interval expires.
 *   Zero is returned if we are in the low-prioriy phase of the the
 *   replenishment interval.
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - All sporadic scheduling parameters in the TCB are valid
 *
 ************************************************************************/

uint32_t sched_sporadic_process(FAR struct tcb_s *tcb, uint32_t ticks,
                                bool noswitches)
{
  DEBUGASSERT(tcb && ticks > 0);

  /* If we are in the low-priority phase of the replenishment interval,
   * then just return zero.
   *
   *   > 0: In high priority phase of interval
   *  == 0: In the low priority phase of the interval
   *   < 0: Stuck in the high priority phase with pre-emption locked.
   */

  if (tcb->timeslice == 0)
    {
      return 0;
    }

  /* Check if the the budget interval has elapse  If 'ticks' is greater
   * than the timeslice value, then we ignore any excess amount.
   *
   * 'ticks' should never be greater than the remaining timeslice.  We try
   * to handle that gracefully but it would be an error in the scheduling
   * if there ever were the case.
   *
   * Notice that in the case where were are stuck in the high priority
   * phase with scheduler locke, timeslice will by -1 and any value of
   * ticks will pass this test.
   */

  if (ticks >= tcb->timeslice)
    {
      /* Does the thread have the scheduler locked? */

      if (tcb->lockcount > 0)
        {
          /* Yes... then we have no option but to give the thread more
           * time at the higher priority.  Dropping the priority could
           * result in a context switch.
           *
           * Set the timeslice value to a negative value to indicate this
           * case.
           */

          tcb->timeslice = -1;
          return 0;
        }
      
      /* We will also suppress context switches if we were called via one of
       * the unusual cases handled by sched_timer_reasses(). In that case,
       * we will return a value of one so that the timer will expire as soon
       * as possible and we can perform this action in the normal timer
       * expiration context.
       *
       * This is kind of kludge, but I am not to concerned because I hope
       * that the situation is impossible or at least could only occur on
       * rare corner-cases.
       */

      if (noswitches)
        {
          tcb->timeslice = -1;
          return 1;
        }

      /* Another possibility is the the budget interval is equal to the
       * entire replenishment interval.  This would seem like such a good
       * thing to do, but is certainly permitted.
       */

      if (tcb->budget >= tcb->repl_period)
        {
          tcb->timeslice = tcb->budget;
          return tcb->budget;
        }

      /* Otherwise enter the low-priority phase of the replenishment cycle */

      sched_sporadic_lowpriority(tcb);
      return 0;
    }

  /* No.. then just decrement the time remaining in the budget interval
   * and continue.
   */

  else
    {
      tcb->timeslice -= ticks;
      return tcb->timeslice;
    }
}

/************************************************************************
 * Name: sched_sporadic_lowpriority
 *
 * Description:
 *   Drop to the lower priority for the duration of the replenishment
 *   period. Called from:
 *
 *   - sched_sporadic_process() when the thread budget expires
 *   - sched_unlock().  When the budget expires while the thread had the
 *     scheduler locked.
 *
 * Parameters:
 *   tcb - The TCB of the thread that is entering the low priority phase. 
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - All sporadic scheduling parameters in the TCB are valid
 *
 ************************************************************************/

void sched_sporadic_lowpriority(FAR struct tcb_s *tcb)
{
  DEBUGASSERT(tcb);

  /* Enter the low-priority phase of the replenishment cycle */

  tcb->timeslice = 0;

  /* Start the timer that will terminate the low priority cycle.  This timer
   * expiration is independent of what else may occur (except that it must
   * be cancelled if the thread exits.
   */

  DEBUGVERIFY(wd_start(&tcb->low_dog, tcb->repl_period - tcb->budget,
                       sched_sporadic_expire, 1, (wdentry_t)tcb));

#ifdef CONFIG_PRIORITY_INHERITANCE
  /* If the priority was boosted above the higher priority, than just
   * reset the base priority.
   */

  if (tcb->sched_priority > tcb->base_priority)
    {
      /* Thread priority was boosted while we were in the high priority
       * state.
       */

      tcb->base_priority = tcb->low_priority;
    }
#endif

  /* Otherwise drop the priority of thread, possible causing a context
   * switch.
   */

  DEBUGVERIFY(sched_reprioritize(tcb, tcb->low_priority));
}

#endif /* CONFIG_SCHED_SPORADIC */
