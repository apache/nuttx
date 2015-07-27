/****************************************************************************
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/clock.h>

#include <arch/irq.h>

#include "clock/clock.h"
#include "sched/sched.h"

#ifdef CONFIG_SCHED_SPORADIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int sporadic_budget_start(FAR struct tcb_s *tcb,
  FAR struct replenishment_s *repl, uint32_t budget);
static int sporadic_budget_next(FAR struct replenishment_s *repl);
static int sporadic_interval_start(FAR struct replenishment_s *repl);
static void sporadic_budget_expire(int argc, wdparm_t arg1, ...);
static void sporadic_interval_expire(int argc, wdparm_t arg1, ...);
FAR struct replenishment_s *
  sporadic_alloc_repl(FAR struct sporadic_s *sporadic);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sporadic_budget_start
 *
 * Description:
 *   Start the next replenishment cycle by (1) increasing the priority of
 *   the thread to the high priority and (2) setting up a timer for the
 *   budgeted portion of the replenishment interval. We do have to take a
 *   few precautions is priority inheritance is enabled.
 *
 * Input Parameters:
 *   tcb    - TCB of the thread whose priority is being boosted.
 *   repl   - The replenishment timer to use
 *   budget - Budgeted execution time
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sporadic_budget_start(FAR struct tcb_s *tcb,
                                 FAR struct replenishment_s *repl,
                                 uint32_t budget)
{
  FAR struct sporadic_s *sporadic;
  int ret;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
  /* Inform the monitor of this event */

  arch_sporadic_start(tcb);
#endif

  /* Start the next replenishment interval */

  tcb->timeslice = budget;
  repl->budget   = budget;

#ifdef CONFIG_SCHED_TICKLESS
  /* Save the time that the replenishment interval started */

  (void)up_timer_gettime(&sporadic->sched_time);
#endif

  /* And start the timer for the budget interval */

  ret = wd_start(&repl->timer, budget, sporadic_budget_expire, 1,
                 (wdentry_t)repl);

#ifdef CONFIG_PRIORITY_INHERITANCE
  /* If the priority was boosted above the higher priority, than just
   * reset the base priority.
   */

  if (tcb->sched_priority > tcb->base_priority)
    {
      /* Boosted... Do we still need to reprioritize? */

      if (sporadic->hi_priority < sporadic->base_priority)
        {
          /* No.. the current execution priority is lower than the
           * boosted priority.  Just reset the base priority.
           */

          tcb->base_priority = sporadic->hi_priority;
          return OK;
        }
    }

  /* The thread priority has not been boosted or it has been boosted to a
   * lower priority than the high priority.  So, in either case, we need to
   * reset the priority.
   */

#endif

  /* Then reprioritize to the higher priority */

  ret = sched_reprioritize(tcb, sporadic->hi_priority);
  if (ret < 0)
    {
      int errcode = get_errno();
      slldbg("ERROR: sched_reprioritize failed: %d\n", errcode);
      return -errcode;
    }

  return OK;
}

/****************************************************************************
 * Name: sporadic_budget_next
 *
 * Description:
 *   Start the next replenishment.  This is called at the complete of each
 *   replenishment interval.
 *
 * Input Parameters:
 *   repl - Replenishment structure whose budget timer just expired.
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sporadic_budget_next(FAR struct replenishment_s *repl)
{
  FAR struct sporadic_s *sporadic;
  FAR struct tcb_s *tcb;
  uint32_t budget;

  DEBUGASSERT(repl != NULL && repl->tcb != NULL);
  tcb      = repl->tcb;
  sporadic = tcb->sporadic;
  DEBUGASSERT(sporadic != NULL);

  /* The budgeted interval will be the pending budget */

  budget = sporadic->pending;
  if (budget > sporadic->budget)
    {
      /* Clip to the maximum */

      budget = sporadic->budget;
    }

  sporadic->pending -= budget;

  /* If the budget zero, then all of the pending budget has been utilized.
   * There are now two possibilities:  (1) There are multiple, active
   * replenishment threads so this one is no longer needed, or (2) there is
   * only one and we need to restart the budget interval with the full
   * budget.
   */

   if (budget == 0)
     {
       if (sporadic->nrepls > 1)
         {
           /* Release this replenishment timer.  Processing will continue
            * to be driven by the remaining timers.
            */

           repl->active = false;
           sporadic->nrepls--;
           return OK;
         }
       else
         {
           /* No, we are the only replenishment timer.  Restart with the
            * full budget.
            */

           budget = sporadic->budget;
         }
     }

  /* Start the next replenishment interval */

  return sporadic_budget_start(tcb, repl, budget);
}

/****************************************************************************
 * Name: sporadic_set_lowpriority
 *
 * Description:
 *   Force the thread to lower priority.
 *
 * Input Parameters:
 *   repl - Replenishment timer to be used
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sporadic_set_lowpriority(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  int ret;

  DEBUGASSERT(tcb != NULL && tcb->sporadic != NULL);
  sporadic = tcb->sporadic;

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
  /* Inform the monitor of this event */

  arch_sporadic_lowpriority(tcb);
#endif

#ifdef CONFIG_PRIORITY_INHERITANCE
  /* If the priority was boosted above the higher priority, than just
   * reset the base priority.
   */

  if (tcb->sched_priority > tcb->base_priority)
    {
      /* Thread priority was boosted while we were in the high priority
       * state.
       */

      tcb->base_priority = sporadic->low_priority;
    }
#endif

  /* Otherwise drop the priority of thread, possible causing a context
   * switch.
   */

  ret = sched_reprioritize(tcb, sporadic->low_priority);
  if (ret < 0)
    {
      int errcode = get_errno();
      slldbg("ERROR: sched_reprioritize failed: %d\n", errcode);
      return -errcode;
    }

  return OK;
}

/****************************************************************************
 * Name: sporadic_interval_start
 *
 * Description:
 *   Start the next replenishment cycle, increasing the priority of the
 *   thread to the high priority.  This is normally a pretty trivial
 *   operation.  But we do have to take a few precautions is priority
 *   inheritance is enabled.
 *
 * Input Parameters:
 *   repl - Replenishment timer to be used
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sporadic_interval_start(FAR struct replenishment_s *repl)
{
  FAR struct sporadic_s *sporadic;
  FAR struct tcb_s *tcb;
  uint32_t remainder;

  DEBUGASSERT(repl != NULL && repl->tcb != NULL);
  tcb      = repl->tcb;
  sporadic = tcb->sporadic;
  DEBUGASSERT(sporadic != NULL);

  /* Enter the low-priority phase of the replenishment cycle */

  tcb->timeslice    = 0;

  /* Calculate the remainder of the replenishment interval.  This is
   * permitted to be zero, in which case we just restart the budget
   * interval without lowering the priority.
   */

  DEBUGASSERT(sporadic->repl_period >= repl->budget);
  remainder = sporadic->repl_period - repl->budget;
  if (remainder == 0)
    {
      return sporadic_budget_next(repl);
    }

  /* Start the timer that will terminate the low priority cycle.  This timer
   * expiration is independent of what else may occur (except that it must
   * be cancelled if the thread exits.
   */

  DEBUGVERIFY(wd_start(&repl->timer, remainder, sporadic_interval_expire,
              1, (wdentry_t)repl));

  /* Drop the priority of thread, possible causing a context switch. */

  return sporadic_set_lowpriority(tcb);
}

/****************************************************************************
 * Name: sporadic_budget_expire
 *
 * Description:
 *   Handles the expiration of a budget interval.  It saves the budget
 *   For the next interval, drops the priority of the thread, and restarts
 *   the timer for the rest of the replenishment period.
 *
 * Input Parameters:
 *   Standard watchdog parameters
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The thread is still running and is still using the sporadic
 *   scheduling policy.
 *
 ****************************************************************************/

static void sporadic_budget_expire(int argc, wdparm_t arg1, ...)
{
  FAR struct replenishment_s *repl = (FAR struct replenishment_s *)arg1;
  FAR struct sporadic_s *sporadic;
  FAR struct tcb_s *tcb;
  uint32_t pending;

  DEBUGASSERT(argc == 1 && repl != NULL && repl->tcb != NULL);
  tcb      = repl->tcb;

  sporadic = tcb->sporadic;
  DEBUGASSERT(sporadic != NULL);

  /* As a special case, we can do nothing here if schedule has been locked.
   * We cannot drop the priority because that might cause a context switch,
   * violating the lock.
   *
   * What we do instead is just deallocate the timer.  When the lock is
   * finally released, sched_sporadic_lowpriority() and that will restart
   * the interval period. timeslice == -1 is the cue to sched_unlock() that
   * this operation is needed.
   */

  if (tcb->lockcount > 0)
    {
      DEBUGASSERT(repl->active && sporadic->nrepls > 0);

      tcb->timeslice = -1;
      repl->budget   = 0;
      repl->active   = false;
      sporadic->nrepls--;
      return;
    }

  /* Calculate any part of the budget that was not utilized.
   *
   *   current   = The initial budget at the beginning of the interval.
   *   timeslice = The unused part of that budget when the thread did
   *               not execute.
   */

  DEBUGASSERT(repl->budget >= tcb->timeslice);
  pending = sporadic->pending + repl->budget - tcb->timeslice;
  if (pending > sporadic->budget)
    {
      /* Limit to the full budget.  This can happen if we are falling
       * behind and the thread is not getting enough CPU bandwidth to
       * satisfy its budget.
       */

      pending = sporadic->budget;
    }

  sporadic->pending = pending;

  /* Drop the priority of the thread and start the timer for the rest of
   * the interval.
   */

  DEBUGVERIFY(sporadic_interval_start(repl));
}

/****************************************************************************
 * Name: sporadic_interval_expire
 *
 * Description:
 *   Handles the expiration of a replenishment interval by starting the
 *   next replenishment interval.
 *
 * Input Parameters:
 *   Standard watchdog parameters
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   The thread is still running and is still using the sporadic
 *   scheduling policy.
 *
 ****************************************************************************/

static void sporadic_interval_expire(int argc, wdparm_t arg1, ...)
{
  FAR struct replenishment_s *repl = (FAR struct replenishment_s *)arg1;

  DEBUGASSERT(argc == 1 && repl != NULL);

  /* Start the next replenishment interval */

  DEBUGVERIFY(sporadic_budget_next(repl));
}

/****************************************************************************
 * Name: sporadic_timer_cancel
 *
 * Description:
 *   Cancel all timers. We do this if/when the budget time expires while the
 *   scheduler is locked.  Basically we need to stop everything and restart
 *   when the scheduler is unlocked.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that has the scheduler locked
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sporadic_timer_cancel(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  FAR struct replenishment_s *repl;
  int i;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

  /* Cancel all timers. */

  for (i = 0; i < CONFIG_SCHED_SPORADIC_MAXREPL; i++)
    {
      repl = &sporadic->replenishments[i];

      /* Cancel any outstanding timer activity */

      wd_cancel(&repl->timer);
      repl->active = false;
    }

  sporadic->nrepls = 0;
}

/****************************************************************************
 * Name: sporadic_alloc_repl
 *
 * Description:
 *   Allocate a replenishment timer structure.
 *
 * Input Parameters:
 *   sporadic - The task's sporadic scheduling state.
 *
 * Returned Value:
 *   The allocated replenishment timer structure; NULL is returned on a
 *   failure
 *
 ****************************************************************************/

FAR struct replenishment_s *
  sporadic_alloc_repl(FAR struct sporadic_s *sporadic)
{
  FAR struct replenishment_s *repl = NULL;
  int i;

  if (sporadic->nrepls < sporadic->max_repl)
    {
      /* Allocate a new replenishment timer */

      DEBUGASSERT(sporadic->max_repl <= CONFIG_SCHED_SPORADIC_MAXREPL);
      for (i = 0; i < sporadic->max_repl; i++)
        {
          FAR struct replenishment_s *tmp = &sporadic->replenishments[i];
          if (!tmp->active)
            {
              repl         = tmp;
              repl->active = true;
              sporadic->nrepls++;
              break;
            }
        }

      /* Since we no that we have not yet reached the max_repl number of
       * timers, the above search should never fail.
       */

      DEBUGASSERT(repl != NULL);
    }

  return repl;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_sporadic_initialize
 *
 * Description:
 *   Allocate resources needed by the sporadic scheduling policy.
 *
 * Input Parameters:
 *   tcb - TCB of the thread whose priority is being boosted.
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

int sched_sporadic_initialize(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  int i;

  DEBUGASSERT(tcb != NULL && tcb->sporadic == NULL);

  /* Allocate the sporadic add-on data structure that will hold the
   * sporadic scheduling parameters and state data.
   */

   sporadic = (FAR struct sporadic_s *)kmm_zalloc(sizeof(struct sporadic_s));
   if (sporadic == NULL)
     {
       slldbg("ERROR: Failed to allocate sporadic data structure\n");
       return -ENOMEM;
     }

   /* The initialize required is to set the back pointer to the TCB in
    * each of the replenishment structures.
    */

  for (i = 0; i < CONFIG_SCHED_SPORADIC_MAXREPL; i++)
    {
      sporadic->replenishments[i].tcb = tcb;
    }

  /* Hook the sporadic add-on into the TCB */

  tcb->sporadic = sporadic;
  return OK;
}

/****************************************************************************
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
 * Input Parameters:
 *   tcb - The TCB of the thread that is beginning sporadic scheduling.
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - All sporadic scheduling parameters in the TCB are valid
 *   - The thread is not currently using the sporadic scheduling policy.
 *
 ****************************************************************************/

int sched_sporadic_start(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  FAR struct replenishment_s *repl;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

  DEBUGASSERT(sporadic->low_priority <= sporadic->hi_priority);
  DEBUGASSERT(sporadic->max_repl <= CONFIG_SCHED_SPORADIC_MAXREPL);
  DEBUGASSERT(sporadic->budget > 0 && sporadic->budget <= sporadic->repl_period);
  DEBUGASSERT(sporadic->nrepls == 0 && sporadic->pending == 0);

  /* Allocate the first replenishment timer (should never fail) */

  repl = sporadic_alloc_repl(sporadic);
  DEBUGASSERT(repl != NULL && sporadic->nrepls == 1);

  /* Then start the first interval */

  return sporadic_budget_start(tcb, repl, sporadic->budget);
}

/****************************************************************************
 * Name: sched_sporadic_stop
 *
 * Description:
 *   Called to terminate sporadic scheduling on a given thread and to
 *   free all resources associated with the policy.  This function is
 *   called in the following circumstances:
 *
 *     - When any thread exits with sporadic scheduling active.
 *     - When any thread using sporadic scheduling is changed to use
 *       some other scheduling policy via sched_setscheduler()
 *
 * Input Parameters:
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
 ****************************************************************************/

int sched_sporadic_stop(FAR struct tcb_s *tcb)
{
  DEBUGASSERT(tcb && tcb->sporadic);

  /* Stop all timers, reset scheduling */

  (void)sched_sporadic_reset(tcb);

  /* The free the container holder the sporadic scheduling parameters */

  sched_kfree(tcb->sporadic);
  tcb->sporadic = NULL;
  return OK;
}

/****************************************************************************
 * Name: sched_sporadic_reset
 *
 * Description:
 *   Called to stop sporadic scheduling on a given thread.  This
 *   function is called in the following circumstances:
 *
 *     - When the sporadic scheduling parameters are changed via
 *       sched_setparam()
 *     - From sched_sporadic_stop when under those conditions.
 *
 * Input Parameters:
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
 ****************************************************************************/

int sched_sporadic_reset(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  FAR struct replenishment_s *repl;
  int i;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

  /* Cancel all timers. */

  for (i = 0; i < CONFIG_SCHED_SPORADIC_MAXREPL; i++)
    {
      repl = (FAR struct replenishment_s *)&sporadic->replenishments[i];

      /* Cancel any outstanding timer activity */

      wd_cancel(&repl->timer);

      /* Re-initialize replenishment data */

      repl->tcb = tcb;
    }

  /* Reset sporadic scheduling parameters and state data */

  sporadic->hi_priority  = 0;
  sporadic->low_priority = 0;
  sporadic->max_repl     = 0;
  sporadic->nrepls       = 0;
  sporadic->repl_period  = 0;
  sporadic->budget       = 0;
  sporadic->pending      = 0;
  return OK;
}

/****************************************************************************
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
 * Input Parameters:
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
 ****************************************************************************/

int sched_sporadic_resume(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  FAR struct replenishment_s *repl;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
  /* Inform the monitor of this event */

  arch_sporadic_resume(tcb);
#endif

  /* Check if are in the budget portion of the replenishment interval.  We
   * know this is the case if the current timeslice is non-zero.
   *
   * This function could also be called before the budget period has had a
   * chance to run, i.e., when the value of timeslice has not been decremented
   * and is still equal to the initial value.
   *
   * This latter case could occur if the thread has never had a chance to
   * run and is also very likely when the thread is just restarted after
   * raising its priority at the beginning of the budget period.  We would
   * not want to start a new timer in these cases.
   */

  if (tcb->timeslice > 0 && tcb->timeslice < sporadic->budget)
    {
      /* Allocate a new replenishment timer.  This will limit us to
       * to maximum number of replenishments (max_repl).
       */

      repl = sporadic_alloc_repl(sporadic);
      if (repl != NULL)
        {
          return sporadic_budget_start(tcb, repl, tcb->timeslice);
        }

      /* We need to return success even on a failure to allocate.  Doing
       * nothing is our fall-back behavior and that is not a failure from
       * the standpoint of higher level logic.
       */

      slldbg("Failed to allocate timer, nrepls=%d\n", sporadic->nrepls);
    }

#ifdef CONFIG_SCHED_TICKLESS
  /* Reset to the resume time */

  (void)up_timer_gettime(&sporadic->sched_time);
#endif

  return OK;
}

/****************************************************************************
 * Name: sched_sporadic_suspend
 *
 * Description:
 *   Called to when a thread with sporadic scheduling is suspended in the
 *   tickless mode.  In this case, there is unaccounted for time from the
 *   time that the last interval timer was started up until this point.
 *
 *   This function calculates the elpased time since then, and adjusts the
 *   timeslice time accordingly.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that is beginning sporadic scheduling.
 *   suspend_time - The time that the thread was suspended.
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 * Assumptions:
 *  - Interrupts are disabled
 *  - All sporadic scheduling parameters in the TCB are valid
 *  - The low priority interval timer is not running
 *
 ****************************************************************************/

#if defined(CONFIG_SCHED_SPORADIC) && (defined(CONFIG_SCHED_TICKLESS) || \
    defined(CONFIG_SPORADIC_INSTRUMENTATION))
int sched_sporadic_suspend(FAR struct tcb_s *tcb,
                           FAR const struct timespec *suspend_time)
{
#ifdef CONFIG_SCHED_TICKLESS
  FAR struct sporadic_s *sporadic;
  struct timespec elapsed_time;
  uint32_t elapsed_ticks;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;
#endif

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
  /* Inform the monitor of this event */

  arch_sporadic_suspend(tcb);
#endif

#ifdef CONFIG_SCHED_TICKLESS
  /* Check if are in the budget portion of the replenishment interval.  We
   * know this is the case if the current timeslice is non-zero.
   *
   * This function could also be called before the budget period has had a
   * chance to run, i.e., when the value of timeslice has not been decremented
   * and is still equal to the initial value.
   *
   * This latter case could occur if the thread has never had a chance to
   * run and is also very likely when the thread is just restarted after
   * raising its priority at the beginning of the budget period.  We would
   * not want to start a new timer in these cases.
   */

  if (tcb->timeslice > 0 && tcb->timeslice < sporadic->budget)
    {
      /* Get the difference in time between the time that the scheduler just
       * ran and time time that the thread was spuspended.  This difference,
       * then, is the unaccounted for time between the time that the timer
       * was started and when the thread was suspended.
       */

      clock_timespec_subtract(suspend_time, &sporadic->sched_time,
                              &elapsed_time);

      /* Convert to ticks */

      elapsed_ticks  = SEC2TICK(elapsed_time.tv_sec);
      elapsed_ticks += NSEC2TICK(elapsed_time.tv_nsec);

      /* One possibility is that the the elapsed time is greater than the
       * time remaining in the time slice.  This is a a race condition.  It
       * means that the timer has just expired.
       *
       * Ideally, we would need to cancel the budget time and start the low-
       * priority portion of the interval.  But we are in a precarious
       * situation here..  The TCB has been removed from the ready to run
       * list already so we cannot use the normal reprioritization logic.
       */

      if (elapsed_ticks >= tcb->timeslice)
        {
          /* REVISIT: The kludge here is to just set the clock to one tick.
           * That will cause the interval to expire very quickly.
           */

          tcb->timeslice = 1;
        }
      else
        {
          tcb->timeslice -= (int32_t)elapsed_ticks;
        }
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Name: sched_sporadic_process
 *
 * Description:
 *   Process the elapsed time interval. Called from this context:
 *
 *   - From the timer interrupt handler while the thread with sporadic
 *     scheduling is running.
 *
 * Input Parameters:
 *   tcb        - The TCB of the thread that is beginning sporadic
                  scheduling.
 *   ticks      - The number of elapsed ticks since the last time this
 *                function was called.
 *   noswitches - We are running in a context where context switching is
 *                not permitted.
 *
 * Returned Value:
 *   The number if ticks remaining until the budget interval expires.
 *   Zero is returned if we are in the low-priority phase of the the
 *   replenishment interval.
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - All sporadic scheduling parameters in the TCB are valid
 *
 ****************************************************************************/

uint32_t sched_sporadic_process(FAR struct tcb_s *tcb, uint32_t ticks,
                                bool noswitches)
{
  FAR struct sporadic_s *sporadic;

  DEBUGASSERT(tcb != NULL && tcb->sporadic != NULL && ticks > 0);

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
   * phase with scheduler locked, timeslice will by -1 and any value of
   * ticks will pass this test.
   */

  if (ticks >= tcb->timeslice)
    {
      /* Does the thread have the scheduler locked? */

      sporadic = tcb->sporadic;
      if (tcb->lockcount > 0)
        {
          /* Yes... then we have no option but to give the thread more
           * time at the higher priority.  Dropping the priority could
           * result in a context switch.
           *
           * Set the timeslice value to a negative value to indicate this
           * case.
           */

          sporadic_timer_cancel(tcb);
          tcb->timeslice    = -1;
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
          tcb->timeslice    = 1;
          return 1;
        }

      /* Another possibility is the the budget interval is equal to the
       * entire replenishment interval.  This would not seem like such a
       * good thing to do, but is certainly permitted.
       */

      if (sporadic->budget >= sporadic->repl_period)
        {
          tcb->timeslice = sporadic->budget;
          return sporadic->budget;
        }

      /* Otherwise enter the low-priority phase of the replenishment cycle */

      sporadic_set_lowpriority(tcb);
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

/****************************************************************************
 * Name: sched_sporadic_lowpriority
 *
 * Description:
 *   Drop to the lower priority for the duration of the replenishment
 *   period. Called from:
 *
 *   - sched_unlock().  When the budget expires while the thread had the
 *     scheduler locked.
 *
 * Input Parameters:
 *   tcb     - The TCB of the thread that is entering the low priority phase.
 *   restart - Restart timers.
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - All sporadic scheduling parameters in the TCB are valid
 *
 ****************************************************************************/

void sched_sporadic_lowpriority(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  FAR struct replenishment_s *repl;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

  /* Enter the low-priority phase of the replenishment cycle.  (This is
   * redundant).
   */

  tcb->timeslice = 0;

  /* Allocate a new replenishment timer.  There should be no timers
   * active at this phase since they were stopped in sched_sporadic_process().
   */

  DEBUGASSERT(sporadic->nrepls < sporadic->max_repl);
  repl = sporadic_alloc_repl(sporadic);
  DEBUGASSERT(repl != NULL);

  /* Drop the priority of thread, possible causing a context switch. */

  DEBUGVERIFY(sporadic_interval_start(repl));
}

#endif /* CONFIG_SCHED_SPORADIC */
