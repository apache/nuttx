/****************************************************************************
 * sched/sched/sched_sporadic.c
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
#include <string.h>
#include <sys/param.h>
#include <assert.h>
#include <debug.h>
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
 * Private Function Prototypes
 ****************************************************************************/

/* Priority controls */

static int sporadic_set_lowpriority(FAR struct tcb_s *tcb);
static int sporadic_set_hipriority(FAR struct tcb_s *tcb);

/* State changes that initiate timer actions */

static int sporadic_budget_start(FAR struct replenishment_s *mrepl);
static int sporadic_interval_start(FAR struct replenishment_s *mrepl);
static int sporadic_replenish_start(FAR struct replenishment_s *repl);
static int sporadic_replenish_delay(FAR struct replenishment_s *repl,
  uint32_t period, uint32_t replenish);

/* Timer expiration handlers */

static void sporadic_budget_expire(wdparm_t arg);
static void sporadic_interval_expire(wdparm_t arg);
static void sporadic_replenish_expire(wdparm_t arg);
static void sporadic_delay_expire(wdparm_t arg);

/* Misc. helpers */

static void sporadic_timer_cancel(FAR struct tcb_s *tcb);
FAR struct replenishment_s *
  sporadic_alloc_repl(FAR struct sporadic_s *sporadic);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sporadic_set_lowpriority
 *
 * Description:
 *   Force the thread to lower priority.
 *
 * Input Parameters:
 *   tcb - TCB of task whose priority will be modified
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
   * reset the base priority and continue to run at the boosted priority.
   *
   * REVISIT:  There is a logic flaw here... If the priority was NOT
   * boosted above the hi_priority, then it still may still need to
   * boosted with respect to the lo_priority.  If the highest priority
   * thread waiting on a semaphore held by the sporadic thread is greater
   * than the low priority (but less than the hi_priority), the new
   * sched_priority should be set to that priority, not to the lo_priority
   *
   * In order to do this we would need to know the highest priority from
   * among all tasks waiting for the all semaphores held by the sporadic
   * task.  That information could be retained by the priority inheritance
   * logic of sem_holder.c for use here.
   */

  if (tcb->sched_priority > tcb->base_priority)
    {
      /* Thread priority was boosted while we were in the high priority
       * state.
       */

      tcb->base_priority = sporadic->low_priority;
    }
  else
#endif
    {
      /* Otherwise drop the priority of thread, possible causing a context
       * switch.
       */

      ret = nxsched_reprioritize(tcb, sporadic->low_priority);
      if (ret < 0)
        {
          serr("ERROR: nxsched_reprioritize failed: %d\n", ret);
          return ret;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: sporadic_set_hipriority
 *
 * Description:
 *   Force the thread to higher priority.
 *
 * Input Parameters:
 *   tcb - TCB of task whose priority will be modified
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sporadic_set_hipriority(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  int ret;

  DEBUGASSERT(tcb != NULL && tcb->sporadic != NULL);
  sporadic = tcb->sporadic;

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
  /* Inform the monitor of this event */

  arch_sporadic_start(tcb);
#endif

#ifdef CONFIG_PRIORITY_INHERITANCE
  /* If the priority was boosted above the higher priority, than just
   * reset the base priority.
   *
   * First, was the priority boosted above the lo_priority which should be
   * the same as the base_priority here? (This is an unnecessary test.
   * sched_priority > hi_priority would be sufficient).
   */

  if (tcb->sched_priority > tcb->base_priority)
    {
      /* Boosted... Do we still need to reprioritize?  If we were boosted to
       * a priority above the hi_priority then we do not need to do anything
       * except to adjust the base_priority
       *
       * REVISIT:  This logic is probably okay.  But may lead to problems
       * when the hi_priority is resumed.  See REVISIT comments in
       * sporadic_set_lowpriority().
       */

      if (tcb->sched_priority > sporadic->hi_priority)
        {
          /* No.. the new execution priority is lower than the boosted
           * priority.  Just reset the base priority.
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

  ret = nxsched_reprioritize(tcb, sporadic->hi_priority);
  if (ret < 0)
    {
      serr("ERROR: nxsched_reprioritize failed: %d\n", ret);
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: sporadic_budget_start
 *
 * Description:
 *   Start the next scheduler cycle by (1) increasing the priority of
 *   the thread to the high priority and (2) setting up a timer for the
 *   budgeted portion of the scheduler interval.
 *
 * Input Parameters:
 *   tcb    - TCB of the thread whose priority is being boosted.
 *   mrepl  - The main timer
 *   budget - Budgeted execution time
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sporadic_budget_start(FAR struct replenishment_s *mrepl)
{
  FAR struct sporadic_s *sporadic;
  FAR struct tcb_s *tcb;

  DEBUGASSERT(mrepl->tcb != NULL);
  tcb              = mrepl->tcb;
  sporadic         = tcb->sporadic;
  DEBUGASSERT(sporadic);

  /* Start the next replenishment interval */

  tcb->timeslice   = sporadic->budget;
  sporadic->active = mrepl;
  mrepl->budget    = sporadic->budget;
  mrepl->flags    &= ~SPORADIC_FLAG_REPLENISH;

  /* Save the time that the budget was started */

  sporadic->eventtime = clock_systime_ticks();

  /* And start the timer for the budget interval */

  DEBUGVERIFY(wd_start(&mrepl->timer, sporadic->budget,
                       sporadic_budget_expire, (wdparm_t)mrepl));

  /* Then reprioritize to the higher priority */

  return sporadic_set_hipriority(tcb);
}

/****************************************************************************
 * Name: sporadic_interval_start
 *
 * Description:
 *   Start the low-priority part of the next, normal cycle, decreasing the
 *   priority of the thread to the low priority.
 *
 * Input Parameters:
 *   mrepl - The main timer
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sporadic_interval_start(FAR struct replenishment_s *mrepl)
{
  FAR struct sporadic_s *sporadic;
  FAR struct tcb_s *tcb;
  uint32_t remainder;

  DEBUGASSERT(mrepl != NULL && mrepl->tcb != NULL);
  tcb      = mrepl->tcb;
  sporadic = tcb->sporadic;
  DEBUGASSERT(sporadic != NULL);

  /* Enter the low-priority phase of the replenishment cycle */

  tcb->timeslice   = 0;
  sporadic->active = mrepl;
  mrepl->flags    &= ~SPORADIC_FLAG_REPLENISH;

  /* Calculate the remainder of the replenishment interval.  This is
   * permitted to be zero, in which case we just restart the budget
   * interval without lowering the priority.
   */

  DEBUGASSERT(sporadic->repl_period >= sporadic->budget);
  remainder = sporadic->repl_period - sporadic->budget;
  if (remainder == 0)
    {
      /* Start the hi-priority portion of the next scheduler cycle */

      return sporadic_budget_start(mrepl);
    }

  /* Start the timer that will terminate the low priority cycle.  This timer
   * expiration is independent of what else may occur (except that it must
   * be canceled if the thread exits.
   */

  DEBUGVERIFY(wd_start(&mrepl->timer, remainder,
              sporadic_interval_expire, (wdparm_t)mrepl));

  /* Drop the priority of thread, possible causing a context switch. */

  return sporadic_set_lowpriority(tcb);
}

/****************************************************************************
 * Name: sporadic_replenish_start
 *
 * Description:
 *   Start the next replenishment by (1) increasing the priority of the
 *   thread to the high priority and (2) setting up a timer for the
 *   replenishment interval.
 *
 * Input Parameters:
 *   repl   - The replenishment timer to use
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sporadic_replenish_start(FAR struct replenishment_s *repl)
{
  FAR struct sporadic_s *sporadic;
  FAR struct tcb_s *tcb;

  DEBUGASSERT(repl->tcb != NULL);
  tcb              = repl->tcb;
  sporadic         = tcb->sporadic;
  DEBUGASSERT(sporadic);

  /* Start the replenishment interval */

  tcb->timeslice   = repl->budget;
  sporadic->active = repl;
  repl->flags     |= SPORADIC_FLAG_REPLENISH;

  /* And start the timer for the budget interval */

  DEBUGVERIFY(wd_start(&repl->timer, repl->budget,
                       sporadic_replenish_expire, (wdparm_t)repl));

  /* Then reprioritize to the higher priority */

  return sporadic_set_hipriority(tcb);
}

/****************************************************************************
 * Name: sporadic_replenish_delay
 *
 * Description:
 *   Start the delay prior to providing the replenishment.
 *
 * Input Parameters:
 *   repl      - Replenishment timer to be used
 *   period    - Delay to the timer expiration
 *   replenish - The replenish time to be applied after the delay
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sporadic_replenish_delay(FAR struct replenishment_s *repl,
                                    uint32_t period, uint32_t replenish)
{
  DEBUGASSERT(repl->tcb != NULL && repl->tcb->sporadic != NULL);

  /* Save information about the replenishment */

  repl->budget = replenish;
  repl->flags |= SPORADIC_FLAG_REPLENISH;

  /* And start the timer for the delay prior to replenishing. */

  DEBUGVERIFY(wd_start(&repl->timer, period,
                       sporadic_delay_expire, (wdparm_t)repl));
  return OK;
}

/****************************************************************************
 * Name: sporadic_budget_expire
 *
 * Description:
 *   Handles the expiration of a budget interval.  It saves the budget
 *   For the next interval, drops the priority of the thread, and restarts
 *   the timer for the rest of the replenishment period.
 *
 *   REVISIT: If there was no execution at all during the entire budget
 *   interval, we make no attempt to replenish that.  It is just lost
 *   execution time.
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

static void sporadic_budget_expire(wdparm_t arg)
{
  FAR struct replenishment_s *mrepl = (FAR struct replenishment_s *)arg;
  FAR struct replenishment_s *repl;
  FAR struct sporadic_s *sporadic;
  FAR struct tcb_s *tcb;

  DEBUGASSERT(mrepl != NULL && mrepl->tcb != NULL);
  tcb = mrepl->tcb;

  /* As a special case, we can do nothing here if scheduler has been locked.
   * We cannot drop the priority because that might cause a context switch,
   * violating the lock.
   *
   * What we do instead is just deallocate all timers.  When the lock is
   * finally released, nxsched_sporadic_lowpriority() and that will restart
   * the interval period. timeslice == -1 is the cue to sched_unlock() that
   * this operation is needed.
   */

  if (nxsched_islocked_tcb(tcb))
    {
      DEBUGASSERT((mrepl->flags & SPORADIC_FLAG_ALLOCED) != 0);

      /* Set the timeslice to the magic value */

      tcb->timeslice = -1;

      /* Cancel and free all timers */

      sporadic_timer_cancel(tcb);
      return;
    }

  /* Were we suspended at the end of the budget time?  If so, start a new
   * replenishment timer for the time since we were suspended (which might
   * be as long as the whole budget interval).
   */

  sporadic = tcb->sporadic;
  DEBUGASSERT(sporadic != NULL);

  if (sporadic->suspended)
    {
      uint32_t unrealized;

      /* The unrealized time is the interval from when the thread was
       * suspended (or when the budget interval was started in the case
       * that the thread was delayed for the entire interval).
       */

      unrealized = clock_systime_ticks() - sporadic->eventtime;
      if (unrealized > 0)
        {
          /* Allocate a new replenishment timer.  This will limit us to the
           * maximum number of replenishments (max_repl).
           */

          repl = sporadic_alloc_repl(sporadic);
          if (repl != NULL)
            {
              uint32_t period;

              /* Calculate the delay to when replenishment interval begins.
               * That time is one half of the scheduler cycle relative to
               * the suspend time.  The delay relative to the current time
               * is then:
               *
               *   repl_time = susp_time + repl_interval / 2;
               *   delay     = repl_time - curr_time
               *   delay     = susp_time - curr_time + repl_interval / 2
               *   delay     = repl_interval / 2 - unrealized
               */

              DEBUGASSERT(unrealized <= (sporadic->repl_period >> 1));
              period = (sporadic->repl_period >> 1) - unrealized;

              /* Start the delay into the next cycle to perform the
               * replenishment.
               */

              DEBUGVERIFY(sporadic_replenish_delay(repl, period,
                                                   unrealized));
            }
        }
    }

  /* Drop the priority of the thread and start the timer for the rest of the
   * scheduler interval.
   */

  DEBUGVERIFY(sporadic_interval_start(mrepl));
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

static void sporadic_interval_expire(wdparm_t arg)
{
  FAR struct replenishment_s *mrepl = (FAR struct replenishment_s *)arg;

  DEBUGASSERT(mrepl != NULL);

  /* If we get here, then (1) this should be the main thread, and (2) there
   * should be no active replenishment thread.
   */

  DEBUGASSERT((mrepl->flags & SPORADIC_FLAG_MAIN) != 0);

  /* Start the hi-priority portion of the next scheduler cycle */

  DEBUGVERIFY(sporadic_budget_start(mrepl));
}

/****************************************************************************
 * Name: sporadic_replenish_expire
 *
 * Description:
 *   Handles the expiration of a replenishment interval timer.
 *
 *   REVISIT: If there was no execution at all during the entire
 *   replenishment interval, we make no attempt to replenish the
 *   replenishment.  It is just lost execution time.
 *
 * Input Parameters:
 *   Standard watchdog parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sporadic_replenish_expire(wdparm_t arg)
{
  FAR struct replenishment_s *repl = (FAR struct replenishment_s *)arg;
  FAR struct sporadic_s *sporadic;
  FAR struct tcb_s *tcb;

  DEBUGASSERT(repl != NULL && repl->tcb != NULL);
  tcb      = repl->tcb;

  sporadic = tcb->sporadic;
  DEBUGASSERT(sporadic != NULL);

  /* This should not be the main timer */

  DEBUGASSERT((repl->flags & (SPORADIC_FLAG_MAIN | SPORADIC_FLAG_REPLENISH))
              == SPORADIC_FLAG_REPLENISH);

  /* As a special case, we can do nothing here if scheduler has been locked.
   * We cannot drop the priority because that might cause a context switch,
   * violating the lock.
   *
   * What we do instead is just deallocate all timers.  When the lock is
   * finally released, nxsched_sporadic_lowpriority() and that will restart
   * the interval period. timeslice == -1 is the cue to sched_unlock() that
   * this operation is needed.
   */

  if (nxsched_islocked_tcb(tcb))
    {
      /* Set the timeslice to the magic value */

      tcb->timeslice = -1;

      /* Cancel and free all timers */

      sporadic_timer_cancel(tcb);
      return;
    }

  /* Make sure that we are in synch with the timer process. */

  tcb->timeslice = 0;

  /* Discard the timer. It has served its purpose. */

  repl->flags = 0;
  sporadic->nrepls--;

  /* Drop the priority of thread, possible causing a context switch. */

  DEBUGVERIFY(sporadic_set_lowpriority(tcb));
}

/****************************************************************************
 * Name: sporadic_delay_expire
 *
 * Description:
 *   Handle the expiration of a delay prior to providing a replenishment.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that has the scheduler locked
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sporadic_delay_expire(wdparm_t arg)
{
  FAR struct replenishment_s *repl = (FAR struct replenishment_s *)arg;

  DEBUGASSERT(repl != NULL);

  /* Start the replenishment */

  DEBUGVERIFY(sporadic_replenish_start(repl));
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
      repl->flags = 0;
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
          if ((tmp->flags & SPORADIC_FLAG_ALLOCED) == 0)
            {
              repl        = tmp;
              repl->flags = SPORADIC_FLAG_ALLOCED;
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
 * Name: nxsched_initialize_sporadic
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

int nxsched_initialize_sporadic(FAR struct tcb_s *tcb)
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
      serr("ERROR: Failed to allocate sporadic data structure\n");
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
 * Name: nxsched_start_sporadic
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

int nxsched_start_sporadic(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  FAR struct replenishment_s *mrepl;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

  /* Allocate the main replenishment timer (should never fail) */

  mrepl = sporadic_alloc_repl(sporadic);
  DEBUGASSERT(mrepl != NULL && sporadic->nrepls == 1);
  mrepl->flags |= SPORADIC_FLAG_MAIN;

  /* Save the time that the scheduler was started */

  sporadic->eventtime = clock_systime_ticks();
  sporadic->suspended = true;

  /* Then start the first interval */

  return sporadic_budget_start(mrepl);
}

/****************************************************************************
 * Name: nxsched_stop_sporadic
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

int nxsched_stop_sporadic(FAR struct tcb_s *tcb)
{
  DEBUGASSERT(tcb && tcb->sporadic);

  /* Stop all timers, reset scheduling */

  nxsched_reset_sporadic(tcb);

  /* The free the container holder the sporadic scheduling parameters */

  kmm_free(tcb->sporadic);
  tcb->sporadic = NULL;
  return OK;
}

/****************************************************************************
 * Name: nxsched_reset_sporadic
 *
 * Description:
 *   Called to stop sporadic scheduling on a given thread.  This
 *   function is called in the following circumstances:
 *
 *     - When the sporadic scheduling parameters are changed via
 *       sched_setparam()
 *     - From nxsched_stop_sporadic when under those conditions.
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

int nxsched_reset_sporadic(FAR struct tcb_s *tcb)
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

      repl->tcb          = tcb;
      repl->budget       = 0;
      repl->flags        = 0;
    }

  /* Reset sporadic scheduling parameters and state data */

  sporadic->suspended    = true;
  sporadic->hi_priority  = 0;
  sporadic->low_priority = 0;
  sporadic->max_repl     = 0;
  sporadic->nrepls       = 0;
  sporadic->repl_period  = 0;
  sporadic->budget       = 0;
  sporadic->eventtime    = 0;
  sporadic->active       = NULL;
  return OK;
}

/****************************************************************************
 * Name: nxsched_resume_sporadic
 *
 * Description:
 *   Called to start the next replenishment interval.  This function is
 *   called in the following circumstances:
 *
 *     - From up_unblocktask() via nxsched_resume_scheduler() when a task
 *       using the sporadic scheduling policy.
 *
 *   This function does nothing if the budget phase as already elapsed or
 *   the maximum number of replenishments have already been performed.
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

int nxsched_resume_sporadic(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  FAR struct replenishment_s *repl;
  clock_t now;
  uint32_t unrealized;
  uint32_t last;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

  /* If we are resumed more than one, ignore all but the first */

  if (!sporadic->suspended)
    {
      return OK;
    }

  /* This is the first time */

  sporadic->suspended = false;

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
  /* Inform the monitor of this event */

  arch_sporadic_resume(tcb);
#endif

  /* Get the time that the thread was [re-]started */

  now = clock_systime_ticks();

  /* Check if are in the budget portion of the replenishment interval.  We
   * know this is the case if the current timeslice is non-zero.
   */

  if (tcb->timeslice > 0)
    {
      /* Unrealized budget time while the thread was suspended */

      unrealized = now - sporadic->eventtime;

      /* Ignore very short pre-emptions that are below
       * our timing resolution.
       */

      if (unrealized > 0)
        {
          /* Handle any part of the budget that was not utilized.
           *
           *   current    = The initial budget at the beginning of the
           *                interval.
           *   unrealized = The unused part of that budget when the
           *                thread did  not execute.
           */

          if (unrealized >= tcb->timeslice)
            {
              /* We lost the remainder of the timeslice, (and then some).
               * No point in starting more timers.
               */

              return OK;
            }

          DEBUGASSERT(sporadic->active);
          tcb->timeslice -= unrealized;

          /* Get the last budgeted time */

          last = sporadic->active->budget;

          /* This function could also be called before the budget period
           * has had a chance to run, i.e., when the value of timeslice
           * has not been decremented and is still equal to the initial
           * value.
           */

          if (tcb->timeslice < last)
            {
              /* Allocate a new replenishment timer.  This will limit us to
               * to maximum number of replenishments (max_repl).
               */

              repl = sporadic_alloc_repl(sporadic);
              if (repl != NULL)
                {
                  uint32_t period;

                  /* Calculate the delay to when replenishment interval
                   * begins.  That time is one half of the scheduler cycle
                   * relative to the suspend time.  The delay relative to
                   * the current time is then:
                   *
                   *   repl_time = susp_time + repl_interval / 2;
                   *   delay     = repl_time - curr_time
                   *   delay     = susp_time - curr_time + repl_interval / 2
                   *   delay     = repl_interval / 2 - unrealized
                   */

                  DEBUGASSERT(unrealized <= (sporadic->repl_period >> 1));
                  period = (sporadic->repl_period >> 1) - unrealized;

                  /* Start the delay into the next cycle to perform the
                   * replenishment.
                   */

                  return sporadic_replenish_delay(repl, period, unrealized);
                }

              /* We need to return success even on a failure to allocate.
               * Doing nothing is our fall-back behavior and that is not a
               * failure from the standpoint of higher level logic.
               */

              serr("ERROR: Failed to allocate timer, nrepls=%d\n",
                   sporadic->nrepls);
            }
        }
    }

  sporadic->eventtime = now;
  return OK;
}

/****************************************************************************
 * Name: nxsched_suspend_sporadic
 *
 * Description:
 *   Called when a thread with sporadic scheduling is suspended.  In this
 *   case, there will be unaccounted for time from the time that the last
 *   when the task is resumed.  All that we need to do here is remember
 *   that time that we were suspended.
 *
 * Input Parameters:
 *   tcb - The TCB of the thread that is beginning sporadic scheduling.
 *
 * Returned Value:
 *   Returns zero (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

int nxsched_suspend_sporadic(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

  /* If we are suspended more than one, ignore all but the first */

  if (!sporadic->suspended)
    {
      /* This is the first time */

      sporadic->suspended = true;

#ifdef CONFIG_SPORADIC_INSTRUMENTATION
      /* Inform the monitor of this event */

      arch_sporadic_suspend(tcb);
#endif

      /* Save the time that the thread was suspended */

      sporadic->eventtime = clock_systime_ticks();
    }

  return OK;
}

/****************************************************************************
 * Name: nxsched_process_sporadic
 *
 * Description:
 *   Process the elapsed time interval. Called from this context:
 *
 *   - From the timer interrupt handler while the thread with sporadic
 *     scheduling is running.
 *
 * Input Parameters:
 *   tcb        - The TCB of the thread that is beginning sporadic
 *                scheduling.
 *   ticks      - The number of elapsed ticks since the last time this
 *                function was called.
 *   noswitches - We are running in a context where context switching is
 *                not permitted.
 *
 * Returned Value:
 *   The number if ticks remaining until the budget interval expires.
 *   Zero is returned if we are in the low-priority phase of the
 *   replenishment interval.
 *
 * Assumptions:
 *   - Interrupts are disabled
 *   - All sporadic scheduling parameters in the TCB are valid
 *
 ****************************************************************************/

uint32_t nxsched_process_sporadic(FAR struct tcb_s *tcb, uint32_t ticks,
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

  /* Check if the budget interval has elapse  If 'ticks' is greater
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
      if (nxsched_islocked_tcb(tcb))
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
       * the unusual cases handled by nxsched_reassess_timer(). In that case,
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
          tcb->timeslice = 1;
          return 1;
        }

      /* Another possibility is the budget interval is equal to the
       * entire replenishment interval.  This would not seem like such a
       * good thing to do, but is certainly permitted.
       */

      if (sporadic->budget >= sporadic->repl_period)
        {
          tcb->timeslice = sporadic->budget;
          return sporadic->budget;
        }

      /* Otherwise enter the low-priority phase of the replenishment cycle.
       * Let the timers handle the priority changes.
       */

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
 * Name: nxsched_sporadic_lowpriority
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

void nxsched_sporadic_lowpriority(FAR struct tcb_s *tcb)
{
  FAR struct sporadic_s *sporadic;
  FAR struct replenishment_s *mrepl;

  DEBUGASSERT(tcb && tcb->sporadic);
  sporadic = tcb->sporadic;

  /* Enter the low-priority phase of the replenishment cycle.  (This is
   * redundant).
   */

  tcb->timeslice = 0;

  /* Allocate a new main timer.  There should be no timers active at this
   * phase since they were stopped in nxsched_process_sporadic().
   */

  DEBUGASSERT(sporadic->nrepls < sporadic->max_repl);
  mrepl = sporadic_alloc_repl(sporadic);
  DEBUGASSERT(mrepl != NULL);

  tcb->timeslice   = 0;
  sporadic->active = mrepl;

  mrepl->flags    |= SPORADIC_FLAG_MAIN;
  mrepl->budget    = sporadic->budget;

  /* Drop the priority of thread, possible causing a context switch. */

  DEBUGVERIFY(sporadic_interval_start(mrepl));
}

#endif /* CONFIG_SCHED_SPORADIC */
