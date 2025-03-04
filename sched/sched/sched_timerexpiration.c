/****************************************************************************
 * sched/sched/sched_timerexpiration.c
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
#include <nuttx/compiler.h>

#include <time.h>
#include <sys/param.h>
#include <assert.h>
#include <debug.h>

#if CONFIG_RR_INTERVAL > 0
#  include <sched.h>
#  include <nuttx/arch.h>
#endif

#include "sched/sched.h"
#include "wdog/wdog.h"
#include "clock/clock.h"

#ifdef CONFIG_CLOCK_TIMEKEEPING
#  include "clock/clock_timekeeping.h"
#endif

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
/* By default, the RTOS tickless logic assumes that range of times that can
 * be represented by the underlying hardware time is so large that no special
 * precautions need to taken.  That is not always the case.  If there is a
 * limit to the maximum timing interval that be represented by the timer,
 * then that limit must be respected.
 *
 * If CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP is defined, then a 32-bit global
 * variable called g_oneshot_maxticks variable is enabled. The variable
 * is initialized by platform-specific logic at runtime to the maximum delay
 * that the timer can wait (in microseconds).  The RTOS tickless logic will
 * then limit all requested delays to this value (in ticks).
 */

uint32_t g_oneshot_maxticks = UINT32_MAX;
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC)
static clock_t nxsched_cpu_scheduler(int cpu, clock_t ticks,
                                     clock_t elapsed, bool noswitches);
#endif
#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC)
static clock_t nxsched_process_scheduler(clock_t ticks, clock_t elapsed,
                                         bool noswitches);
#endif
static clock_t nxsched_timer_process(clock_t ticks, clock_t elapsed,
                                     bool noswitches);
static clock_t nxsched_timer_start(clock_t ticks, clock_t interval);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the tick that the timer was stopped.  All future times are
 * calculated against this time.  It must be valid at all times when
 * the timer is not running.
 */

static clock_t g_timer_tick;

/* This is the duration of the currently active timer or, when
 * nxsched_timer_expiration() is called, the duration of interval timer
 * that just expired.  The value zero means that no timer was active.
 */

static atomic_t g_timer_interval;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline_function clock_t  get_time_tick(void)
{
#ifdef CONFIG_SYSTEM_TIME64
  return atomic64_read((FAR atomic64_t *)&g_timer_tick);
#else
  return atomic_read((FAR atomic_t *)&g_timer_tick);
#endif
}

static inline_function clock_t update_time_tick(clock_t tick)
{
#ifdef CONFIG_SYSTEM_TIME64
  return atomic64_xchg((FAR atomic64_t *)&g_timer_tick, tick);
#else
  return atomic_xchg((FAR atomic_t *)&g_timer_tick, tick);
#endif
}

#if !defined(CONFIG_SCHED_TICKLESS_TICK_ARGUMENT) && !defined(CONFIG_CLOCK_TIMEKEEPING)
int up_timer_gettick(FAR clock_t *ticks)
{
  struct timespec ts;
  int ret;
  ret = up_timer_gettime(&ts);
  *ticks = clock_time2ticks(&ts);
  return ret;
}
#endif

#ifndef CONFIG_SCHED_TICKLESS_TICK_ARGUMENT
#  ifdef CONFIG_SCHED_TICKLESS_ALARM
int up_alarm_tick_start(clock_t ticks)
{
  struct timespec ts;
  clock_ticks2time(&ts, ticks);
  return up_alarm_start(&ts);
}

int up_alarm_tick_cancel(FAR clock_t *ticks)
{
  struct timespec ts;
  int ret;
  ret = up_alarm_cancel(&ts);
  *ticks = clock_time2ticks(&ts);
  return ret;
}
#  else
int up_timer_tick_start(clock_t ticks)
{
  struct timespec ts;
  clock_ticks2time(&ts, ticks);
  return up_timer_start(&ts);
}

int up_timer_tick_cancel(FAR clock_t *ticks)
{
  struct timespec ts;
  int ret;
  ret = up_timer_cancel(&ts);
  *ticks = clock_time2ticks(&ts);
  return ret;
}
#  endif
#endif

/****************************************************************************
 * Name:  nxsched_cpu_scheduler
 *
 * Description:
 *   Check for operations specific to scheduling policy of the currently
 *   active task on a single CPU.
 *
 * Input Parameters:
 *   cpu - The CPU that we are performing the scheduler operations on.
 *   elapsed - The number of ticks that have elapsed on the interval timer.
 *   noswitches - True: Can't do context switches now.
 *
 * Returned Value:
 *   The number if ticks remaining until the next time slice expires.
 *   Zero is returned if there is no time slicing (i.e., the task at the
 *   head of the ready-to-run list does not support round robin
 *   scheduling).
 *
 *   The value one may returned under certain circumstances that probably
 *   can't happen.  The value one is the minimal timer setup and it means
 *   that a context switch is needed now, but cannot be performed because
 *   noswitches == true.
 *
 ****************************************************************************/

#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC)
static clock_t nxsched_cpu_scheduler(int cpu, clock_t ticks,
                                     clock_t elapsed, bool noswitches)
{
  FAR struct tcb_s *rtcb = current_task(cpu);
  FAR struct tcb_s *ntcb = current_task(cpu);
  clock_t ret = 0;

#if CONFIG_RR_INTERVAL > 0
  /* Check if the currently executing task uses round robin scheduling. */

  if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_RR)
    {
      /* Yes, check if the currently executing task has exceeded its
       * timeslice.
       */

      ret = nxsched_process_roundrobin(rtcb, elapsed, noswitches);
    }
#endif

#ifdef CONFIG_SCHED_SPORADIC
  /* Check if the currently executing task uses sporadic scheduling. */

  if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_SPORADIC)
    {
      FAR struct sporadic_s *sporadic = rtcb->sporadic;
      DEBUGASSERT(sporadic);

      /* Save the last time that the scheduler ran.  This time was saved
       * higher in the calling hierarchy but cannot be applied until here.
       * That is because there are cases that context switches may occur
       * between then and before we get here.  So we can't positive of
       * which task TCB to save the time in until we are here and
       * committed to updating the scheduler for this TCB.
       */

      sporadic->eventtime = ticks;

      /* Yes, check if the currently executing task has exceeded its
       * budget.
       */

      ret = nxsched_process_sporadic(rtcb, elapsed, noswitches);
    }
#endif

  /* If a context switch occurred, then need to return delay remaining for
   * the new task at the head of the ready to run list.
   */

  ntcb = current_task(cpu);

  /* Check if the new task at the head of the ready-to-run has changed. */

  if (rtcb != ntcb)
    {
      /* Recurse just to get the correct return value */

      return nxsched_process_scheduler(ticks, 0, true);
    }

  /* Returning zero means that there is no interesting event to be timed */

  return ret;
}
#endif

/****************************************************************************
 * Name: nxsched_process_scheduler
 *
 * Description:
 *   Check for operations specific to scheduling policy of the currently
 *   active task on a single CPU.
 *
 * Input Parameters:
 *   ticks - The number of ticks that represent current time.
 *   elapsed - The number of ticks that have elapsed on the interval timer.
 *   noswitches - True: Can't do context switches now.
 *
 * Returned Value:
 *   The number if ticks remaining until the next time slice expires.
 *   Zero is returned if there is no time slicing (i.e., the task at the
 *   head of the ready-to-run list does not support round robin
 *   scheduling).
 *
 *   The value one may returned under certain circumstances that probably
 *   can't happen.  The value one is the minimal timer setup and it means
 *   that a context switch is needed now, but cannot be performed because
 *   noswitches == true.
 *
 ****************************************************************************/

#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC)
static clock_t nxsched_process_scheduler(clock_t ticks, clock_t elapsed,
                                         bool noswitches)
{
  clock_t minslice = CLOCK_MAX;
  clock_t timeslice;
  irqstate_t flags;
  int i;

  /* Single CPU case:
   * For nested interrupts, higher IRQs may interrupt nxsched_cpu_scheduler()
   * but nxsched_cpu_scheduler() requires that interrupts be disabled.
   * We are in ISR context, no meaning we are disabled the interrupts.
   *
   * SMP case:
   * enter_critical_section() does much more than just disable interrupts on
   * the local CPU; it also manages spinlocks to assure the stability of the
   * TCB that we are manipulating.
   */

  flags = enter_critical_section();

  /* Perform scheduler operations on all CPUs */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      timeslice = nxsched_cpu_scheduler(i, ticks, elapsed, noswitches);
      if (timeslice > 0 && timeslice < minslice)
        {
          minslice = timeslice;
        }
    }

  leave_critical_section(flags);
  return minslice < CLOCK_MAX ? minslice : 0;
}
#else
#  define nxsched_process_scheduler(t, e, n) (0)
#endif

/****************************************************************************
 * Name:  nxsched_timer_process
 *
 * Description:
 *   Process events on timer expiration.
 *
 * Input Parameters:
 *   ticks - The number of ticks that have elapsed on the interval timer.
 *   noswitches - True: Can't do context switches now.
 *
 * Returned Value:
 *   The number of ticks to use when setting up the next timer.  Zero if
 *   there is no interesting event to be timed.
 *
 ****************************************************************************/

static clock_t nxsched_timer_process(clock_t ticks, clock_t elapsed,
                                     bool noswitches)
{
  clock_t rettime = 0;
  clock_t tmp;

#ifdef CONFIG_CLOCK_TIMEKEEPING
  /* Process wall time */

  clock_update_wall_time();
#endif

  /* Check for operations specific to scheduling policy of the currently
   * active task.
   */

  tmp = nxsched_process_scheduler(ticks, elapsed, noswitches);
  if (tmp > 0)
    {
      rettime = tmp;
    }

  /* Process watchdogs */

  tmp = wd_timer(ticks, noswitches);
  if (tmp > 0 && (rettime == 0 || tmp < rettime))
    {
      rettime = tmp;
    }

  return rettime;
}

/****************************************************************************
 * Name:  nxsched_timer_start
 *
 * Description:
 *   Start the interval timer.
 *
 * Input Parameters:
 *   ticks - The number of ticks defining the timer interval to setup.
 *   interval - The number of ticks to use when setting up the next timer.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static clock_t nxsched_timer_start(clock_t ticks, clock_t interval)
{
  int ret;

  if (interval > 0)
    {
#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
      if (interval > g_oneshot_maxticks)
        {
          interval = g_oneshot_maxticks;
        }
#endif

#ifdef CONFIG_SCHED_TICKLESS_ALARM
      /* Convert the delay to a time in the future (with respect
       * to the time when last stopped the timer).
       */

      ret = up_alarm_tick_start(ticks + interval);
#else
      /* [Re-]start the interval timer */

      ret = up_timer_tick_start(interval);
#endif

      if (ret < 0)
        {
          serr("ERROR: up_timer_start/up_alarm_start failed: %d\n", ret);
          UNUSED(ret);
        }
    }

  return interval;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  nxsched_alarm_expiration
 *
 * Description:
 *   if CONFIG_SCHED_TICKLESS is defined, then this function is provided by
 *   the RTOS base code and called from platform-specific code when the
 *   alarm used to implement the tick-less OS expires.
 *
 * Input Parameters:
 *   ts - The time that the alarm expired
 *
 * Returned Value:
 *   None
 *
 * Assumptions/Limitations:
 *   Base code implementation assumes that this function is called from
 *   interrupt handling logic with interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS_ALARM
void nxsched_alarm_tick_expiration(clock_t ticks)
{
  clock_t elapsed;
  clock_t nexttime;

  /* Save the time that the alarm occurred */

  elapsed = ticks - update_time_tick(ticks);

  /* Process the timer ticks and set up the next interval (or not) */

  nexttime = nxsched_timer_process(ticks, elapsed, false);

  elapsed = nxsched_timer_start(ticks, nexttime);
  atomic_set(&g_timer_interval, elapsed);
}

void nxsched_alarm_expiration(FAR const struct timespec *ts)
{
  clock_t ticks;

  DEBUGASSERT(ts);

  ticks = clock_time2ticks(ts);
  nxsched_alarm_tick_expiration(ticks);
}
#endif

/****************************************************************************
 * Name: nxsched_timer_expiration
 *
 * Description:
 *   if CONFIG_SCHED_TICKLESS is defined, then this function is provided by
 *   the RTOS base code and called from platform-specific code when the
 *   interval timer used to implement the tick-less OS expires.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Base code implementation assumes that this function is called from
 *   interrupt handling logic with interrupts disabled.
 *
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS_ALARM
void nxsched_timer_expiration(void)
{
  clock_t ticks;
  clock_t elapsed;
  clock_t nexttime;

  /* Get the interval associated with last expiration */

  up_timer_gettick(&ticks);
  update_time_tick(ticks);
  elapsed = atomic_read(&g_timer_interval);

  /* Process the timer ticks and set up the next interval (or not) */

  nexttime = nxsched_timer_process(ticks, elapsed, false);

  elapsed = nxsched_timer_start(ticks, nexttime);
  atomic_set(&g_timer_interval, elapsed);
}
#endif

/****************************************************************************
 * Name:  nxsched_reassess_timer
 *
 * Description:
 *   It is necessary to re-assess the timer interval in several
 *   circumstances:
 *
 *   - If the watchdog at the head of the expiration list changes (or if its
 *     delay changes.  This can occur as a consequence of the actions of
 *     wd_start() or wd_cancel().
 *   - When pre-emption is re-enabled.  A previous time slice may have
 *     expired while pre-emption was enabled and now needs to be executed.
 *
 *   In the original design, it was also planned that
 *   nxsched_reassess_timer() be called whenever there was a change at the
 *   head of the ready-to-run list.  That call was intended to establish a
 *   new time-slice for the newly activated task or to stop the timer if
 *   time-slicing is no longer needed.  However, it turns out that that
 *   solution is too fragile:  The system is too vulnerable at the time
 *   that the ready-to-run list is modified in order to muck with timers.
 *
 *   The kludge/work-around is simple to keep the timer running all of the
 *   time with an interval of no more than the timeslice interval.  If we
 *   do this, then there is really no need to do anything on context
 *   switches.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Note:
 *   This function is called from the critical section
 *
 ****************************************************************************/

void nxsched_reassess_timer(void)
{
  clock_t nexttime;
  clock_t ticks;
  clock_t elapsed;

  /* Cancel the timer and get the current time */

#ifdef CONFIG_SCHED_TICKLESS_ALARM
  up_alarm_tick_cancel(&ticks);
#else
  up_timer_gettick(&ticks);
  up_timer_tick_cancel(&elapsed);
  DEBUGASSERT(elapsed <= atomic_read(&g_timer_interval));
#endif

  /* Convert this to the elapsed time and update clock tickbase */

  elapsed = ticks - update_time_tick(ticks);

  /* Process the timer ticks and start next timer */

  nexttime = nxsched_timer_process(ticks, elapsed, true);

  elapsed = nxsched_timer_start(ticks, nexttime);
  atomic_set(&g_timer_interval, elapsed);
}

/****************************************************************************
 * Name:  nxsched_get_next_expired
 *
 * Description:
 *   Get the time remaining until the next timer expiration.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The time remaining until the next timer expiration.
 *
 ****************************************************************************/

clock_t nxsched_get_next_expired(void)
{
  sclock_t ret;

  ret = get_time_tick() + atomic_read(&g_timer_interval) -
        clock_systime_ticks();

  return ret < 0 ? 0 : ret;
}

#endif /* CONFIG_SCHED_TICKLESS */
