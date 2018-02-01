/****************************************************************************
 * sched/sched/sched_timerexpiration.c
 *
 *   Copyright (C) 2014-2016 Gregory Nutt. All rights reserved.
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
#include <nuttx/compiler.h>

#include <time.h>
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
 * Pre-processor Definitions
 ****************************************************************************/
/* In the original design, it was planned that sched_timer_reasses() be
 * called whenever there was a change at the head of the ready-to-run
 * list.  That call was intended to establish a new time-slice or to
 * stop an old time-slice timer.  However, it turns out that that
 * solution is too fragile:  The system is too vulnerable at the time
 * that the read-to-run list is modified in order to muck with timers.
 *
 * The kludge/work-around is simple to keep the timer running all of the
 * time with an interval of no more than the timeslice interval.  If we
 * this, then there is really no need to do anything when on context
 * switches.
 */

#define KEEP_ALIVE_HACK 1

#if CONFIG_RR_INTERVAL > 0
#  define KEEP_ALIVE_TICKS MSEC2TICK(CONFIG_RR_INTERVAL)
#else
#  define KEEP_ALIVE_TICKS MSEC2TICK(80)
#endif

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

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
static uint32_t sched_cpu_scheduler(int cpu, uint32_t ticks, bool noswitches);
#endif
#if CONFIG_RR_INTERVAL > 0 || defined(CONFIG_SCHED_SPORADIC)
static uint32_t sched_process_scheduler(uint32_t ticks, bool noswitches);
#endif
static unsigned int sched_timer_process(unsigned int ticks, bool noswitches);
static void sched_timer_start(unsigned int ticks);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is the duration of the currently active timer or, when
 * sched_timer_expiration() is called, the duration of interval timer
 * that just expired.  The value zero means that no timer was active.
 */

static unsigned int g_timer_interval;

#ifdef CONFIG_SCHED_SPORADIC
/* This is the time of the last scheduler assessment */

static struct timespec g_sched_time;
#endif

#ifdef CONFIG_SCHED_TICKLESS_ALARM
/* This is the time that the timer was stopped.  All future times are
 * calculated against this time.  It must be valid at all times when
 * the timer is not running.
 */

static struct timespec g_stop_time;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_cpu_scheduler
 *
 * Description:
 *   Check for operations specific to scheduling policy of the currently
 *   active task on a single CPU.
 *
 * Input Parameters:
 *   cpu - The CPU that we are performing the scheduler operations on.
 *   ticks - The number of ticks that have elapsed on the interval timer.
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
static uint32_t sched_cpu_scheduler(int cpu, uint32_t ticks, bool noswitches)
{
  FAR struct tcb_s *rtcb  = current_task(cpu);
  FAR struct tcb_s *ntcb  = current_task(cpu);
  uint32_t ret = 0;

#if CONFIG_RR_INTERVAL > 0
  /* Check if the currently executing task uses round robin scheduling. */

  if ((rtcb->flags & TCB_FLAG_POLICY_MASK) == TCB_FLAG_SCHED_RR)
    {
      /* Yes, check if the currently executing task has exceeded its
       * timeslice.
       */

      ret = sched_roundrobin_process(rtcb, ticks, noswitches);
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

      sporadic->sched_time.tv_sec  = g_sched_time.tv_sec;
      sporadic->sched_time.tv_nsec = g_sched_time.tv_nsec;

      /* Yes, check if the currently executing task has exceeded its
       * budget.
       */

      ret = sched_sporadic_process(rtcb, ticks, noswitches);
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

      return sched_process_scheduler(0, true);
    }

  /* Returning zero means that there is no interesting event to be timed */

#ifdef KEEP_ALIVE_HACK
  if (ret == 0)
    {
      /* Apply the keep alive hack */

      return KEEP_ALIVE_TICKS;
    }
#endif

  return ret;
}
#endif

/****************************************************************************
 * Name: sched_process_scheduler
 *
 * Description:
 *   Check for operations specific to scheduling policy of the currently
 *   active task on a single CPU.
 *
 * Input Parameters:
 *   ticks - The number of ticks that have elapsed on the interval timer.
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
static uint32_t sched_process_scheduler(uint32_t ticks, bool noswitches)
{
#ifdef CONFIG_SMP
  uint32_t minslice = UINT32_MAX;
  uint32_t timeslice;
  irqstate_t flags;
  int i;

  /* If we are running on a single CPU architecture, then we know interrupts
   * a disabled an there is no need to explicitly call
   * enter_critical_section().  However, in the SMP case,
   * enter_critical_section() does much more than just disable interrupts on
   * the local CPU; it also manages spinlocks to assure the stability of the
   * TCB that we are manipulating.
   */

  flags = enter_critical_section();

  /* Perform scheduler operations on all CPUs */

  for (i = 0; i < CONFIG_SMP_NCPUS; i++)
    {
      timeslice = sched_cpu_scheduler(i, ticks, noswitches);
      if (timeslice > 0 && timeslice < minslice)
        {
          minslice = timeslice;
        }
    }

  leave_critical_section(flags);
  return minslice < UINT32_MAX ? minslice : 0;

#else
  /* Perform scheduler operations on the single CPUs */

  return sched_cpu_scheduler(0, ticks, noswitches);
#endif
}
#else
#  define sched_process_scheduler(t,n) (0)
#endif

/****************************************************************************
 * Name:  sched_timer_process
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

static unsigned int sched_timer_process(unsigned int ticks, bool noswitches)
{
  unsigned int cmptime = UINT_MAX;
  unsigned int rettime  = 0;
  unsigned int tmp;

#ifdef CONFIG_CLOCK_TIMEKEEPING
  /* Process wall time */

  clock_update_wall_time();
#endif

  /* Process watchdogs */

  tmp = wd_timer(ticks);
  if (tmp > 0)
    {
      cmptime = tmp;
      rettime = tmp;
    }

  /* Check for operations specific to scheduling policy of the currently
   * active task.
   */

  tmp = sched_process_scheduler(ticks, noswitches);
  if (tmp > 0 && tmp < cmptime)
    {
      rettime  = tmp;
    }

  return rettime;
}

/****************************************************************************
 * Name:  sched_timer_start
 *
 * Description:
 *   Start the interval timer.
 *
 * Input Parameters:
 *   ticks - The number of ticks defining the timer interval to setup.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void sched_timer_start(unsigned int ticks)
{
#ifdef CONFIG_HAVE_LONG_LONG
  uint64_t usecs;
  uint64_t secs;
#else
  uint64_t usecs;
  uint64_t secs;
#endif
  uint32_t nsecs;
  int ret;

  /* Set up the next timer interval (or not) */

  g_timer_interval = 0;
  if (ticks > 0)
    {
      struct timespec ts;

#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
      if (ticks > g_oneshot_maxticks)
        {
          ticks = g_oneshot_maxticks;
        }
#endif

      /* Save new timer interval */

      g_timer_interval = ticks;

      /* Convert ticks to a struct timespec that up_timer_start() can
       * understand.
       *
       * REVISIT: Calculations may not have an acceptable range if uint64_t
       * is not supported(?)
       */

#ifdef CONFIG_HAVE_LONG_LONG
      usecs = TICK2USEC((uint64_t)ticks);
#else
      usecs = TICK2USEC(ticks);
#endif
      secs  = usecs / USEC_PER_SEC;
      nsecs = (usecs - (secs * USEC_PER_SEC)) * NSEC_PER_USEC;

      ts.tv_sec  = (time_t)secs;
      ts.tv_nsec = (long)nsecs;

#ifdef CONFIG_SCHED_TICKLESS_ALARM
      /* Convert the delay to a time in the future (with respect
       * to the time when last stopped the timer).
       */

      clock_timespec_add(&g_stop_time, &ts, &ts);
      ret = up_alarm_start(&ts);

#else
      /* [Re-]start the interval timer */

      ret = up_timer_start(&ts);
#endif

      if (ret < 0)
        {
          serr("ERROR: up_timer_start/up_alarm_start failed: %d\n");
          UNUSED(ret);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sched_alarm_expiration
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
void sched_alarm_expiration(FAR const struct timespec *ts)
{
  unsigned int elapsed;
  unsigned int nexttime;

  DEBUGASSERT(ts);

  /* Save the time that the alarm occurred */

  g_stop_time.tv_sec  = ts->tv_sec;
  g_stop_time.tv_nsec = ts->tv_nsec;

#ifdef CONFIG_SCHED_SPORADIC
  /* Save the last time that the scheduler ran */

  g_sched_time.tv_sec  = ts->tv_sec;
  g_sched_time.tv_nsec = ts->tv_nsec;
#endif

  /* Get the interval associated with last expiration */

  elapsed          = g_timer_interval;
  g_timer_interval = 0;

  /* Process the timer ticks and set up the next interval (or not) */

  nexttime = sched_timer_process(elapsed, false);
  sched_timer_start(nexttime);
}
#endif

/****************************************************************************
 * Name: sched_timer_expiration
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
void sched_timer_expiration(void)
{
  unsigned int elapsed;
  unsigned int nexttime;

  /* Get the interval associated with last expiration */

  elapsed          = g_timer_interval;
  g_timer_interval = 0;

#ifdef CONFIG_SCHED_SPORADIC
  /* Save the last time that the scheduler ran */

  (void)up_timer_gettime(&g_sched_time);
#endif

  /* Process the timer ticks and set up the next interval (or not) */

  nexttime = sched_timer_process(elapsed, false);
  sched_timer_start(nexttime);
}
#endif

/****************************************************************************
 * Name:  sched_timer_cancel
 *
 * Description:
 *   Stop the current timing activity.  This is currently called just before
 *   a new entry is inserted at the head of a timer list and also as part
 *   of the processing of sched_timer_reassess().
 *
 *   This function(1) cancels the current timer, (2) determines how much of
 *   the interval has elapsed, (3) completes any partially timed events
 *   (including updating the delay of the timer at the head of the timer
 *   list), and (2) returns the number of ticks that would be needed to
 *   resume timing and complete this delay.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Number of timer ticks that would be needed to complete the delay (zero
 *   if the timer was not active).
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS_ALARM
unsigned int sched_timer_cancel(void)
{
  struct timespec ts;
  unsigned int elapsed;

  /* Cancel the alarm and and get the time that the alarm was cancelled.
   * If the alarm was not enabled (or, perhaps, just expired since
   * interrupts were disabled), up_timer_cancel() will return the
   * current time.
   */

  ts.tv_sec        = g_stop_time.tv_sec;
  ts.tv_nsec       = g_stop_time.tv_nsec;
  g_timer_interval = 0;

  (void)up_alarm_cancel(&g_stop_time);

#ifdef CONFIG_SCHED_SPORADIC
  /* Save the last time that the scheduler ran */

  g_sched_time.tv_sec  = g_stop_time.tv_sec;
  g_sched_time.tv_nsec = g_stop_time.tv_nsec;
#endif

  /* Convert this to the elapsed time */

  clock_timespec_subtract(&g_stop_time, &ts, &ts);

  /* Convert to ticks */

  elapsed  = SEC2TICK(ts.tv_sec);
  elapsed += NSEC2TICK(ts.tv_nsec);

  /* Process the timer ticks and return the next interval */

  return sched_timer_process(elapsed, true);
}
#else
unsigned int sched_timer_cancel(void)
{
  struct timespec ts;
  unsigned int ticks;
  unsigned int elapsed;

  /* Get the time remaining on the interval timer and cancel the timer. */

  (void)up_timer_cancel(&ts);

#ifdef CONFIG_SCHED_SPORADIC
  /* Save the last time that the scheduler ran */

  g_sched_time.tv_sec  = ts.tv_sec;
  g_sched_time.tv_nsec = ts.tv_nsec;
#endif

  /* Convert to ticks */

  ticks  = SEC2TICK(ts.tv_sec);
  ticks += NSEC2TICK(ts.tv_nsec);
  DEBUGASSERT(ticks <= g_timer_interval);

  /* Handle the partial timer.  This will reassess all timer conditions and
   * re-start the interval timer with the correct delay.  Context switches
   * are not permitted in this case because we are not certain of the
   * calling conditions.
   */

  elapsed          = g_timer_interval - ticks;
  g_timer_interval = 0;

  /* Process the timer ticks and return the next interval */

  return sched_timer_process(elapsed, true);
}
#endif

/****************************************************************************
 * Name:  sched_timer_resume
 *
 * Description:
 *   Re-assess the next deadline and restart the interval timer.  This is
 *   called from wd_start() after it has inserted a new delay into the
 *   timer list.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 * Assumptions:
 *   This function is called right after sched_timer_cancel().  If
 *   CONFIG_SCHED_TICKLESS_ALARM=y, then g_stop_time must be the value time
 *   when the timer was cancelled.
 *
 ****************************************************************************/

void sched_timer_resume(void)
{
  unsigned int nexttime;

#ifdef CONFIG_SCHED_SPORADIC
  /* Save the last time that the scheduler ran */

  (void)up_timer_gettime(&g_sched_time);
#endif

  /* Reassess the next deadline (by simply processing a zero ticks expired)
   * and set up the next interval (or not).
   */

  nexttime = sched_timer_process(0, true);
  sched_timer_start(nexttime);
}

/****************************************************************************
 * Name:  sched_timer_reassess
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
 *   In the original design, it was also planned that sched_timer_reasses()
 *   be called whenever there was a change at the head of the ready-to-run
 *   list.  That call was intended to establish a new time-slice for the
 *   newly activated task or to stop the timer if time-slicing is no longer
 *   needed.  However, it turns out that that solution is too fragile:  The
 *   system is too vulnerable at the time that the read-to-run list is
 *   modified in order to muck with timers.
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
 ****************************************************************************/

void sched_timer_reassess(void)
{
  unsigned int nexttime;

  /* Cancel and restart the timer */

  nexttime = sched_timer_cancel();
  sched_timer_start(nexttime);
}

#endif /* CONFIG_SCHED_TICKLESS */
