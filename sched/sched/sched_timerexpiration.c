/************************************************************************
 * sched/sched/sched_timerexpiration.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#include <nuttx/compiler.h>

#include <time.h>
#include <assert.h>
#include <debug.h>

#if CONFIG_RR_INTERVAL > 0
# include <sched.h>
# include <nuttx/arch.h>
#endif

#include "sched/sched.h"
#include "wdog/wdog.h"
#include "clock/clock.h"

#ifdef CONFIG_SCHED_TICKLESS

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/
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

#if CONFIG_RR_INTERVAL > 0
#  define KEEP_ALIVE_HACK 1
#endif

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/

/************************************************************************
 * Private Variables
 ************************************************************************/
/* This is the duration of the currently active timer or, when 
 * sched_timer_expiration() is called, the duration of interval timer
 * that just expired.  The value zero means that no timer was active.
 */

static unsigned int g_timer_interval;

#ifdef CONFIG_SCHED_TICKLESS_ALARM
/* This is the time that the timer was stopped.  All future times are
 * calculated against this time.  It must be valid at all times when
 * the timer is not running.
 */

static struct timespec g_stop_time;
#endif

/************************************************************************
 * Private Functions
 ************************************************************************/
/************************************************************************
 * Name:  sched_timespec_add
 *
 * Description:
 *   Add timespec ts1 to to2 and return the result in ts3
 *
 * Inputs:
 *   ts1 and ts2: The two timespecs to be added
 *   t23: The location to return the result (may be ts1 or ts2)
 *
 * Return Value:
 *   None
 *
 ************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS_ALARM
static void sched_timespec_add(FAR const struct timespec *ts1,
                               FAR const struct timespec *ts2,
                               FAR struct timespec *ts3)
{
  time_t sec = ts1->tv_sec + ts2->tv_sec;
  long nsec  = ts1->tv_nsec + ts2->tv_nsec;

  if (nsec >= NSEC_PER_SEC)
    {
      nsec -= NSEC_PER_SEC;
      sec++;
    }

  ts3->tv_sec  = sec;
  ts3->tv_nsec = nsec;
}
#endif

/************************************************************************
 * Name:  sched_timespec_subtract
 *
 * Description:
 *   Subtract timespec ts2 from to1 and return the result in ts3.
 *   Zero is returned if the time difference is negative.
 *
 * Inputs:
 *   ts1 and ts2: The two timespecs to be subtracted (ts1 - ts2)
 *   t23: The location to return the result (may be ts1 or ts2)
 *
 * Return Value:
 *   None
 *
 ************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS_ALARM
static void sched_timespec_subtract(FAR const struct timespec *ts1,
                                    FAR const struct timespec *ts2,
                                    FAR struct timespec *ts3)
{
  time_t sec;
  long nsec;

  if (ts1->tv_sec < ts2->tv_sec)
    {
      sec  = 0;
      nsec = 0;
    }
  else if (ts1->tv_sec == ts2->tv_sec && ts1->tv_nsec <= ts2->tv_nsec)
    {
      sec  = 0;
      nsec = 0;
    }
  else
    {
      sec = ts1->tv_sec + ts2->tv_sec;
      if (ts1->tv_nsec < ts2->tv_nsec)
        {
          nsec = (ts1->tv_nsec + NSEC_PER_SEC) - ts2->tv_nsec;
          sec--;
        }
      else
        {
          nsec = ts1->tv_nsec - ts2->tv_nsec;
        }
    }

  ts3->tv_sec = sec;
  ts3->tv_nsec = nsec;
}
#endif

/************************************************************************
 * Name:  sched_process_timeslice
 *
 * Description:
 *   Check if the currently executing task has exceeded its time slice.
 *
 * Inputs:
 *   ticks - The number of ticks that have elapsed on the interval timer.
 *   noswitches - True: Can't do context switches now.
 *
 * Return Value:
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
 ************************************************************************/

#if CONFIG_RR_INTERVAL > 0
static unsigned int
sched_process_timeslice(unsigned int ticks, bool noswitches)
{
  FAR struct tcb_s *rtcb  = (FAR struct tcb_s*)g_readytorun.head;
#ifdef KEEP_ALIVE_HACK
  unsigned int ret = MSEC2TICK(CONFIG_RR_INTERVAL);
#else
  unsigned int ret = 0;
#endif
  int decr;

  /* Check if the currently executing task uses round robin
   * scheduling.
   */

  if ((rtcb->flags & TCB_FLAG_ROUND_ROBIN) != 0)
    {
      /* Now much can we decrement the timeslice delay?  If 'ticks'
       * is greater than the timeslice value, then we ignore any
       * excess amount.
       *
       * 'ticks' should never be greater than the remaining timeslice.
       * We try to handle that gracefully but it would be an error
       * in the scheduling if there ever were the case.
       */

      DEBUGASSERT(ticks <= rtcb->timeslice);
      decr = MIN(rtcb->timeslice, ticks);

      /* Decrement the timeslice counter */

      rtcb->timeslice -= decr;

      /* Did decrementing the timeslice counter cause the timeslice to
       * expire?
       *
       * If the task has pre-emption disabled. Then we will freeze the
       * timeslice count at the value until pre-emption has been enabled.
       */

      ret = rtcb->timeslice;
      if (rtcb->timeslice <= 0 && rtcb->lockcount == 0)
        {
          /* We will also suppress context switches if we were called
           * via one of the unusual cases handled by sched_timer_reasses().
           * In that case, we will return a value of one so that the
           * timer will expire as soon as possible and we can perform
           * this action in the normal timer expiration context.
           *
           * This is kind of kludge, but I am not to concerned because
           * I hope that the situation is impossible or at least could
           * only occur on rare corner-cases.
           */

          if (noswitches)
            {
              ret = 1;
            }
          else
            {
              /* Reset the timeslice. */

              rtcb->timeslice = MSEC2TICK(CONFIG_RR_INTERVAL);
              ret = rtcb->timeslice;

              /* We know we are at the head of the ready to run
               * prioritized list.  We must be the highest priority
               * task eligible for execution.  Check the next task
               * in the ready to run list.  If it is the same
               * priority, then we need to relinquish the CPU and
               * give that task a shot.
               */

              if (rtcb->flink &&
                  rtcb->flink->sched_priority >= rtcb->sched_priority)
                {
                  /* Just resetting the task priority to its current
                   * value.  This this will cause the task to be
                   * rescheduled behind any other tasks at the same
                   * priority.
                   */

                  up_reprioritize_rtr(rtcb, rtcb->sched_priority);

                  /* We will then need to return timeslice remaining for
                   * the new task at the head of the ready to run list
                   */
  
                  rtcb = (FAR struct tcb_s*)g_readytorun.head;

                  /* Check if the new task at the head of the ready-to-run
                   * supports round robin scheduling.
                   */

                  if ((rtcb->flags & TCB_FLAG_ROUND_ROBIN) != 0)
                    {
                      /* The new task at the head of the ready to run
                       * list does not support round robin scheduling.
                       */

#ifdef KEEP_ALIVE_HACK
                      ret = MSEC2TICK(CONFIG_RR_INTERVAL);
#else
                      ret = 0;
#endif
                    }
                  else
                    {
                      /* Return the time remaining on slice for the new
                       * task (or at least one for the same reasons as
                       * discussed above).
                       */

                      ret = rtcb->timeslice > 0 ? rtcb->timeslice : 1;
                    }
                }
            }
        }
    }

  return ret;
}
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
#if CONFIG_RR_INTERVAL > 0
  unsigned int cmptime = UINT_MAX;
#endif
  unsigned int rettime  = 0;
  unsigned int tmp;

  /* Process watchdogs */

  tmp = wd_timer(ticks);
  if (tmp > 0)
    {
#if CONFIG_RR_INTERVAL > 0
      cmptime = tmp;
#endif
      rettime  = tmp;
    }

#if CONFIG_RR_INTERVAL > 0
  /* Check if the currently executing task has exceeded its
   * timeslice.
   */

  tmp = sched_process_timeslice(ticks, noswitches);
  if (tmp > 0 && tmp < cmptime)
    {
      rettime  = tmp;
    }
#endif

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

      sched_timespec_add(&g_stop_time, &ts, &ts);
      ret = up_alarm_start(&ts);

#else
       /* [Re-]start the interval timer */

       ret = up_timer_start(&ts);
#endif

      /* [Re-]start the interval timer */

      ret = up_timer_start(&ts);
      if (ret < 0)
        {
          slldbg("ERROR: up_timer_start failed: %d\n");
          UNUSED(ret);
        }
    }
}

/************************************************************************
 * Public Functions
 ************************************************************************/

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

  /* Convert this to the elapsed time */

  sched_timespec_subtract(&g_stop_time, &ts, &ts);

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
 *   this, then there is really no need to do anything when on context
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
