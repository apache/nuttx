/************************************************************************
 * sched/sched_timerexpiration.c
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

#include "os_internal.h"
#include "wdog/wdog.h"
#include "clock/clock.h"

#ifdef CONFIG_SCHED_TICKLESS

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

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

/************************************************************************
 * Private Functions
 ************************************************************************/

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
  unsigned int ret = 0;
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
                  if ((rtcb->flags & TCB_FLAG_ROUND_ROBIN) != 0)
                    {
                      /* The new task at the head of the ready to run
                       * list does not support round robin scheduling.
                       */

                      ret = 0;
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
 *   Base code implementation assumes that this function is called from
 *   interrupt handling logic with interrupts disabled.
 *
 ****************************************************************************/

static void sched_timer_process(unsigned int ticks, bool noswitches)
{
  unsigned int nextime = UINT_MAX;
  bool needtimer = false;
  uint32_t msecs;
  uint32_t secs;
  uint32_t nsecs;
  unsigned int tmp;
  int ret;

  /* Process watchdogs */

  tmp = wd_timer(ticks);
  if (tmp > 0)
    {
      nextime   = tmp;
      needtimer = true;
    }

#if CONFIG_RR_INTERVAL > 0
  /* Check if the currently executing task has exceeded its
   * timeslice.
   */

  tmp = sched_process_timeslice(ticks, noswitches);
  if (tmp > 0 && tmp < nextime)
    {
      nextime   = tmp;
      needtimer = true;
    }
#endif

  /* Set up the next timer interval (or not) */

  g_timer_interval = 0;
  if (needtimer)
    {
      struct timespec ts;

      /* Save new timer interval */

      g_timer_interval = nextime;

      /* Convert ticks to a struct timespec that up_timer_start() can
       * understand.
       */

      msecs = TICK2MSEC(nextime);
      secs  = msecs / MSEC_PER_SEC;
      nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;

      ts.tv_sec  = (time_t)secs;
      ts.tv_nsec = (long)nsecs;

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
 * Name: sched_timer_expiration
 *
 * Description:
 *   if CONFIG_SCHED_TICKLESS is defined, then this function is provided by
 *   the RTOS base code and called from platform-specific code when the
 *   interval timer used to implemented the tick-less OS expires.
 *
 * Input Parameters:
 *
 * Returned Value:
 *   Base code implementation assumes that this function is called from
 *   interrupt handling logic with interrupts disabled.
 *
 ****************************************************************************/

void sched_timer_expiration(void)
{
  unsigned int elapsed;

  elapsed          = g_timer_interval;
  g_timer_interval = 0;
  sched_timer_process(elapsed, false);
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
 *   - Any context switch occurs, changing the task at the head of the
 *     ready-to-run list.  The task at the head of list may require
 *     different timeslice processing (or no timeslice at all).
 *   - When pre-emption is re-enabled.  A previous time slice may have
 *     expired while pre-emption was enabled and now needs to be executed.
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
  sched_timer_process(elapsed, true);
}
#endif /* CONFIG_SCHED_TICKLESS */
