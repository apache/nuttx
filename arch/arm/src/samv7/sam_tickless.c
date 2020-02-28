/****************************************************************************
 * arch/arm/src/samv7/sam_tickless.c
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
 * Tickless OS Support.
 *
 * When CONFIG_SCHED_TICKLESS is enabled, all support for timer interrupts
 * is suppressed and the platform specific code is expected to provide the
 * following custom functions.
 *
 *   void up_timer_initialize(void): Initializes the timer facilities.  Called
 *     early in the initialization sequence (by up_initialize()).
 *   int up_timer_gettime(FAR struct timespec *ts):  Returns the current
 *     time from the platform specific time source.
 *   int up_timer_cancel(void):  Cancels the interval timer.
 *   int up_timer_start(FAR const struct timespec *ts): Start (or re-starts)
 *     the interval timer.
 *
 * The RTOS will provide the following interfaces for use by the platform-
 * specific interval timer implementation:
 *
 *   void nxsched_timer_expiration(void):  Called by the platform-specific
 *     logic when the interval timer expires.
 *
 ****************************************************************************/
/****************************************************************************
 * SAMV7 Timer Usage
 *
 * This current implementation uses two timers:  A one-shot timer to provide
 * the timed events and a free running timer to provide the current time.
 * Since timers are a limited resource, that could be an issue on some
 * systems.
 *
 * We could do the job with a single timer if we were to keep the single
 * timer in a free-running at all times.  The SAMV7 timer/counters have
 * 16-bit counters with the capability to generate a compare interrupt when
 * the timer matches a compare value but also to continue counting without
 * stopping (giving another, different interrupt when the timer rolls over
 * from 0xffff to zero).  So we could potentially just set the compare at
 * the number of ticks you want PLUS the current value of timer.  Then you
 * could have both with a single timer:  An interval timer and a free-
 * running counter with the same timer!
 *
 * Patches are welcome!
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>

#include <nuttx/arch.h>

#include "sam_oneshot.h"
#include "sam_freerun.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SAMV7_HAVE_TC
#  error Timer/counters must be selected for the Tickless OS option
#endif

#ifndef CONFIG_SAMV7_ONESHOT
#  error CONFIG_SAMV7_ONESHOT must be selected for the Tickless OS option
#endif

#ifndef CONFIG_SAMV7_FREERUN
#  error CONFIG_SAMV7_FREERUN must be selected for the Tickless OS option
#endif

#ifndef CONFIG_SAMV7_TICKLESS_FREERUN
#  error CONFIG_SAMV7_TICKLESS_FREERUN must be selected for the Tickless OS option
#endif

#ifndef CONFIG_SAMV7_TICKLESS_ONESHOT
#  error CONFIG_SAMV7_TICKLESS_ONESHOT must be selected for the Tickless OS option
#endif

#if CONFIG_SAMV7_TICKLESS_ONESHOT == 0 && !defined(CONFIG_SAMV7_TC0)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 0 && CONFIG_SAMV7_TC0 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 1 && !defined(CONFIG_SAMV7_TC0)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 1 && CONFIG_SAMV7_TC0 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 2 && !defined(CONFIG_SAMV7_TC0)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 2 && CONFIG_SAMV7_TC0 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 3 && !defined(CONFIG_SAMV7_TC1)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 3 && CONFIG_SAMV7_TC1 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 4 && !defined(CONFIG_SAMV7_TC1)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 4 && CONFIG_SAMV7_TC1 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 5 && !defined(CONFIG_SAMV7_TC1)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 5 && CONFIG_SAMV7_TC1 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 6 && !defined(CONFIG_SAMV7_TC2)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 6 && CONFIG_SAMV7_TC2 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 7 && !defined(CONFIG_SAMV7_TC2)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 7 && CONFIG_SAMV7_TC2 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 8 && !defined(CONFIG_SAMV7_TC2)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 8 && CONFIG_SAMV7_TC2 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 9 && !defined(CONFIG_SAMV7_TC3)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 9 && CONFIG_SAMV7_TC3 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 10 && !defined(CONFIG_SAMV7_TC3)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 10 && CONFIG_SAMV7_TC3 not selected
#elif CONFIG_SAMV7_TICKLESS_ONESHOT == 11 && !defined(CONFIG_SAMV7_TC3)
#  error CONFIG_SAMV7_TICKLESS_ONESHOT == 11 && CONFIG_SAMV7_TC3 not selected
#endif

#if CONFIG_SAMV7_TICKLESS_ONESHOT < 0 || CONFIG_SAMV7_TICKLESS_ONESHOT > 11
#  error CONFIG_SAMV7_TICKLESS_ONESHOT is not valid
#endif

#if CONFIG_SAMV7_TICKLESS_FREERUN == 0 && !defined(CONFIG_SAMV7_TC0)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 0 && CONFIG_SAMV7_TC0 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 1 && !defined(CONFIG_SAMV7_TC0)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 1 && CONFIG_SAMV7_TC0 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 2 && !defined(CONFIG_SAMV7_TC0)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 2 && CONFIG_SAMV7_TC0 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 3 && !defined(CONFIG_SAMV7_TC1)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 3 && CONFIG_SAMV7_TC1 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 4 && !defined(CONFIG_SAMV7_TC1)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 4 && CONFIG_SAMV7_TC1 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 5 && !defined(CONFIG_SAMV7_TC1)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 5 && CONFIG_SAMV7_TC1 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 6 && !defined(CONFIG_SAMV7_TC2)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 6 && CONFIG_SAMV7_TC2 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 7 && !defined(CONFIG_SAMV7_TC2)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 7 && CONFIG_SAMV7_TC2 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 8 && !defined(CONFIG_SAMV7_TC2)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 8 && CONFIG_SAMV7_TC2 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 9 && !defined(CONFIG_SAMV7_TC3)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 9 && CONFIG_SAMV7_TC3 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 10 && !defined(CONFIG_SAMV7_TC3)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 10 && CONFIG_SAMV7_TC3 not selected
#elif CONFIG_SAMV7_TICKLESS_FREERUN == 11 && !defined(CONFIG_SAMV7_TC3)
#  error CONFIG_SAMV7_TICKLESS_FREERUN == 11 && CONFIG_SAMV7_TC3 not selected
#endif

#if CONFIG_SAMV7_TICKLESS_FREERUN < 0 || CONFIG_SAMV7_TICKLESS_FREERUN > 11
#  error CONFIG_SAMV7_TICKLESS_FREERUN is not valid
#endif

#if CONFIG_SAMV7_TICKLESS_FREERUN == CONFIG_SAMV7_TICKLESS_ONESHOT
#  error CONFIG_SAMV7_TICKLESS_FREERUN is the same as CONFIG_SAMV7_TICKLESS_ONESHOT
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sam_tickless_s
{
  struct sam_oneshot_s oneshot;
  struct sam_freerun_s freerun;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct sam_tickless_s g_tickless;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_oneshot_handler
 *
 * Description:
 *   Called when the one shot timer expires
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in the initialization sequence before any special
 *   concurrency protections are required.
 *
 ****************************************************************************/

static void sam_oneshot_handler(void *arg)
{
  tmrinfo("Expired...\n");
  nxsched_timer_expiration();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_timer_initialize
 *
 * Description:
 *   Initializes all platform-specific timer facilities.  This function is
 *   called early in the initialization sequence by up_initialize().
 *   On return, the current up-time should be available from
 *   up_timer_gettime() and the interval timer is ready for use (but not
 *   actively timing.
 *
 *   Provided by platform-specific code and called from the architecture-
 *   specific logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in the initialization sequence before any special
 *   concurrency protections are required.
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  int ret;

  /* Initialize the one-shot timer */

  ret = sam_oneshot_initialize(&g_tickless.oneshot,
                               CONFIG_SAMV7_TICKLESS_ONESHOT,
                               CONFIG_USEC_PER_TICK);
  if (ret < 0)
    {
      tmrerr("ERROR: sam_oneshot_initialize failed\n");
      DEBUGPANIC();
    }

  DEBUGASSERT(ONESHOT_INITIALIZED(&g_tickless.oneshot));

  /* Initialize the free-running timer */

  ret = sam_freerun_initialize(&g_tickless.freerun,
                               CONFIG_SAMV7_TICKLESS_FREERUN,
                               CONFIG_USEC_PER_TICK);
  if (ret < 0)
    {
      tmrerr("ERROR: sam_freerun_initialize failed\n");
      DEBUGPANIC();
    }

  DEBUGASSERT(FREERUN_INITIALIZED(&g_tickless.freerun));
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   up_timer_initialize() was called).  This function is functionally
 *   equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, FAR struct timespec *ts);
 *
 *   when clockid is CLOCK_MONOTONIC.
 *
 *   This function provides the basis for reporting the current time and
 *   also is used to eliminate error build-up from small errors in interval
 *   time calculations.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the location in which to return the up-time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   Called from the normal tasking context.  The implementation must
 *   provide whatever mutual exclusion is necessary for correct operation.
 *   This can include disabling interrupts in order to assure atomic register
 *   operations.
 *
 ****************************************************************************/

int up_timer_gettime(FAR struct timespec *ts)
{
  return FREERUN_INITIALIZED(&g_tickless.freerun) ?
         sam_freerun_counter(&g_tickless.freerun, ts) :
         -EAGAIN;
}

/****************************************************************************
 * Name: up_timer_cancel
 *
 * Description:
 *   Cancel the interval timer and return the time remaining on the timer.
 *   These two steps need to be as nearly atomic as possible.
 *   nxsched_timer_expiration() will not be called unless the timer is
 *   restarted with up_timer_start().
 *
 *   If, as a race condition, the timer has already expired when this
 *   function is called, then that pending interrupt must be cleared so
 *   that up_timer_start() and the remaining time of zero should be
 *   returned.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the remaining time.  Zero should be returned
 *        if the timer is not active.  ts may be zero in which case the
 *        time remaining is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

int up_timer_cancel(FAR struct timespec *ts)
{
  return ONESHOT_INITIALIZED(&g_tickless.oneshot) && FREERUN_INITIALIZED(&g_tickless.freerun) ?
         sam_oneshot_cancel(&g_tickless.oneshot, &g_tickless.freerun, ts) :
         -EAGAIN;
}

/****************************************************************************
 * Name: up_timer_start
 *
 * Description:
 *   Start the interval timer.  nxsched_timer_expiration() will be
 *   called at the completion of the timeout (unless up_timer_cancel
 *   is called to stop the timing.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the time interval until nxsched_timer_expiration() is
 *        called.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

int up_timer_start(FAR const struct timespec *ts)
{
  return ONESHOT_INITIALIZED(&g_tickless.oneshot) ?
         sam_oneshot_start(&g_tickless.oneshot, &g_tickless.freerun, sam_oneshot_handler, NULL, ts) :
         -EAGAIN;
}
#endif /* CONFIG_SCHED_TICKLESS */
