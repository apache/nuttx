/****************************************************************************
 * arch/sparc/src/bm3803/bm3803_tickless.c
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
 * Tickless OS Support.
 *
 * When CONFIG_SCHED_TICKLESS is enabled, all support for timer interrupts
 * is suppressed and the platform specific code is expected to provide the
 * following custom functions.
 *
 *   void up_timer_initialize(void): Initializes the timer facilities.
 *     Called early in the initialization sequence (by up_initialize()).
 *   int up_timer_gettime(struct timespec *ts):  Returns the current
 *     time from the platform specific time source.
 *   int up_timer_cancel(void):  Cancels the interval timer.
 *   int up_timer_start(const struct timespec *ts): Start (or re-starts)
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
 * BM3803 Timer Usage
 *
 * This current implementation uses two timers:  A one-shot timer to provide
 * the timed events and a free running timer to provide the current time.
 * Since timers are a limited resource, that could be an issue on some
 * systems.
 *
 * We could do the job with a single timer if we were to keep the single
 * timer in a free-running at all times.  The STM32 timer/counters have
 * 16-bit/32-bit counters with the capability to generate a compare interrupt
 * when the timer matches a compare value but also to continue counting
 * without stopping (giving another, different interrupt when the timer
 * rolls over from 0xffffffff to zero).  So we could potentially just set
 * the compare at the number of ticks you want PLUS the current value of
 * timer.  Then you could have both with a single timer:  An interval timer
 * and a free-running counter with the same timer!
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

#include <nuttx/arch.h>
#include <assert.h>
#include <debug.h>

#include "bm3803_oneshot.h"
#include "bm3803_freerun.h"

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BM3803_ONESHOT
#  error CONFIG_BM3803_ONESHOT must be selected for the Tickless OS option
#endif

#ifndef CONFIG_BM3803_FREERUN
#  error CONFIG_BM3803_FREERUN must be selected for the Tickless OS option
#endif

#ifndef CONFIG_BM3803_TICKLESS_FREERUN
#  error CONFIG_BM3803_TICKLESS_FREERUN must be selected for the Tickless OS option
#endif

#ifndef CONFIG_BM3803_TICKLESS_ONESHOT
#  error CONFIG_BM3803_TICKLESS_ONESHOT must be selected for the Tickless OS option
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct bm3803_tickless_s
{
  struct bm3803_oneshot_s oneshot;
  struct bm3803_freerun_s freerun;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct bm3803_tickless_s g_tickless;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3803_oneshot_handler
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

static void bm3803_oneshot_handler(void *arg)
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
#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
  uint64_t max_delay;
#endif
  int ret;

  /* Initialize the one-shot timer */

  ret = bm3803_oneshot_initialize(&g_tickless.oneshot,
                                 CONFIG_BM3803_TICKLESS_ONESHOT,
                                 CONFIG_USEC_PER_TICK);
  if (ret < 0)
    {
      tmrerr("ERROR: bm3803_oneshot_initialize failed\n");
      DEBUGASSERT(0);
    }

#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
  /* Get the maximum delay of the one-shot timer in microseconds */

  ret = bm3803_oneshot_max_delay(&g_tickless.oneshot, &max_delay);
  if (ret < 0)
    {
      tmrerr("ERROR: bm3803_oneshot_max_delay failed\n");
      DEBUGASSERT(0);
    }

  /* Convert this to configured clock ticks for use by the OS timer logic */

  max_delay /= CONFIG_USEC_PER_TICK;
  if (max_delay > UINT24_MAX)
    {
      g_oneshot_maxticks = UINT24_MAX;
    }
  else
    {
      g_oneshot_maxticks = max_delay;
    }
#endif

  /* Initialize the free-running timer */

  ret = bm3803_freerun_initialize(&g_tickless.freerun,
                                 CONFIG_BM3803_TICKLESS_FREERUN,
                                 CONFIG_USEC_PER_TICK);
  if (ret < 0)
    {
      tmrerr("ERROR: bm3803_freerun_initialize failed\n");
      DEBUGASSERT(0);
    }
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   up_timer_initialize() was called).  This function is functionally
 *   equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, struct timespec *ts);
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

int up_timer_gettime(struct timespec *ts)
{
  return bm3803_freerun_counter(&g_tickless.freerun, ts);
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

int up_timer_cancel(struct timespec *ts)
{
  return bm3803_oneshot_cancel(&g_tickless.oneshot, ts);
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

int up_timer_start(const struct timespec *ts)
{
  return bm3803_oneshot_start(&g_tickless.oneshot, bm3803_oneshot_handler,
                              NULL, ts);
}
#endif /* CONFIG_SCHED_TICKLESS */
