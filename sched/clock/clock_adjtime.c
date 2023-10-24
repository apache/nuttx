/****************************************************************************
 * sched/clock/clock_adjtime.c
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

#ifdef CONFIG_CLOCK_ADJTIME

#include <sys/time.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Current adjtime implementation uses periodic clock tick to adjust clock
 * period. Therefore this implementation will not work when tickless mode
 * is enabled by CONFIG_SCHED_TICKLESS=y
 */

#ifdef CONFIG_SCHED_TICKLESS
# error CONFIG_CLOCK_ADJTIME is not supported when CONFIG_SCHED_TICKLESS \
        is enabled!
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adjtime
 *
 * Description:
 *   The adjtime() function gradually adjusts the system clock (as returned
 *   by gettimeofday(2)).  The amount of time by which the clock is to be
 *   adjusted is specified in the structure pointed to by delta.
 *
 *   This structure has the following form:
 *
 *     struct timeval
 *       {
 *         time_t      tv_sec;     (seconds)
 *         long        tv_usec;    (microseconds)
 *       };
 *
 *   If the adjustment in delta is positive, then the system clock is
 *   speeded up by some small percentage until the adjustment has been
 *   completed.  If the adjustment in delta is negative, then the clock is
 *   slowed down in a similar fashion.
 *
 *   If a clock adjustment from an earlier adjtime() call is already in
 *   progress at the time of a later adjtime() call, and delta is not NULL
 *   for the later call, then the earlier adjustment is stopped, but any
 *   already completed part of that adjustment is not undone.
 *
 *   If olddelta is not NULL, then the buffer that it points to is used to
 *   return the amount of time remaining from any previous adjustment that
 *   has not yet been completed.
 *
 *   NOTE: This is not a POSIX interface but derives from 4.3BSD, System V.
 *   It is also supported for Linux compatibility.
 *
 ****************************************************************************/

int adjtime(FAR const struct timeval *delta, FAR struct timeval *olddelta)
{
  irqstate_t flags;
  long long adjust_usec;
  long long period_usec;
  long long adjust_usec_old;
  long long count;                /* Number of cycles over which
                                   * we adjust the period
                                   */
  long long incr;                 /* Period increment applied on
                                   * every cycle.
                                   */
  long long count_old;            /* Previous number of cycles over which
                                   * we adjust the period
                                   */
  long long incr_old;             /* Previous period increment applied on
                                   * every cycle.
                                   */
  long long incr_limit;
  int is_negative;

  if (!delta)
    {
      set_errno(EINVAL);
      return -1;
    }

  flags = enter_critical_section();

  adjust_usec = (long long)delta->tv_sec * USEC_PER_SEC + delta->tv_usec;

  if (adjust_usec < 0)
    {
      adjust_usec = -adjust_usec;
      is_negative = 1;
    }
  else
    {
      is_negative = 0;
    }

  /* Get period in usec. Target hardware has to provide support for
   * this function call.
   */

  up_get_timer_period(&period_usec);

  /* Determine how much we want to adjust timer period and the number
   * of cycles over which we want to do the adjustment.
   */

  count = (USEC_PER_MSEC * CONFIG_CLOCK_ADJTIME_PERIOD_MS) / period_usec;
  incr = adjust_usec / count;

  /* Compute maximum possible period increase and check
   * whether previously computed increase exceeds the maximum
   * one.
   */

  incr_limit = CONFIG_CLOCK_ADJTIME_SLEWLIMIT_PPM
               / (USEC_PER_SEC / period_usec);
  if (incr > incr_limit)
    {
      /* It does... limit computed increase and increment count. */

      incr = incr_limit;
      count = adjust_usec / incr;
    }

  /* If requested adjustment is smaller than 1 microsecond per tick,
   * adjust the count instead.
   */

  if (adjust_usec == 0)
    {
      incr = 0;
      count = 0;
    }
  else if (incr == 0)
    {
      incr = 1;
      count = adjust_usec / incr;
    }

  if (is_negative == 1)
    {
      /* Positive or negative? */

      incr = -incr;
    }

  leave_critical_section(flags);

  /* Initialize clock adjustment and get old adjust values. */

  clock_set_adjust(incr, count, &incr_old, &count_old);

  adjust_usec_old = count_old * incr_old;
  if (olddelta)
    {
      olddelta->tv_sec  = adjust_usec_old / USEC_PER_SEC;
      olddelta->tv_usec = adjust_usec_old;
    }

  return OK;
}

#endif /* CONFIG_CLOCK_ADJTIME */
