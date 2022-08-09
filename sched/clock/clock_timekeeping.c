/****************************************************************************
 * sched/clock/clock_timekeeping.c
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

#ifdef CONFIG_CLOCK_TIMEKEEPING

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

#define NTP_MAX_ADJUST 500

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timespec g_clock_wall_time;
static uint64_t        g_clock_last_counter;
static uint64_t        g_clock_mask;
static long            g_clock_adjust;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_get_current_time
 ****************************************************************************/

static int clock_get_current_time(FAR struct timespec *ts,
                                  FAR struct timespec *base)
{
  irqstate_t flags;
  uint64_t counter;
  uint64_t offset;
  uint64_t nsec;
  time_t sec;
  int ret;

  flags = enter_critical_section();

  ret = up_timer_getcounter(&counter);
  if (ret < 0)
    {
      goto errout_in_critical_section;
    }

  offset = (counter - g_clock_last_counter) & g_clock_mask;
  nsec   = offset * NSEC_PER_TICK;
  sec    = nsec   / NSEC_PER_SEC;
  nsec  -= sec    * NSEC_PER_SEC;

  nsec  += base->tv_nsec;
  if (nsec >= NSEC_PER_SEC)
    {
      nsec -= NSEC_PER_SEC;
      sec  += 1;
    }

  ts->tv_nsec = nsec;
  ts->tv_sec = base->tv_sec + sec;

errout_in_critical_section:
  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_timekeeping_get_wall_time
 ****************************************************************************/

int clock_timekeeping_get_wall_time(FAR struct timespec *ts)
{
  return clock_get_current_time(ts, &g_clock_wall_time);
}

/****************************************************************************
 * Name: clock_timekeeping_set_wall_time
 ****************************************************************************/

int clock_timekeeping_set_wall_time(FAR const struct timespec *ts)
{
  irqstate_t flags;
  uint64_t counter;
  int ret;

  flags = enter_critical_section();

  ret = up_timer_getcounter(&counter);
  if (ret < 0)
    {
      goto errout_in_critical_section;
    }

  memcpy(&g_clock_wall_time, ts, sizeof(struct timespec));

  g_clock_adjust       = 0;
  g_clock_last_counter = counter;

errout_in_critical_section:
  leave_critical_section(flags);
  return ret;
}

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
 *         suseconds_t tv_usec;    (microseconds)
 *       };
 *
 *   If the adjustment in delta is positive, then the system clock is
 *   speeded up by some small percentage (i.e., by adding a small amount of
 *   time to the clock value in each second) until the adjustment has been
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
  long adjust_usec;

  if (!delta)
    {
      set_errno(EINVAL);
      return -1;
    }

  flags = enter_critical_section();

  adjust_usec = delta->tv_sec * USEC_PER_SEC + delta->tv_usec;

  if (olddelta)
    {
      olddelta->tv_usec = g_clock_adjust;
    }

  g_clock_adjust = adjust_usec;

  leave_critical_section(flags);

  return OK;
}

/****************************************************************************
 * Name: clock_update_wall_time
 ****************************************************************************/

void clock_update_wall_time(void)
{
  irqstate_t flags;
  uint64_t counter;
  uint64_t offset;
  int64_t nsec;
  time_t sec;
  int ret;

  flags = enter_critical_section();

  ret = up_timer_getcounter(&counter);
  if (ret < 0)
    {
      goto errout_in_critical_section;
    }

  offset = (counter - g_clock_last_counter) & g_clock_mask;
  if (offset == 0)
    {
      goto errout_in_critical_section;
    }

  nsec  = offset * NSEC_PER_TICK;
  sec   = nsec / NSEC_PER_SEC;
  nsec -= sec * NSEC_PER_SEC;

  nsec += g_clock_wall_time.tv_nsec;
  if (nsec >= NSEC_PER_SEC)
    {
      nsec -= NSEC_PER_SEC;
      sec  += 1;
    }

  if (g_clock_adjust != 0 && sec > 0)
    {
      long adjust = NTP_MAX_ADJUST * (long)sec;
      if (g_clock_adjust < adjust && g_clock_adjust > -adjust)
        {
          adjust = g_clock_adjust;
        }

      nsec += adjust * NSEC_PER_USEC;

      while (nsec < 0)
        {
          nsec += NSEC_PER_SEC;
          sec  -= 1;
        }

      while (nsec >= NSEC_PER_SEC)
        {
          nsec -= NSEC_PER_SEC;
          sec  += 1;
        }
    }

  g_clock_wall_time.tv_sec += sec;
  g_clock_wall_time.tv_nsec = (long)nsec;

  g_clock_last_counter = counter;

errout_in_critical_section:
  leave_critical_section(flags);
}

/****************************************************************************
 * Name: clock_inittimekeeping
 ****************************************************************************/

void clock_inittimekeeping(FAR const struct timespec *tp)
{
  up_timer_getmask(&g_clock_mask);

  if (tp)
    {
      memcpy(&g_clock_wall_time, tp, sizeof(struct timespec));
    }
  else
    {
      clock_basetime(&g_clock_wall_time);
    }

  up_timer_getcounter(&g_clock_last_counter);
}

#endif /* CONFIG_CLOCK_TIMEKEEPING */
