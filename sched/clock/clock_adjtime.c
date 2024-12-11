/****************************************************************************
 * sched/clock/clock_adjtime.c
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

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct wdog_s g_adjtime_wdog;
static long g_adjtime_ppb;
static spinlock_t g_adjtime_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Restore default rate after adjustment period expires */

static void adjtime_wdog_callback(wdparm_t arg)
{
  irqstate_t flags;

  UNUSED(arg);

  flags = spin_lock_irqsave(&g_adjtime_lock);

#ifdef CONFIG_ARCH_HAVE_ADJTIME
  up_adjtime(0);
#endif

#ifdef CONFIG_RTC_ADJTIME
  up_rtc_adjtime(0);
#endif

  g_adjtime_ppb = 0;
  spin_unlock_irqrestore(&g_adjtime_lock, flags);
}

/* Query remaining adjustment in microseconds */

static long long adjtime_remaining_usec(void)
{
  return (long long)g_adjtime_ppb
    * TICK2MSEC(wd_gettime(&g_adjtime_wdog))
    / (MSEC_PER_SEC * NSEC_PER_USEC);
}

/* Start new adjustment period */

static int adjtime_start(long long adjust_usec)
{
  long long ppb;
  long long ppb_limit;
  irqstate_t flags;
  int ret = OK;

  /* Calculate rate adjustmend to get adjust_usec change over the
   * CONFIG_CLOCK_ADJTIME_PERIOD_MS.
   */

  ppb = adjust_usec * NSEC_PER_USEC;
  ppb = ppb * MSEC_PER_SEC / CONFIG_CLOCK_ADJTIME_PERIOD_MS;

  /* Limit to maximum rate adjustment */

  ppb_limit = CONFIG_CLOCK_ADJTIME_SLEWLIMIT_PPM * 1000;
  if (ppb > ppb_limit)
    {
      ppb = ppb_limit;
    }
  else if (ppb < -ppb_limit)
    {
      ppb = -ppb_limit;
    }

  flags = spin_lock_irqsave(&g_adjtime_lock);
  sched_lock();

  /* Set new adjustment */

  g_adjtime_ppb = ppb;

#ifdef CONFIG_ARCH_HAVE_ADJTIME
  up_adjtime(g_adjtime_ppb);
#endif

#ifdef CONFIG_RTC_ADJTIME
  ret = up_rtc_adjtime(g_adjtime_ppb);
#endif

  /* Queue cancellation of adjustment after configured period */

  if (g_adjtime_ppb != 0)
    {
      wd_start(&g_adjtime_wdog, MSEC2TICK(CONFIG_CLOCK_ADJTIME_PERIOD_MS),
              adjtime_wdog_callback, 0);
    }
  else
    {
      wd_cancel(&g_adjtime_wdog);
    }

  spin_unlock_irqrestore(&g_adjtime_lock, flags);
  sched_unlock();

  return ret;
}

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
  long long adjust_usec = 0;
  long long adjust_usec_old = 0;
  int ret = OK;

  if (olddelta)
    {
      adjust_usec_old = adjtime_remaining_usec();
      olddelta->tv_sec  = adjust_usec_old / USEC_PER_SEC;
      olddelta->tv_usec = adjust_usec_old;
    }

  if (delta)
    {
      adjust_usec = (long long)delta->tv_sec * USEC_PER_SEC + delta->tv_usec;
      ret = adjtime_start(adjust_usec);
    }

  if (ret < 0)
    {
      set_errno(-ret);
      return -1;
    }
  else
    {
      return OK;
    }
}

#endif /* CONFIG_CLOCK_ADJTIME */
