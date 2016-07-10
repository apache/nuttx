/************************************************************************
 * sched/clock/clock_timekeeping.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author:  Max Neklyudov <macscomp@gmail.com>
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

#ifdef CONFIG_SCHED_TIMEKEEPING

#include <sys/time.h>
#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "clock/clock.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

#define NTP_MAX_ADJUST 500

/**********************************************************************
 * Private Data
 **********************************************************************/

static struct timespec g_clock_wall_time;
static struct timespec g_clock_monotonic_time;
static uint64_t        g_clock_last_counter;
static uint64_t        g_clock_mask;
static long            g_clock_adjust;

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Name: clock_get_current_time
 ************************************************************************/

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
  if (nsec > NSEC_PER_SEC)
    {
      nsec -= NSEC_PER_SEC;
      sec += 1;
    }

  ts->tv_nsec = nsec;
  ts->tv_sec = base->tv_sec + sec;

errout_in_critical_section:
  leave_critical_section(flags);
  return ret;
}

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: clock_timekeeping_get_monotonic_time
 ************************************************************************/

int clock_timekeeping_get_monotonic_time(FAR struct timespec *ts)
{
  return clock_get_current_time(ts, &g_clock_monotonic_time);
}

/************************************************************************
 * Name: clock_timekeeping_get_wall_time
 ************************************************************************/

int clock_timekeeping_get_wall_time(FAR struct timespec *ts)
{
  return clock_get_current_time(ts, &g_clock_wall_time);
}

/************************************************************************
 * Name: clock_timekeeping_set_wall_time
 ************************************************************************/

int clock_timekeeping_set_wall_time(FAR struct timespec *ts)
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

  g_clock_wall_time    = *ts;
  g_clock_adjust       = 0;
  g_clock_last_counter = counter;

errout_in_critical_section:
  leave_critical_section(flags);
  return ret;
}

/************************************************************************
 * Name: adjtime
 ************************************************************************/

int adjtime(const struct timeval *delta, struct timeval *olddelta)
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

/************************************************************************
 * Name: clock_update_wall_time
 ************************************************************************/

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

  g_clock_monotonic_time.tv_sec += sec;
  g_clock_monotonic_time.tv_nsec += nsec;
  if (g_clock_monotonic_time.tv_nsec > NSEC_PER_SEC)
    {
      g_clock_monotonic_time.tv_nsec -= NSEC_PER_SEC;
      g_clock_monotonic_time.tv_sec += 1;
    }

  nsec += g_clock_wall_time.tv_nsec;
  if (nsec > NSEC_PER_SEC)
    {
      nsec -= NSEC_PER_SEC;
      sec += 1;
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
          sec -= 1;
        }

      while (nsec > NSEC_PER_SEC)
        {
          nsec -= NSEC_PER_SEC;
          sec += 1;
        }
    }

  g_clock_wall_time.tv_sec += sec;
  g_clock_wall_time.tv_nsec = (long)nsec;

  g_clock_last_counter = counter;

errout_in_critical_section:
  leave_critical_section(flags);
}

/************************************************************************
 * Name: clock_inittimekeeping
 ************************************************************************/

void clock_inittimekeeping(void)
{
  struct tm rtctime;

  up_timer_getmask(&g_clock_mask);

  /* Get the broken-errout_in_critical_section time from the date/time RTC. */

  (void)up_rtc_getdatetime(&rtctime);

  /* And use the broken-errout_in_critical_section time to initialize the system time */

  g_clock_wall_time.tv_sec  = mktime(&rtctime);
  g_clock_wall_time.tv_nsec = 0;

  memset(&g_clock_monotonic_time, 0, sizeof(g_clock_monotonic_time));
}

#endif /* CONFIG_SCHED_TIMEKEEPING */
