/****************************************************************************
 * sched/clock/clock_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2011-2012, 2017-2018 Gregory Nutt. All rights
 *     reserved.
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

#include <stdint.h>
#include <time.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_RTC
#  include <nuttx/irq.h>
#endif

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/time.h>

#include "clock/clock.h"
#ifdef CONFIG_CLOCK_TIMEKEEPING
#  include "clock/clock_timekeeping.h"
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS
#ifdef CONFIG_SYSTEM_TIME64
volatile uint64_t g_system_timer;
#else
volatile uint32_t g_system_timer;
#endif
#endif

#ifndef CONFIG_CLOCK_TIMEKEEPING
struct timespec   g_basetime;

#ifdef CONFIG_CLOCK_MONOTONIC
struct timespec   g_monotonic_basetime;
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_basetime
 *
 * Description:
 *   Get the initial time value from the best source available.
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
#if defined(CONFIG_RTC_DATETIME)
/* Initialize the system time using a broken out date/time structure */

static inline int clock_basetime(FAR struct timespec *tp)
{
  struct tm rtctime;
  long nsecs = 0;
  int ret;

  /* Get the broken-out time from the date/time RTC. */

#ifdef CONFIG_ARCH_HAVE_RTC_SUBSECONDS
  ret = up_rtc_getdatetime_with_subseconds(&rtctime, &nsecs);
#else
  ret = up_rtc_getdatetime(&rtctime);
#endif
  if (ret >= 0)
    {
      /* And use the broken-out time to initialize the system time */

      tp->tv_sec  = mktime(&rtctime);
      tp->tv_nsec = nsecs;
    }

  return ret;
}

#elif defined(CONFIG_RTC_HIRES)

/* Initialize the system time using a high-resolution structure */

static inline int clock_basetime(FAR struct timespec *tp)
{
  /* Get the complete time from the hi-res RTC. */

  return up_rtc_gettime(tp);
}

#else

/* Initialize the system time using seconds only */

static inline int clock_basetime(FAR struct timespec *tp)
{
  /* Get the seconds (only) from the lo-resolution RTC */

  tp->tv_sec  = up_rtc_time();
  tp->tv_nsec = 0;
  return OK;
}

#endif /* CONFIG_RTC_HIRES */
#else /* CONFIG_RTC */

static inline int clock_basetime(FAR struct timespec *tp)
{
  time_t jdn = 0;

  /* Get the EPOCH-relative julian date from the calendar year,
   * month, and date
   */

  jdn = clock_calendar2utc(CONFIG_START_YEAR, CONFIG_START_MONTH - 1,
                           CONFIG_START_DAY);

  /* Set the base time as seconds into this julian day. */

  tp->tv_sec  = jdn * SEC_PER_DAY;
  tp->tv_nsec = 0;
  return OK;
}

#endif /* CONFIG_RTC */

/****************************************************************************
 * Name: clock_inittime
 *
 * Description:
 *   Get the initial time value from the best source available.
 *
 ****************************************************************************/

static void clock_inittime(void)
{
  /* (Re-)initialize the time value to match the RTC */

#ifndef CONFIG_CLOCK_TIMEKEEPING
#ifndef CONFIG_RTC_HIRES
  clock_basetime(&g_basetime);
#endif
#ifndef CONFIG_SCHED_TICKLESS
  g_system_timer = INITIAL_SYSTEM_TIMER_TICKS;
  if (g_system_timer > 0)
    {
      struct timespec ts;

      (void)clock_ticks2time((sclock_t)g_system_timer, &ts);

      /* Adjust base time to hide initial timer ticks. */

      g_basetime.tv_sec  -= ts.tv_sec;
      g_basetime.tv_nsec -= ts.tv_nsec;
      while (g_basetime.tv_nsec < 0)
        {
          g_basetime.tv_nsec += NSEC_PER_SEC;
          g_basetime.tv_sec--;
        }

#ifdef CONFIG_CLOCK_MONOTONIC
      /* Adjust monotonic clock offset to hide initial timer ticks. */

      g_monotonic_basetime.tv_sec  -= ts.tv_sec;
      g_monotonic_basetime.tv_nsec -= ts.tv_nsec;
      while (g_monotonic_basetime.tv_nsec < 0)
        {
          g_monotonic_basetime.tv_nsec += NSEC_PER_SEC;
          g_monotonic_basetime.tv_sec--;
        }
#endif /* CONFIG_CLOCK_MONOTONIC */
    }
#endif /* !CONFIG_SCHED_TICKLESS */
#else
  clock_inittimekeeping();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_initialize
 *
 * Description:
 *   Perform one-time initialization of the timing facilities.
 *
 ****************************************************************************/

void clock_initialize(void)
{
#if defined(CONFIG_RTC) && !defined(CONFIG_RTC_EXTERNAL)
  /* Initialize the internal RTC hardware.  Initialization of external RTC
   * must be deferred until the system has booted.
   */

  up_rtc_initialize();
#endif

  /* Initialize the time value to match the RTC */

  clock_inittime();
}

/****************************************************************************
 * Name: clock_synchronize
 *
 * Description:
 *   Synchronize the system timer to a hardware RTC.  This operation is
 *   normally performed automatically by the system during clock
 *   initialization.  However, the user may also need to explicitly re-
 *   synchronize the system timer to the RTC under certain conditions where
 *   the system timer is known to be in error.  For example, in certain low-
 *   power states, the system timer may be stopped but the RTC will continue
 *   keep correct time.  After recovering from such low-power state, this
 *   function should be called to restore the correct system time.
 *
 *   Calling this function could result in system time going "backward" in
 *   time, especially with certain lower resolution RTC implementations.
 *   Time going backward could have bad consequences if there are ongoing
 *   timers and delays.  So use this interface with care.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#ifdef CONFIG_RTC
void clock_synchronize(void)
{
  irqstate_t flags;

  /* Re-initialize the time value to match the RTC */

  flags = enter_critical_section();
  clock_inittime();
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: clock_resynchronize
 *
 * Description:
 *   Resynchronize the system timer to a hardware RTC.  The user can
 *   explicitly re-synchronize the system timer to the RTC under certain
 *   conditions where the system timer is known to be in error.  For example,
 *   in certain low-power states, the system timer may be stopped but the
 *   RTC will continue keep correct time.  After recovering from such
 *   low-power state, this function should be called to restore the correct
 *   system time. Function also keeps monotonic clock at rate of RTC.
 *
 *   Calling this function will not result in system time going "backward" in
 *   time. If setting system time with RTC would result time going "backward"
 *   then resynchronization is not performed.
 *
 * Input Parameters:
 *   rtc_diff:  amount of time system-time is adjusted forward with RTC
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

#if defined(CONFIG_RTC) && !defined(CONFIG_SCHED_TICKLESS) && \
    !defined(CONFIG_CLOCK_TIMEKEEPING)
void clock_resynchronize(FAR struct timespec *rtc_diff)
{
  struct timespec rtc_time, bias, curr_ts;
  struct timespec rtc_diff_tmp;
  irqstate_t flags;
  int32_t carry;
  int ret;

  if (rtc_diff == NULL)
    {
      rtc_diff = &rtc_diff_tmp;
    }

  /* Set the time value to match the RTC */

  flags = enter_critical_section();

  /* Get RTC time */

  ret = clock_basetime(&rtc_time);
  if (ret < 0)
    {
      /* Error reading RTC, skip resynchronization. */

      sinfo("rtc error %d, skip resync\n", ret);

      rtc_diff->tv_sec = 0;
      rtc_diff->tv_nsec = 0;
      goto skip;
    }

  /* Get the elapsed time since power up (in milliseconds).  This is a
   * bias value that we need to use to correct the base time.
   */

  (void)clock_systimespec(&bias);

  /* Add the base time to this.  The base time is the time-of-day
   * setting.  When added to the elapsed time since the time-of-day
   * was last set, this gives us the current time.
   */

  curr_ts.tv_sec  = bias.tv_sec + g_basetime.tv_sec;
  curr_ts.tv_nsec = bias.tv_nsec + g_basetime.tv_nsec;

  /* Handle carry to seconds. */

  if (curr_ts.tv_nsec >= NSEC_PER_SEC)
    {
      carry            = curr_ts.tv_nsec / NSEC_PER_SEC;
      curr_ts.tv_sec  += carry;
      curr_ts.tv_nsec -= (carry * NSEC_PER_SEC);
    }

  /* Check if RTC has advanced past system time. */

  if (curr_ts.tv_sec > rtc_time.tv_sec ||
      (curr_ts.tv_sec == rtc_time.tv_sec && curr_ts.tv_nsec >= rtc_time.tv_nsec))
    {
      /* Setting system time with RTC now would result time going
       * backwards. Skip resynchronization.
       */

      sinfo("skip resync\n");

      rtc_diff->tv_sec = 0;
      rtc_diff->tv_nsec = 0;
    }
  else
    {
      /* Save RTC time as the new base time. */

      g_basetime.tv_sec  = rtc_time.tv_sec;
      g_basetime.tv_nsec = rtc_time.tv_nsec;

      /* Subtract that bias from the basetime so that when the system
       * timer is again added to the base time, the result is the current
       * time relative to basetime.
       */

      if (g_basetime.tv_nsec < bias.tv_nsec)
        {
          g_basetime.tv_nsec += NSEC_PER_SEC;
          g_basetime.tv_sec--;
        }

      /* Result could be negative seconds */

      g_basetime.tv_nsec -= bias.tv_nsec;
      g_basetime.tv_sec  -= bias.tv_sec;

      sinfo("basetime=(%ld,%lu) bias=(%ld,%lu)\n",
            (long)g_basetime.tv_sec, (unsigned long)g_basetime.tv_nsec,
            (long)bias.tv_sec, (unsigned long)bias.tv_nsec);

      /* Output difference between time at entry and new current time. */

      rtc_diff->tv_sec = (bias.tv_sec + g_basetime.tv_sec) - curr_ts.tv_sec;
      rtc_diff->tv_nsec = (bias.tv_nsec + g_basetime.tv_nsec) - curr_ts.tv_nsec;

      /* Handle carry to seconds. */

      if (rtc_diff->tv_nsec < 0)
        {
          carry = -((-(rtc_diff->tv_nsec + 1)) / NSEC_PER_SEC + 1);
        }
      else if (rtc_diff->tv_nsec >= NSEC_PER_SEC)
        {
          carry = rtc_diff->tv_nsec / NSEC_PER_SEC;
        }
      else
        {
          carry = 0;
        }

      if (carry)
        {
          rtc_diff->tv_sec  += carry;
          rtc_diff->tv_nsec -= (carry * NSEC_PER_SEC);
        }

#ifdef CONFIG_CLOCK_MONOTONIC
      /* Monotonic clock follows wall time since system start-up. Adjust
       * CLOCK_MONOTONIC same amount as CLOCK_REALTIME.
       */

      g_monotonic_basetime.tv_sec  += (uint32_t)rtc_diff->tv_sec;
      g_monotonic_basetime.tv_nsec += (uint32_t)rtc_diff->tv_nsec;

      /* Handle carry to seconds. */

      if (g_monotonic_basetime.tv_nsec >= NSEC_PER_SEC)
        {
          carry = g_monotonic_basetime.tv_nsec / NSEC_PER_SEC;
          g_monotonic_basetime.tv_sec += carry;
          g_monotonic_basetime.tv_nsec -= (carry * NSEC_PER_SEC);
        }
#endif
    }

skip:
  leave_critical_section(flags);
}
#endif

/****************************************************************************
 * Name: clock_timer
 *
 * Description:
 *   This function must be called once every time the real time clock
 *   interrupt occurs.  The interval of this clock interrupt must be
 *   USEC_PER_TICK
 *
 ****************************************************************************/

#ifndef CONFIG_SCHED_TICKLESS
void clock_timer(void)
{
  /* Increment the per-tick system counter */

  g_system_timer++;
}
#endif
