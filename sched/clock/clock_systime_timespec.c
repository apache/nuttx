/****************************************************************************
 * sched/clock/clock_systime_timespec.c
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

#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include "clock/clock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_systime_timespec
 *
 * Description:
 *   Return the current value of the system timer counter as a struct
 *   timespec.  The returned time is the elapsed time since power up.
 *
 * Input Parameters:
 *   ts - Location to return the time
 *
 * Returned Value:
 *   Current version almost always returns OK. Currently errors are
 *   possible with CONFIG_RTC_HIRES only.
 *
 * Assumptions:
 *
 ****************************************************************************/

int clock_systime_timespec(FAR struct timespec *ts)
{
#ifdef CONFIG_RTC_HIRES
  /* Do we have a high-resolution RTC that can provide us with the time? */

  if (g_rtc_enabled)
    {
      int ret;

      /* Get the hi-resolution time from the RTC.  This will return the
       * current time, not the time since power up.
       */

      ret = up_rtc_gettime(ts);
      if (ret < 0)
        {
          return ret;
        }

      /* Subtract the base time to this in order to convert this to the
       * time since power up.
       */

      DEBUGASSERT(ts->tv_sec >= g_basetime.tv_sec);
      if (ts->tv_sec < g_basetime.tv_sec)
        {
          /* Negative times are not supported */

          return -ENOSYS;
        }

      ts->tv_sec -= g_basetime.tv_sec;
      if (ts->tv_nsec < g_basetime.tv_nsec)
        {
          /* Borrow */

          if (ts->tv_sec < 1)
            {
              /* Negative times are not supported */

              return -ENOSYS;
            }

          ts->tv_sec--;
          ts->tv_nsec += NSEC_PER_SEC;
        }

      ts->tv_nsec -= g_basetime.tv_nsec;
      return OK;
    }
  else
#endif
    {
#if defined(CONFIG_SCHED_TICKLESS)
      /* In tickless mode, all timing is controlled by platform-specific
       * code.  Let the platform timer do the work.
       */

      return up_timer_gettime(ts);

#elif defined(CONFIG_HAVE_LONG_LONG) && (CONFIG_USEC_PER_TICK % 1000) != 0
      /* 64-bit microsecond calculations should improve our accuracy
       * when the clock period is in units of microseconds.
       */

      uint64_t usecs;
      uint64_t secs;
      uint64_t nsecs;

      /* Get the time since power-on in seconds and microseconds.
       * NOTE that overflow is still possible if we use a 64-bit
       * timer.
       */

      usecs = (uint64_t)TICK2USEC(clock_systime_ticks());
      secs  = usecs / USEC_PER_SEC;

      /* Return the elapsed time in seconds and nanoseconds */

      nsecs = (usecs - (secs * USEC_PER_SEC)) * NSEC_PER_USEC;

      ts->tv_sec  = (time_t)secs;
      ts->tv_nsec = (long)nsecs;
      return OK;

#else
      /* We know that the clock rate is in units of milliseconds
       * show we should be able to do the calculations with less
       * chance of overflow.
       *
       * 32-bit millisecond calculations should be just fine in
       * most cases.  For a 32-bit system timer and a clock period
       * of 10 milliseconds, the msecs value will overflow at about
       * 49.7 days.
       *
       * So.. we will still use 64-bit calculations if we have them
       * in order to avoid that limitation.
       */

#ifdef CONFIG_HAVE_LONG_LONG
      uint64_t msecs;
      uint64_t secs;
      uint64_t nsecs;
#define WIDE_CAST (uint64_t)
#else
      clock_t msecs;
      clock_t secs;
      clock_t nsecs;
#define WIDE_CAST
#endif

      /* Get the time since power-on in seconds and milliseconds */

      msecs = TICK2MSEC(WIDE_CAST clock_systime_ticks());
      secs  = msecs / MSEC_PER_SEC;

      /* Return the elapsed time in seconds and nanoseconds */

      nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;

      ts->tv_sec  = (time_t)secs;
      ts->tv_nsec = (long)nsecs;
      return OK;
#endif
    }
}
