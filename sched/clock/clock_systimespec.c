/****************************************************************************
 * sched/clock/clock_systimespec.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_systimespec
 *
 * Description:
 *   Return the current value of the system timer counter as a struct
 *   timespec.
 *
 * Parameters:
 *   ts - Location to return the time
 *
 * Return Value:
 *   Current version always returns OK
 *
 * Assumptions:
 *
 ****************************************************************************/

int clock_systimespec(FAR struct timespec *ts)
{
#ifdef CONFIG_RTC_HIRES
  /* Do we have a high-resolution RTC that can provide us with the time? */

  if (g_rtc_enabled)
    {
      /* Get the hi-resolution time from the RTC */

      return up_rtc_gettime(ts);
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
      /* 64-bit microsecond calculations should improve our accuracy. */

      uint64_t usecs;
      uint64_t secs;
      uint64_t nsecs;

      /* Get the time since power-on in seconds and milliseconds */

      usecs = TICK2MSEC(clock_systimer());
      secs  = usecs / USEC_PER_SEC;

      /* Return the elapsed time in seconds and nanoseconds */

      nsecs = (usecs - (secs * USEC_PER_SEC)) * NSEC_PER_USEC;

      ts->tv_sec  = (time_t)secs;
      ts->tv_nsec = (long)nsecs;
      return OK;

#else
      /* 32-bit millisecond calculations should be just fine. */

      uint32_t msecs;
      uint32_t secs;
      uint32_t nsecs;

      /* Get the time since power-on in seconds and milliseconds */

      msecs = TICK2MSEC(clock_systimer());
      secs  = msecs / MSEC_PER_SEC;

      /* Return the elapsed time in seconds and nanoseconds */

      nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;

      ts->tv_sec  = (time_t)secs;
      ts->tv_nsec = (long)nsecs;
      return OK;
#endif
    }
}
