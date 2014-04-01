/************************************************************************
 * sched/clock_gettime.c
 *
 *   Copyright (C) 2007, 2009, 2011, 2014 Gregory Nutt. All rights reserved.
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
#include <nuttx/rtc.h>

#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <arch/irq.h>

#include "clock_internal.h"

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Private Function Prototypes
 ************************************************************************/

/**********************************************************************
 * Public Constant Data
 **********************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/

/**********************************************************************
 * Private Variables
 **********************************************************************/

/************************************************************************
 * Private Functions
 ************************************************************************/

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: clock_gettime
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 ************************************************************************/

int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
#ifdef CONFIG_SYSTEM_TIME64
  uint64_t msecs;
  uint64_t secs;
  uint64_t nsecs;
#else
  uint32_t msecs;
  uint32_t secs;
  uint32_t nsecs;
#endif
  uint32_t carry;
  int ret = OK;

  sdbg("clock_id=%d\n", clock_id);
  DEBUGASSERT(tp != NULL);

#ifdef CLOCK_MONOTONIC
  /* CLOCK_MONOTONIC is an optional under POSIX: "If the Monotonic Clock
   * option is supported, all implementations shall support a clock_id
   * of CLOCK_MONOTONIC defined in <time.h>. This clock represents the
   * monotonic clock for the system. For this clock, the value returned
   * by clock_gettime() represents the amount of time (in seconds and
   * nanoseconds) since an unspecified point in the past (for example,
   * system start-up time, or the Epoch). This point does not change
   * after system start-up time. The value of the CLOCK_MONOTONIC clock
   * cannot be set via clock_settime(). This function shall fail if it
   * is invoked with a clock_id argument of CLOCK_MONOTONIC."
   */

  if (clock_id == CLOCK_MONOTONIC)
    {
      /* Get the time since power-on in seconds and milliseconds */

      msecs = MSEC_PER_TICK * g_system_timer;
      secs  = msecs / MSEC_PER_SEC;

      /* Return the elapsed time in seconds and nanoseconds */

      nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;

      tp->tv_sec  = (time_t)secs;
      tp->tv_nsec = (long)nsecs;
    }
  else
#endif

  /* CLOCK_REALTIME - POSIX demands this to be present.  CLOCK_REALTIME
   * represents the machine's best-guess as to the current wall-clock,
   * time-of-day time. This means that CLOCK_REALTIME can jump forward and
   * backward as the system time-of-day clock is changed.
   *
   * If an RTC is supported, then the non-standard CLOCK_ACTIVETIME is also
   * supported to manage time based on the system timer interrupt separately
   * from the RTC.  This may be necessary, for example, in certain cases where
   * the system timer interrupt has been stopped in low power modes.
   */

#ifdef CONFIG_RTC
  if (clock_id == CLOCK_REALTIME || clock_id == CLOCK_ACTIVETIME)
#else
  if (clock_id == CLOCK_REALTIME)
#endif
    {
      /* Do we have a high-resolution RTC that can provide us with the time? */

#ifdef CONFIG_RTC_HIRES
      if (g_rtc_enabled && clock_id != CLOCK_ACTIVETIME)
        {
          /* Yes.. Get the hi-resolution time from the RTC unless the caller
           * has specifically asked for the system timer (CLOCK_ACTIVETIME)
           */

          ret = up_rtc_gettime(tp);
        }
      else
#endif
        {
          /* Get the elapsed time since the time-of-day was last set.
           * g_system_timer provides the number of clock times since
           * power was applied; the bias value corresponds to the time
           * when the time-of-day was last set.
           */

          msecs = MSEC_PER_TICK * (g_system_timer - g_tickbias);

          sdbg("msecs = %d g_tickbias=%d\n",
               (int)msecs, (int)g_tickbias);

          /* Get the elapsed time in seconds and nanoseconds. */

          secs  = msecs / MSEC_PER_SEC;
          nsecs = (msecs - (secs * MSEC_PER_SEC)) * NSEC_PER_MSEC;

          sdbg("secs = %d + %d nsecs = %d + %d\n",
               (int)msecs, (int)g_basetime.tv_sec,
               (int)nsecs, (int)g_basetime.tv_nsec);

          /* Add the base time to this.  The base time is the time-of-day
           * setting.  When added to the elapsed time since the time-of-day
           * was last set, this gives us the current time.
           */

          secs  += (uint32_t)g_basetime.tv_sec;
          nsecs += (uint32_t)g_basetime.tv_nsec;

          /* Handle carry to seconds. */

          if (nsecs > NSEC_PER_SEC)
            {
              carry  = nsecs / NSEC_PER_SEC;
              secs  += carry;
              nsecs -= (carry * NSEC_PER_SEC);
            }

          /* And return the result to the caller. */

          tp->tv_sec  = (time_t)secs;
          tp->tv_nsec = (long)nsecs;
        }

      sdbg("Returning tp=(%d,%d)\n", (int)tp->tv_sec, (int)tp->tv_nsec);
    }
  else
    {
      sdbg("Returning ERROR\n");

      set_errno(EINVAL);
      ret = ERROR;
    }

  return ret;
}
