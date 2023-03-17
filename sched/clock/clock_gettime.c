/****************************************************************************
 * sched/clock/clock_gettime.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spinlock.h>

#include "clock/clock.h"
#ifdef CONFIG_CLOCK_TIMEKEEPING
#  include "clock/clock_timekeeping.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_gettime
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 ****************************************************************************/

int clock_gettime(clockid_t clock_id, struct timespec *tp)
{
#ifndef CONFIG_CLOCK_TIMEKEEPING
  struct timespec ts;
  uint32_t carry;
#endif
  int ret = OK;

  DEBUGASSERT(tp != NULL);

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

  if (clock_id == CLOCK_MONOTONIC || clock_id == CLOCK_BOOTTIME)
    {
      /* The the time elapsed since the timer was initialized at power on
       * reset.
       */

      ret = clock_systime_timespec(tp);
    }

  /* CLOCK_REALTIME - POSIX demands this to be present.  CLOCK_REALTIME
   * represents the machine's best-guess as to the current wall-clock,
   * time-of-day time. This means that CLOCK_REALTIME can jump forward and
   * backward as the system time-of-day clock is changed.
   */

  else if (clock_id == CLOCK_REALTIME)
    {
      /* Get the elapsed time since the time-of-day was last set.
       * clock_systime_timespec() provides the time since power was applied;
       * the bias value corresponds to the time when the time-of-day was
       * last set.
       */

#if defined(CONFIG_CLOCK_TIMEKEEPING)
      ret = clock_timekeeping_get_wall_time(tp);
#else
      ret = clock_systime_timespec(&ts);
      if (ret == OK)
        {
          irqstate_t flags;

          /* Add the base time to this.  The base time is the time-of-day
           * setting.  When added to the elapsed time since the time-of-day
           * was last set, this gives us the current time.
           */

          flags = spin_lock_irqsave(NULL);

          ts.tv_sec  += (uint32_t)g_basetime.tv_sec;
          ts.tv_nsec += (uint32_t)g_basetime.tv_nsec;

          spin_unlock_irqrestore(NULL, flags);

          /* Handle carry to seconds. */

          if (ts.tv_nsec >= NSEC_PER_SEC)
            {
              carry       = ts.tv_nsec / NSEC_PER_SEC;
              ts.tv_sec  += carry;
              ts.tv_nsec -= (carry * NSEC_PER_SEC);
            }

          /* And return the result to the caller. */

          tp->tv_sec  = ts.tv_sec;
          tp->tv_nsec = ts.tv_nsec;
        }
#endif /* CONFIG_CLOCK_TIMEKEEPING */
    }
  else
    {
      ret = -EINVAL;
    }

  /* Check for errors and set the errno value if necessary */

  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
