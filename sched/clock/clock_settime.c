/****************************************************************************
 * sched/clock/clock_settime.c
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

#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include "clock/clock.h"
#ifdef CONFIG_CLOCK_TIMEKEEPING
#  include "clock/clock_timekeeping.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_settime
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 ****************************************************************************/

int clock_settime(clockid_t clock_id, FAR const struct timespec *tp)
{
#ifndef CONFIG_CLOCK_TIMEKEEPING
  struct timespec bias;
  irqstate_t flags;
#endif
  int ret = OK;

  sinfo("clock_id=%d\n", clock_id);
  DEBUGASSERT(tp != NULL);

  /* CLOCK_REALTIME - POSIX demands this to be present. This is the wall
   * time clock.
   */

  if (clock_id == CLOCK_REALTIME &&
      tp->tv_nsec >= 0 && tp->tv_nsec < 1000000000)
    {
#ifndef CONFIG_CLOCK_TIMEKEEPING
      /* Interrupts are disabled here so that the in-memory time
       * representation and the RTC setting will be as close as
       * possible.
       */

      flags = enter_critical_section();

      /* Get the elapsed time since power up (in milliseconds).  This is a
       * bias value that we need to use to correct the base time.
       */

      clock_systime_timespec(&bias);

      /* Save the new base time. */

      g_basetime.tv_sec  = tp->tv_sec;
      g_basetime.tv_nsec = tp->tv_nsec;

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

      /* Setup the RTC (lo- or high-res) */

#ifdef CONFIG_RTC
      if (g_rtc_enabled)
        {
          up_rtc_settime(tp);
        }
#endif

      leave_critical_section(flags);

      sinfo("basetime=(%ld,%lu) bias=(%ld,%lu)\n",
            (long)g_basetime.tv_sec, (unsigned long)g_basetime.tv_nsec,
            (long)bias.tv_sec, (unsigned long)bias.tv_nsec);
#else
      ret = clock_timekeeping_set_wall_time(tp);
#endif
    }
  else
    {
      serr("Returning ERROR\n");
      set_errno(EINVAL);
      ret = ERROR;
    }

  return ret;
}
