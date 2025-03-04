/****************************************************************************
 * sched/clock/clock_settime.c
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

#include <time.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>
#include <sys/time.h>

#include "clock/clock.h"
#ifdef CONFIG_CLOCK_TIMEKEEPING
#  include "clock/clock_timekeeping.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxclock_settime
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 *   CLOCK_REALTIME - POSIX demands this to be present. This is the wall
 *   time clock.
 *
 ****************************************************************************/

void nxclock_settime(clockid_t clock_id, FAR const struct timespec *tp)
{
#ifndef CONFIG_CLOCK_TIMEKEEPING
  struct timespec bias;
  irqstate_t flags;
#  ifdef CONFIG_CLOCK_ADJTIME
  const struct timeval zerodelta = {
    0, 0
  };
#  endif

  /* Interrupts are disabled here so that the in-memory time
   * representation and the RTC setting will be as close as
   * possible.
   */

  /* Get the elapsed time since power up (in milliseconds).  This is a
   * bias value that we need to use to correct the base time.
   */

  clock_systime_timespec(&bias);

  flags = spin_lock_irqsave(&g_basetime_lock);

  clock_timespec_subtract(tp, &bias, &g_basetime);

  spin_unlock_irqrestore(&g_basetime_lock, flags);

  /* Setup the RTC (lo- or high-res) */

#  ifdef CONFIG_RTC
  if (g_rtc_enabled)
    {
      up_rtc_settime(tp);
    }
#  endif

#  ifdef CONFIG_CLOCK_ADJTIME
  /* Cancel any ongoing adjustment */

  adjtime(&zerodelta, NULL);
#  endif
#else
  clock_timekeeping_set_wall_time(tp);
#endif
}

/****************************************************************************
 * Name: clock_settime
 *
 * Description:
 *   Clock Functions based on POSIX APIs
 *
 *   CLOCK_REALTIME - POSIX demands this to be present. This is the wall
 *   time clock.
 *
 ****************************************************************************/

int clock_settime(clockid_t clock_id, FAR const struct timespec *tp)
{
  if (clock_id != CLOCK_REALTIME || tp == NULL ||
      tp->tv_nsec < 0 || tp->tv_nsec >= 1000000000)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  nxclock_settime(clock_id, tp);
  return OK;
}
