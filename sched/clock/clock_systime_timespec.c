/****************************************************************************
 * sched/clock/clock_systime_timespec.c
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

#include <stdint.h>
#include <time.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/spinlock.h>

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
  if (g_rtc_enabled)
    {
      irqstate_t flags;

      up_rtc_gettime(ts);

      flags = spin_lock_irqsave(&g_basetime_lock);
      clock_timespec_subtract(ts, &g_basetime, ts);
      spin_unlock_irqrestore(&g_basetime_lock, flags);
    }
  else
    {
      ts->tv_sec = 0;
      ts->tv_nsec = 0;
    }
#elif defined(CONFIG_ALARM_ARCH) || \
      defined(CONFIG_TIMER_ARCH) || \
      defined(CONFIG_SCHED_TICKLESS)
  up_timer_gettime(ts);
#else
  clock_ticks2time(ts, g_system_ticks);
#endif
  return 0;
}

