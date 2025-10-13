/****************************************************************************
 * sched/clock/clock_systime_ticks.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include "clock/clock.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* See nuttx/clock.h */

#undef clock_systime_ticks

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_systime_ticks
 *
 * Description:
 *   Return the current value of the 32/64-bit system timer counter.
 *
 *   Indirect access to the system timer counter is required through this
 *   function if the execution environment does not have direct access to
 *   kernel global data.
 *
 *   Use of this function is also required to assure atomic access to the
 *   64-bit system timer.
 *
 *   NOTE:  This is an internal OS interface and should not be called from
 *   application code.  Rather, the functionally equivalent, standard
 *   interface clock() should be used.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current value of the system timer counter
 *
 ****************************************************************************/

clock_t clock_systime_ticks(void)
{
#ifdef CONFIG_RTC_HIRES
  struct timespec ts =
    {
      0
    };

  clock_systime_timespec(&ts);
  return clock_time2ticks_floor(&ts);
#elif defined(CONFIG_ALARM_ARCH) || \
      defined(CONFIG_TIMER_ARCH) || \
      defined(CONFIG_SCHED_TICKLESS)
  clock_t ticks = 0;

  up_timer_gettick(&ticks);
  return ticks;
#else
  return clock_get_sched_ticks();
#endif
}
