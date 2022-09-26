/****************************************************************************
 * sched/clock/clock_systime_ticks.c
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

/* 32-bit mask for 64-bit timer values */

#define TIMER_MASK32 0x00000000ffffffff

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
  /* Do we have a high-resolution RTC that can provide us with the time? */

  if (g_rtc_enabled)
    {
      struct timespec ts;

      /* Get the time from the platform specific hardware */

      clock_systime_timespec(&ts);

      /* Convert to a 64-bit value in microseconds,
       * then in clock tick units.
       */

      return timespec_to_tick(&ts);
    }
  else
#endif
    {
      /* In tickless mode, all timing is controlled by platform-specific
       * code.  Let the platform timer do the work.
       */

#if defined(CONFIG_SCHED_TICKLESS_TICK_ARGUMENT)
      clock_t ticks;
      up_timer_gettick(&ticks);
      return ticks;
#elif defined(CONFIG_SCHED_TICKLESS)
      struct timespec ts;
      up_timer_gettime(&ts);
      return timespec_to_tick(&ts);
#elif defined(CONFIG_SYSTEM_TIME64)

      clock_t sample;
      clock_t verify;

      /* 64-bit accesses are not atomic on most architectures.  The following
       * loop samples the 64-bit timer twice and loops in the rare event that
       * there was 32-bit rollover between samples.
       *
       * If there is no 32-bit rollover, then:
       *
       *  - The MS 32-bits of each sample will be the same, and
       *  - The LS 32-bits of the second sample will be greater than or equal
       *    to the LS 32-bits for the first sample.
       */

      do
        {
          verify = g_system_ticks;
          sample = g_system_ticks;
        }
      while ((sample &  TIMER_MASK32)  < (verify &  TIMER_MASK32) ||
             (sample & ~TIMER_MASK32) != (verify & ~TIMER_MASK32));

      return sample;

#else /* CONFIG_SYSTEM_TIME64 */

      /* Return the current system time */

      return g_system_ticks;

#endif /* CONFIG_SYSTEM_TIME64 */
    }
}
