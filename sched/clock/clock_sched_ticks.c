/****************************************************************************
 * sched/clock/clock_sched_ticks.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: clock_increase_sched_ticks
 *
 * Description:
 *   Increment the scheduler tick counter. This function should be called
 *   each time the real-time clock interrupt occurs, indicating the passage
 *   of one or more scheduling ticks.
 *
 * Input Parameters:
 *   ticks - The number of ticks to increment (typically 1)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void clock_increase_sched_ticks(clock_t ticks)
{
  /* Increment the per-tick scheduler counter */

#ifdef CONFIG_SYSTEM_TIME64
  atomic64_fetch_add((FAR atomic64_t *)&g_system_ticks, ticks);
#else
  atomic_fetch_add((FAR atomic_t *)&g_system_ticks, ticks);
#endif
}

/****************************************************************************
 * Name: clock_get_sched_ticks
 *
 * Description:
 *   Return the current value of the scheduler tick counter. This counter
 *   only increases while the scheduler is running, and is independent of
 *   the real-time clock.
 *
 * Returned Value:
 *   The current number of scheduler ticks.
 *
 ****************************************************************************/

clock_t clock_get_sched_ticks(void)
{
#ifdef CONFIG_SYSTEM_TIME64
  clock_t sample;
  clock_t verify;

  /* 64-bit accesses are not atomic on most architectures. The following
   * loop samples the 64-bit counter twice and retries in the rare case
   * that a 32-bit rollover occurs between samples.
   *
   * If no 32-bit rollover occurs:
   *  - The MS 32 bits of both samples will be identical, and
   *  - The LS 32 bits of the second sample will be greater than or equal
   *    to those of the first.
   */

  do
    {
      verify = g_system_ticks;
      sample = g_system_ticks;
    }
  while ((sample & TIMER_MASK32)  < (verify & TIMER_MASK32) ||
         (sample & ~TIMER_MASK32) != (verify & ~TIMER_MASK32));

  return sample;
#else
  /* On 32-bit systems, atomic access is guaranteed */

  return g_system_ticks;
#endif
}
