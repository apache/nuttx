/****************************************************************************
 * sched/sched/sched_timer.c
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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <errno.h>

#include "sched/sched.h"
#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* High-resolution timer used to drive the scheduler tick.
 *
 * In non-tickless mode, this timer periodically generates a scheduler
 * tick with interval NSEC_PER_TICK.
 *
 * In tickless mode, the timer is still used, but the callback does not
 * request automatic re-arming.
 */

#ifdef CONFIG_HRTIMER
static hrtimer_t g_sched_hrtimer;
static bool g_sched_hrtimer_started = false;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_hrtimer_callback
 *
 * Description:
 *   High-resolution timer callback used to drive the scheduler.
 *
 *   This callback is invoked when the scheduler hrtimer expires.
 *   It advances the scheduler time base by calling
 *   nxsched_process_tick().
 *
 * Input Parameters:
 *   hrtimer - Pointer to the expired hrtimer instance
 *   expired - Expiration time in nanoseconds
 *
 * Returned Value:
 *   In non-tickless mode:
 *     Returns the next expiration interval (NSEC_PER_TICK),
 *     causing the hrtimer to be re-armed periodically.
 *
 *   In tickless mode:
 *     Returns 0 to indicate that the timer should not be
 *     automatically restarted.
 *
 ****************************************************************************/

#ifdef CONFIG_HRTIMER
static uint64_t
nxsched_hrtimer_callback(FAR const struct hrtimer_s *hrtimer,
                         uint64_t expired)
{
  UNUSED(hrtimer);
  UNUSED(expired);

  /* Advance scheduler time and process time slice expiration */

  nxsched_process_tick();

#ifndef CONFIG_SCHED_TICKLESS
  /* Periodic tick mode: re-arm timer with fixed tick interval */

  return NSEC_PER_TICK;
#else
  /* Tickless mode controls the next wakeup explicitly */

  return 0;
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_HRTIMER) && defined(CONFIG_SCHED_TICKLESS)
int nxsched_hrtimer_tick_start(clock_t tick)
{
  return hrtimer_start(&g_sched_hrtimer,
                       nxsched_hrtimer_callback,
                       tick * NSEC_PER_TICK,
                       HRTIMER_MODE_ABS);
}
#endif

/****************************************************************************
 * Name: nxsched_process_timer
 *
 * Description:
 *   Process scheduler timing events.
 *
 *   If high-resolution timers are enabled, this function dispatches
 *   expired hrtimers based on the current hrtimer time.
 *
 *   Otherwise, it falls back to directly processing a scheduler tick.
 *
 ****************************************************************************/

void nxsched_process_timer(void)
{
#ifdef CONFIG_HRTIMER
  /* Process all expired high-resolution timers */

  if (g_sched_hrtimer_started == false)
    {
      g_sched_hrtimer_started = true;
      hrtimer_start(&g_sched_hrtimer,
                    nxsched_hrtimer_callback,
                    NSEC_PER_TICK,
                    HRTIMER_MODE_REL);
    }

  hrtimer_process(clock_systime_nsec());
#else
  /* Fallback: process one scheduler tick */

  nxsched_process_tick();
#endif
}
