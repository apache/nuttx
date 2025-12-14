/****************************************************************************
 * sched/sched/sched_processhrtimer.c
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
#include <nuttx/compiler.h>
#include <time.h>

#include "sched/sched.h"
#include "hrtimer/hrtimer.h"
#include "clock/clock.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Global high-resolution timer instance used by the scheduler */

static hrtimer_t g_nxsched_hrtimer;

/* Whether the scheduler high-resolution timer has been initialized */

static bool g_sched_hrtimer_inited = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_hrtimer_callback
 *
 * Description:
 *   High-resolution timer callback used for scheduler tick processing.
 *
 *   This callback updates the scheduler state and arms the next timer
 *   event. The behavior differs depending on whether tickless mode is
 *   enabled:
 *
 *   - In tickless mode, nxsched_tick_expiration() determines the next
 *     expiration point.
 *   - In non-tickless mode, a tick is generated unconditionally at
 *     every timer interval.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the high-resolution timer instance that expired.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static uint32_t nxsched_hrtimer_callback(FAR hrtimer_t *hrtimer)
{
#ifdef CONFIG_SCHED_TICKLESS
  uint64_t now = hrtimer_gettime();
  nxsched_tick_expiration(NSEC2TICK(now));
#else

  /* Process a single scheduler tick */

  nxsched_process_timer();

  /* Schedule the next tick relative to now */

  return NSEC_PER_TICK;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
int nxsched_hrtimer_start(clock_t ticks)
{
  return hrtimer_start(&g_nxsched_hrtimer,
                       hrtimer->expired + ticks * NSEC_PER_TICK,
                       HRTIMER_MODE_ABS);
}
#endif

/****************************************************************************
 * Name: nxsched_process_hrtimer
 *
 * Description:
 *   Process pending high-resolution timer events for the scheduler.
 *
 *   This function performs lazy initialization of the scheduler's
 *   high-resolution timer and processes expired timer events. It is intended
 *   to be invoked periodically from architecture-specific high-resolution
 *   timer handling code.
 *
 * Input Parameters:
 *   None.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void nxsched_process_hrtimer(void)
{
  uint64_t now = hrtimer_gettime();

  /* Perform one-time initialization */

  if (g_sched_hrtimer_inited == false)
    {
      g_sched_hrtimer_inited = true;

      /* Initialize the scheduler hrtimer */

      hrtimer_init(&g_nxsched_hrtimer,
                   nxsched_hrtimer_callback,
                   NULL);

      /* Start the first timer event */

      hrtimer_start(&g_nxsched_hrtimer,
                    now + NSEC_PER_TICK,
                    HRTIMER_MODE_ABS);
    }

  /* Process any expired high-resolution timer events */

  hrtimer_process(now);
}
