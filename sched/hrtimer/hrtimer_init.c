/****************************************************************************
 * sched/hrtimer/hrtimer_init.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with
 * the License.  You may obtain a copy of the License at
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

#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Global high-resolution default timer queue.
 *
 * This queue is used by the system to manage all default high-resolution
 * timers.  It is initialized with a fixed heap storage and associated with
 * a hardware timer lower when started.
 */

struct hrtimer_upperhalf_s g_default_hrtimer_upperhalf =
{
  .lower   = NULL,
  .started = false,
  .lock    = SP_UNLOCKED,
};

/* Global SysTick timer instance used for scheduler tick handling */

FAR struct hrtimer_s g_hrtimer_systick_timer;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_systick_callback
 *
 * Description:
 *   Callback function for the SysTick-based high-resolution timer.  It
 *   reloads the timer to trigger after the next tick interval and invokes
 *   the scheduler's tick processing logic.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the SysTick hrtimer instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Executed in timer interrupt or tick processing context.
 *
 ****************************************************************************/

static void hrtimer_systick_callback(FAR struct hrtimer_s *hrtimer)
{
#ifdef CONFIG_SCHED_TICKLESS
  clock_t now = 0;

  ONESHOT_TICK_CURRENT(hrtimer->upper->lower, &now);
  nxsched_alarm_tick_expiration(now);
#else
  /* Reload SysTick timer to trigger after the next system tick interval */

  hrtimer_reload(hrtimer,
                 HRTIMER_USEC2TIME(hrtimer->upper->lower, USEC_PER_TICK));

  /* Process scheduler tick events */

  nxsched_process_timer();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_upper_start
 *
 * Description:
 *   Start a high-resolution timer queue.  If the queue has already been
 *   started, this function simply returns.  Otherwise, the queue is marked
 *   as started.  For the global default queue, this function will also
 *   schedule the SysTick timer to expire after one system tick.  Finally,
 *   the associated hardware timer lower is started.
 *
 * Input Parameters:
 *   upper - Pointer to the high-resolution timer queue to be started
 *
 * Returned Value:
 *   OK (0) is returned on success.  A negated errno value is returned
 *   on failure.
 *
 * Assumptions:
 *   Caller must ensure that the lower driver has been properly
 *   initialized before invoking this function.
 *
 ****************************************************************************/

int hrtimer_upper_start(FAR struct hrtimer_upperhalf_s *upper)
{
  int ret = OK;

  DEBUGASSERT(upper != NULL);

  /* Return immediately if the queue is already started or invalid */

  if (upper->lower == NULL || upper->started == true)
    {
      return -EINVAL;
    }

  RB_INIT(&upper->tree);
  upper->started = true;

  /* If this is the global default queue, schedule the SysTick timer */

  if (upper == &g_default_hrtimer_upperhalf)
    {
      hrtimer_init(&g_hrtimer_systick_timer,
                   hrtimer_systick_callback, NULL);

      ret = hrtimer_start(&g_hrtimer_systick_timer,
                          NSEC_PER_TICK,
                          HRTIMER_MODE_REL);
    }

  /* Start the hardware timer via its lower operations */

  if (ret == OK)
    {
      ret = HRTIMER_START(upper->lower);
    }

  return ret;
}

/* Generate red-black tree implementation for high-resolution timers */

RB_GENERATE(hrtimer_tree, hrtimer_node, entry, hrtimer_cmp);
