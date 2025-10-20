/****************************************************************************
 * sched/hrtimer/hrtimer_init.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with
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
 * This queue manages all default high-resolution timers.  It is initialized
 * statically and associated with a hardware timer lower when started.
 */

struct hrtimer_upperhalf_s g_default_hrtimer_upperhalf =
{
  .started = false,
  .lock    = SP_UNLOCKED,
};

/* Global SysTick timer instance */

FAR struct hrtimer_s g_hrtimer_systick_timer;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_systick_callback
 *
 * Description:
 *   Callback for the SysTick-based high-resolution timer.  In tickless mode,
 *   it signals the scheduler of tick expiration.  In tickful mode, it
 *   reloads the timer and processes scheduler tick events.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the SysTick timer instance
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Executed in interrupt or timer context.
 *
 ****************************************************************************/

static void hrtimer_systick_callback(FAR struct hrtimer_s *hrtimer)
{
#ifdef CONFIG_SCHED_TICKLESS
  uint64_t now = 0;

  up_alarm_gettick(&now);
  nxsched_alarm_tick_expiration(now);
#else
  /* Reload SysTick timer for the next tick */

  hrtimer_reload(hrtimer, NSEC_PER_TICK);

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
 *   started, this function returns an error.  Otherwise the queue is marked
 *   started, and if it is the global default queue, the SysTick timer is
 *   also scheduled.  Finally, the hardware timer lower is started.
 *
 * Input Parameters:
 *   upper   - Timer queue to be started
 *   clockid - Associated clock ID
 *
 * Returned Value:
 *   OK (zero) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   The lower driver must be initialized before calling this function.
 *
 ****************************************************************************/

int hrtimer_upper_start(FAR struct hrtimer_upperhalf_s *upper,
                        htimer_clockid_t clockid)
{
  int ret = OK;

  DEBUGASSERT(upper != NULL);

  /* Return error if already started */

  if (upper->started)
    {
      return -EINVAL;
    }

  RB_INIT(&upper->tree);
  upper->clockid = clockid;
  upper->started = true;

  /* If global default queue, schedule the SysTick timer */

  if (upper == &g_default_hrtimer_upperhalf)
    {
      hrtimer_init(&g_hrtimer_systick_timer,
                   hrtimer_systick_callback, NULL);

      ret = hrtimer_start(&g_hrtimer_systick_timer,
                          NSEC_PER_TICK,
                          HRTIMER_MODE_REL);
    }

  /* Start hardware timer via lower-half ops */

  if (ret == OK)
    {
      ret = hrtimer_clock_start(upper);
    }

  return ret;
}

/* Generate red-black tree implementation for high-resolution timers */

RB_GENERATE(hrtimer_tree, hrtimer_node, entry, hrtimer_cmp);
