/****************************************************************************
 * sched/hrtimer/hrtimer_process.c
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
#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_process
 *
 * Description:
 *   Process all expired high-resolution timers. This function repeatedly
 *   retrieves the earliest timer from the active timer RB-tree, checks if it
 *   has expired relative to the current time, removes it from the tree,
 *   and invokes its callback function. Processing continues until:
 *
 *     1. No additional timers have expired, or
 *     2. The active timer set is empty.
 *
 *   After all expired timers are processed, the next expiration event is
 *   scheduled based on:
 *
 *     - The earliest remaining timer, or
 *     - A fallback expiration (current time + HRTIMER_DEFAULT_INCREMENT)
 *       if no timers remain.
 *
 * Input Parameters:
 *   now - Current high-resolution timestamp.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Notes:
 *   - This function acquires a spinlock to protect the timer RB-tree.
 *   - Timer callbacks are invoked with interrupts enabled
 *     to avoid deadlocks.
 *   - DEBUGASSERT ensures that timer callbacks are valid.
 ****************************************************************************/

void hrtimer_process(uint64_t now)
{
  FAR hrtimer_t *hrtimer;
  irqstate_t flags;
  hrtimer_cb func;
  uint64_t expired;
  uint64_t period;
  int cpu = this_cpu();

  /* Lock the hrtimer RB-tree to protect access */

  flags = spin_lock_irqsave(&g_hrtimer_spinlock);

  /* Fetch the earliest active timer */

  hrtimer = hrtimer_get_first();

  while (hrtimer != NULL)
    {
      func = hrtimer->func;

      /* Ensure the timer callback is valid */

      DEBUGASSERT(func != NULL);

      expired = hrtimer->expired;

      /* Check if the timer has expired */

      if (!clock_compare(expired, now))
        {
          break;
        }

      /* Remove the expired timer from the active tree */

      hrtimer_remove(hrtimer);

      g_hrtimer_running[cpu] = hrtimer;

      /* Leave critical section before invoking the callback */

      spin_unlock_irqrestore(&g_hrtimer_spinlock, flags);

      /* Invoke the timer callback */

      period = func(hrtimer, expired);

      /* Re-enter critical section to update timer state */

      flags = spin_lock_irqsave(&g_hrtimer_spinlock);

      g_hrtimer_running[cpu] = NULL;

      /* If the timer is periodic and has not been rearmed or
       * cancelled concurrently,
       * compute next expiration and reinsert into RB-tree
       */

      if (period > 0 && hrtimer->expired == expired)
        {
          hrtimer->expired += period;

          /* Ensure no overflow occurs */

          DEBUGASSERT(hrtimer->expired > period);

          hrtimer_insert(hrtimer);
        }

      /* Fetch the next earliest timer */

      hrtimer = hrtimer_get_first();
    }

  /* Schedule the next timer expiration */

  if (hrtimer != NULL)
    {
      /* Start timer for the next earliest expiration */

      (void)hrtimer_starttimer(hrtimer->expired);
    }

  /* Leave critical section */

  spin_unlock_irqrestore(&g_hrtimer_spinlock, flags);
}
