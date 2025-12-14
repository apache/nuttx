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
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <assert.h>
#include <hrtimer/hrtimer.h>

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
 *   ts - Pointer to the current high-resolution timestamp.
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
  uint64_t expired;
  irqstate_t flags;

  /* Lock the hrtimer RB-tree to protect access */

  flags = spin_lock_irqsave(&g_hrtspinlock);

  /* Fetch the earliest active timer */

  hrtimer = (FAR hrtimer_t *)RB_MIN(hrtimer_tree_s, &g_activetree);

  while (hrtimer != NULL)
    {
      /* Check if the timer has expired */

      if (!clock_compare(hrtimer->expired, now))
        {
          break;
        }

      /* Remove the expired timer from the active tree */

      RB_REMOVE(hrtimer_tree_s, &g_activetree, &hrtimer->node);

      /* Ensure the timer callback is valid */

      DEBUGASSERT(hrtimer->func != NULL);

      /* Unlock tree while calling the timer callback to prevent deadlock */

      spin_unlock_irqrestore(&g_hrtspinlock, flags);

      /* Invoke the timer callback */

      hrtimer->func(hrtimer);

      /* Re-lock the tree for further processing */

      flags = spin_lock_irqsave(&g_hrtspinlock);

      /* Fetch the next earliest timer */

      hrtimer = (FAR hrtimer_t *)RB_MIN(hrtimer_tree_s, &g_activetree);
    }

  /* Schedule the next timer expiration */

  if (hrtimer != NULL)
    {
      /* Start timer for the next earliest expiration */

      (void)hrtimer_starttimer(hrtimer->expired);
    }

  /* Unlock the timer tree */

  spin_unlock_irqrestore(&g_hrtspinlock, flags);
}
