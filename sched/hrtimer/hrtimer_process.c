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
  uint32_t period = 0;
  irqstate_t flags;

  /* Lock the hrtimer RB-tree to protect access */

  flags = spin_lock_irqsave(&g_hrtimer_spinlock);

  /* Fetch the earliest active timer */

  hrtimer = (FAR hrtimer_t *)RB_MIN(hrtimer_tree_s, &g_hrtimer_tree);

  while (hrtimer != NULL)
    {
      /* Check if the timer has expired */

      if (!clock_compare(hrtimer->expired, now))
        {
          break;
        }

      /* Remove the expired timer from the active tree */

      RB_REMOVE(hrtimer_tree_s, &g_hrtimer_tree, &hrtimer->node);

      /* Ensure the timer callback is valid */

      DEBUGASSERT(hrtimer->func != NULL);

      hrtimer->state = HRTIMER_STATE_RUNNING;
      hrtimer->cpus++;

      /* cpus is a running reference counter and must never wrap */

      DEBUGASSERT(hrtimer->cpus != 0);

      /* Leave critical section before invoking the callback */

      spin_unlock_irqrestore(&g_hrtimer_spinlock, flags);

      /* Execute the timer callback */

      period = hrtimer->func(hrtimer);

      /* Re-enter critical section to update timer state */

      flags = spin_lock_irqsave(&g_hrtimer_spinlock);

      hrtimer->cpus--;

      switch (hrtimer->state)
        {
          case HRTIMER_STATE_RUNNING:
            {
              /* Timer callback completed normally */

              if (period > 0)
                {
                  /* Periodic timer: re-arm with the next expiration */

                  hrtimer->expired += period;
                  hrtimer->state = HRTIMER_STATE_ARMED;

                  RB_INSERT(hrtimer_tree_s, &g_hrtimer_tree,
                            &hrtimer->node);
                }
              else
                {
                  /* One-shot timer: deactivate when last instance ends */

                  if (hrtimer->cpus == 0)
                    {
                      hrtimer->state = HRTIMER_STATE_INACTIVE;
                    }
                }

              break;
            }

          case HRTIMER_STATE_CANCELED:
            {
              /* Timer was canceled during callback execution */

              hrtimer->state = HRTIMER_STATE_INACTIVE;
              break;
            }

          case HRTIMER_STATE_ARMED:
          default:
            break;
        }

      /* Fetch the next earliest timer */

      hrtimer = (FAR hrtimer_t *)RB_MIN(hrtimer_tree_s, &g_hrtimer_tree);
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
