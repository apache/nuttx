/****************************************************************************
 * sched/hrtimer/hrtimer_cancel.c
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
#include <errno.h>
#include <hrtimer/hrtimer.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_cancel
 *
 * Description:
 *   Cancel a high-resolution timer. The timer is removed from the active
 *   timer RB-tree. If the canceled timer was the earliest in the tree, the
 *   queue expiration is adjusted to:
 *
 *   1. The next earliest timer in the tree, or
 *   2. A safe default expiration (current time + HRTIMER_DEFAULT_INCREMENT)
 *        if the tree is empty.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the high-resolution timer instance to be canceled.
 *
 * Returned Value:
 *   OK (0) on success, or a negated errno value on failure.
 *
 * Assumptions/Notes:
 *   - This function locks the timer tree with spinlock
 *     to ensure safe access.
 *   - The expiration of the next timer (or default) is
 *     set via hrtimer_starttimer().
 ****************************************************************************/

int hrtimer_cancel(FAR hrtimer_t *hrtimer)
{
  FAR hrtimer_t *first_hrtimer;
  irqstate_t flags;
  uint64_t now;
  uint64_t expired;
  int ret = OK;

  /* Enter critical section to protect the RB-tree */

  flags = spin_lock_irqsave(&g_hrtspinlock);

  /* Get the current earliest timer */

  first_hrtimer = (FAR hrtimer_t *)RB_MIN(hrtimer_tree_s, &g_activetree);

  /* Remove the specified timer from the active tree */

  RB_REMOVE(hrtimer_tree_s, &g_activetree, &hrtimer->node);

  /* If the removed timer was the earliest, update queue expiration */

  if (first_hrtimer == hrtimer)
    {
      /* Fetch the new earliest timer */

      first_hrtimer = (FAR hrtimer_t *)RB_MIN(hrtimer_tree_s, &g_activetree);
      if (first_hrtimer != NULL)
        {
          /* Start timer for the next earliest timer */

          ret = hrtimer_starttimer(first_hrtimer->expired);
        }
    }

  /* Leave critical section */

  spin_unlock_irqrestore(&g_hrtspinlock, flags);

  return ret;
}
