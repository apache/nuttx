/****************************************************************************
 * sched/hrtimer/hrtimer_start.c
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

#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_insert
 *
 * Description:
 *   Insert the given high-resolution timer into the active timer RB-tree.
 *   If the timer is already in the tree, it will be replaced.
 *   If the inserted timer becomes the earliest timer in the tree, the
 *   hardware timer will be configured to fire at its expiration.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the hrtimer structure to be inserted.
 *
 * Returned Value:
 *   OK (0) on success; negated errno value on failure.
 *
 * Assumptions/Notes:
 *   - This function should be called with interrupts disabled or under
 *     spinlock protection to ensure RB-tree integrity.
 *   - If the timer is currently running, insertion is rejected with -EBUSY.
 ****************************************************************************/

static inline int hrtimer_insert(FAR hrtimer_t *hrtimer)
{
  DEBUGASSERT(hrtimer != NULL);

  /* Insert (or replace) the timer into the RB-tree ordered by expiration */

  RB_INSERT(hrtimer_tree_s, &g_hrtimer_tree, &hrtimer->node);

  /* Mark the timer as armed */

  hrtimer->state = HRTIMER_STATE_ARMED;

  /* If the inserted timer is now the earliest, start hardware timer */

  if (&hrtimer->node == RB_MIN(hrtimer_tree_s, &g_hrtimer_tree))
    {
      return hrtimer_starttimer(hrtimer->expired);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_start
 *
 * Description:
 *   Start a high-resolution timer to expire after a specified duration
 *   in nanoseconds, either as an absolute or relative time.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the hrtimer structure.
 *   ns      - Expiration time in nanoseconds. Interpretation
 *             depends on mode.
 *   mode    - Timer mode (HRTIMER_MODE_ABS or HRTIMER_MODE_REL).
 *
 * Returned Value:
 *   OK (0) on success, or a negated errno value on failure.
 *
 * Assumptions/Notes:
 *   - This function disables interrupts briefly via spinlock to safely
 *     insert the timer into the RB-tree.
 *   - Absolute mode sets the timer to expire at the given absolute time.
 *   - Relative mode sets the timer to expire after 'ns'
 *     nanoseconds from the current time.
 ****************************************************************************/

int hrtimer_start(FAR hrtimer_t *hrtimer,
                  uint64_t ns,
                  enum hrtimer_mode_e mode)
{
  irqstate_t flags;
  int ret = OK;

  DEBUGASSERT(hrtimer != NULL);

  /* Protect RB-tree manipulation with spinlock and disable interrupts */

  flags = spin_lock_irqsave(&g_hrtimer_spinlock);

  /* Reject start if the timer is already running or armed */

  if ((hrtimer->state == HRTIMER_STATE_RUNNING) ||
      (hrtimer->state == HRTIMER_STATE_ARMED))
    {
      spin_unlock_irqrestore(&g_hrtimer_spinlock, flags);
      return -EBUSY;
    }

  /* Compute absolute expiration time */

  if (mode == HRTIMER_MODE_ABS)
    {
      hrtimer->expired = ns;
    }
  else
    {
      hrtimer->expired = hrtimer_gettime() + ns;
    }

  /* Insert the timer into the RB-tree */

  ret = hrtimer_insert(hrtimer);

  spin_unlock_irqrestore(&g_hrtimer_spinlock, flags);
  return ret;
}
