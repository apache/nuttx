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
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_insert
 *
 * Description:
 *   Insert the given high-resolution timer into the active timer RB-tree.
 *   If the timer already exists, it will be removed and re-inserted.
 *   If the inserted timer is the earliest in the tree, start the hardware
 *   timer to fire at its expiration.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the hrtimer structure to be inserted.
 *
 * Returned Value:
 *   OK (0) on success, or a negated errno value on failure.
 *
 * Assumptions/Notes:
 *   - This function should be called with interrupts disabled or under
 *     spinlock protection to ensure RB-tree integrity.
 *   - Inline function for performance in critical path.
 ****************************************************************************/

static inline_function
int hrtimer_insert(FAR hrtimer_t *hrtimer)
{
  int ret = OK;
  FAR struct hrtimer_node_s *inserted =
    RB_INSERT(hrtimer_tree_s, &g_activetree, &hrtimer->node);

  if (inserted == NULL)
    {
      if (&hrtimer->node == RB_MIN(hrtimer_tree_s, &g_activetree))
        {
          /* If new timer is the earliest, start hardware timer */

          ret = hrtimer_starttimer(hrtimer->expired);
        }
    }
  else
    {
      ret = -EINVAL;
    }

  return ret;
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
 *     nanoseconds from now.
 ****************************************************************************/

int hrtimer_start(FAR hrtimer_t *hrtimer,
                  uint64_t ns,
                  enum hrtimer_mode_e mode)
{
  irqstate_t flags;
  int ret = OK;

  /* Compute expiration time based on mode */

  if (mode == HRTIMER_MODE_ABS)
    {
      hrtimer->expired = ns;
    }
  else
    {
      hrtimer->expired = hrtimer_gettime() + ns;
    }

  /* Insert the timer under spinlock protection */

  flags = spin_lock_irqsave(&g_hrtspinlock);
  ret = hrtimer_insert(hrtimer);
  spin_unlock_irqrestore(&g_hrtspinlock, flags);

  return ret;
}
