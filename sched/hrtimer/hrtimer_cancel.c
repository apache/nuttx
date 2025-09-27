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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_cancel
 *
 * Description:
 *   Cancel a high-resolution timer.  The timer is removed from its queue.
 *   If the canceled timer was the earliest in the queue, then the queue
 *   expiration is adjusted to:
 *
 *   1. The next earliest timer in the queue, or
 *   2. A default expiration value (current time + default increment)
 *      if the queue becomes empty.
 *
 * Input Parameters:
 *   hrtimer - High-resolution timer instance to be canceled
 *
 * Returned Value:
 *   OK (0) is returned on success.  A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int hrtimer_cancel(FAR struct hrtimer_s *hrtimer)
{
  FAR struct hrtimer_upperhalf_s *upper;
  FAR struct hrtimer_node        *node;
  irqstate_t flags;
  clock_t now;
  clock_t expiration_time;
  int ret = OK;

  DEBUGASSERT(hrtimer != NULL);

  /* Get the queue associated with this timer */

  upper = hrtimer->upper;
  if (upper == NULL || upper->lower == NULL)
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&upper->lock);

  /* Get the current earliest timer */

  node = RB_MIN(hrtimer_tree, &upper->tree);

  /* Remove the timer from the queue */

  RB_REMOVE(hrtimer_tree, &upper->tree, &hrtimer->node);

  /* If the removed timer was the earliest, recalculate queue expiration */

  if (node == &hrtimer->node)
    {
      /* Get the new earliest timer in the queue */

      node = RB_MIN(hrtimer_tree, &upper->tree);
      if (node != NULL)
        {
          /* Update queue expiration to the next earliest timer */

          ret = hrtimer_setexpire(upper, node->expiration_time);
        }
      else
        {
          /* No timers remain: set expiration to a safe default value */

          now             = HRTIMER_CURRENT(upper->lower);
          expiration_time = now + HRTIMER_DEFAULT_INCREMENT;

          ret = hrtimer_setexpire(upper, expiration_time);
        }
    }

  spin_unlock_irqrestore(&upper->lock, flags);
  return ret;
}
