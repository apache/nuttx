/****************************************************************************
 * sched/hrtimer/hrtimer_reload.c
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

#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_reload
 *
 * Description:
 *   Reload (reschedule) a high-resolution timer by advancing its expiration
 *   time by the specified interval in nanoseconds. This is typically used
 *   for periodic timers where the expiration must be continuously moved
 *   forward after each trigger.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the high-resolution timer instance to be updated.
 *             Must be a valid and active timer.
 *   ns      - Interval in nanoseconds to add to the timer's expiration time.
 *             Must not exceed HRTIMER_MAX_INCREMENT.
 *
 * Returned Value:
 *   OK (0) is returned on success.
 *   A negated errno value is returned on failure:
 *     -EINVAL: Invalid parameters (NULL pointer or interval too large).
 *
 * Assumptions:
 *   - Called in a context where it is safe to manipulate the timer queue.
 *   - Caller must ensure that the timer has been started and is associated
 *     with a valid upper-half timer queue.
 *   - This function may update the hardware expiration time if the reloaded
 *     timer becomes the earliest scheduled timer in the queue.
 *
 ****************************************************************************/

int hrtimer_reload(FAR struct hrtimer_s *hrtimer, uint64_t ns)
{
  FAR struct hrtimer_upperhalf_s *upper = hrtimer->upper;
  int ret = OK;

  if (upper == NULL ||
      ns > HRTIMER_MAX_INCREMENT)
    {
      return -EINVAL;
    }

  /* Advance the expiration time */

  hrtimer->node.expiration_time += ns;

  /* Update hardware expiration if this is the earliest timer */

  if ((RB_INSERT(hrtimer_tree, &upper->tree, &hrtimer->node) == NULL) &&
      (&hrtimer->node == RB_MIN(hrtimer_tree, &upper->tree)))
    {
      ret = hrtimer_setexpire(upper, hrtimer->node.expiration_time);
    }

  return ret;
}
