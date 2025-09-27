/****************************************************************************
 * sched/hrtimer/hrtimer_reload.c
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

#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_reload
 *
 * Description:
 *   Reload a high-resolution timer by extending its expiration time with
 *   the specified interval.  This API is mainly used for periodic timers
 *   where the expiration time must be advanced after each timeout to
 *   schedule the next event.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the high-resolution timer instance to be reloaded
 *   time    - The interval (in timer ticks) to add to the expiration time
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A negated errno value is returned
 *   on failure.
 *
 * Assumptions:
 *   Called from a context where it is safe to manipulate timer structures.
 *
 ****************************************************************************/

int hrtimer_reload(FAR struct hrtimer_s *hrtimer, clock_t time)
{
  FAR struct hrtimer_upperhalf_s *upper = hrtimer->upper;
  int ret = OK;

  if (upper == NULL || upper->lower == NULL ||
      time > HRTIMER_MAX_INCREMENT)
    {
      return -EINVAL;
    }

  /* Advance the expiration time */

  hrtimer->node.expiration_time += time;

  /* Update hardware expiration if this is the earliest timer */

  if ((RB_INSERT(hrtimer_tree, &upper->tree, &hrtimer->node) == NULL) &&
      (&hrtimer->node == RB_MIN(hrtimer_tree, &upper->tree)))
    {
      ret = hrtimer_setexpire(upper, hrtimer->node.expiration_time);
    }

  return ret;
}
