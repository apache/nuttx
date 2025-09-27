/****************************************************************************
 * sched/hrtimer/hrtimer_add.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <assert.h>

#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_upper_add_abs
 *
 * Description:
 *   Add a high-resolution timer to the queue using an absolute expiration
 *   time. If the timer is the earliest one in the queue, update the
 *   expiration time in the underlying hardware or system timer.
 *
 * Input Parameters:
 *   upper   - High-resolution timer upper-half driver instance
 *   hrtimer - High-resolution timer instance
 *   start   - Absolute expiration time (in ticks)
 *
 * Returned Value:
 *   Returns OK (zero) on success.
 *   Returns a negated errno value on failure.
 *
 * Assumptions:
 *   - The caller provides a valid 'upper' and 'hrtimer'
 *   - The caller holds no locks; locking is handled internally
 *
 ****************************************************************************/

int hrtimer_upper_add_abs(FAR struct hrtimer_upperhalf_s *upper,
                          FAR struct hrtimer_s *hrtimer,
                          clock_t start)
{
  irqstate_t flags;
  int ret = OK;
  FAR struct hrtimer_node_s *inserted;

  DEBUGASSERT(upper != NULL && upper->lower != NULL);

  /* Bind this timer to the queue */

  hrtimer->upper           = upper;
  hrtimer->node.expiration_time = start;

  /* Insert the timer into the RB-tree */

  flags = spin_lock_irqsave(&upper->lock);

  inserted = RB_INSERT(hrtimer_tree, &upper->tree, &hrtimer->node);
  if (inserted != NULL)
    {
      /* Duplicate entry detected */

      ret = -EINVAL;
    }
  else
    {
      /* If this timer is the earliest, update hardware expiration */

      if (&hrtimer->node == RB_MIN(hrtimer_tree, &upper->tree))
        {
          ret = hrtimer_setexpire(upper, start);
        }
    }

  spin_unlock_irqrestore(&upper->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: hrtimer_upper_add_rel
 *
 * Description:
 *   Add a high-resolution timer to the queue using a relative expiration
 *   time. The absolute expiration is calculated as the current time plus
 *   the specified increment. If the timer is the earliest one in the queue,
 *   update the expiration time in the underlying hardware or system timer.
 *
 * Input Parameters:
 *   upper     - High-resolution timer upper-half driver instance
 *   hrtimer   - High-resolution timer instance
 *   increment - Relative time interval (in ticks)
 *
 * Returned Value:
 *   Returns OK (zero) on success.
 *   Returns a negated errno value on failure.
 *
 * Assumptions:
 *   - The caller provides a valid 'upper' and 'hrtimer'
 *   - The caller holds no locks; locking is handled internally
 *
 ****************************************************************************/

int hrtimer_upper_add_rel(FAR struct hrtimer_upperhalf_s *upper,
                          FAR struct hrtimer_s *hrtimer,
                          clock_t increment)
{
  irqstate_t flags;
  clock_t now;
  clock_t expiration_value;
  int ret = OK;
  FAR struct hrtimer_node_s *inserted;

  DEBUGASSERT(upper != NULL && upper->lower != NULL);
  DEBUGASSERT(increment <= HRTIMER_MAX_INCREMENT);

  /* Bind this timer to the queue */

  hrtimer->upper = upper;

  /* Get the current time from the lower-half driver */

  now = HRTIMER_CURRENT(upper->lower);

  /* Calculate the absolute expiration time */

  expiration_value = now + increment;
  hrtimer->node.expiration_time = expiration_value;

  /* Insert the timer into the RB-tree */

  flags = spin_lock_irqsave(&upper->lock);

  inserted = RB_INSERT(hrtimer_tree, &upper->tree, &hrtimer->node);
  if (inserted != NULL)
    {
      /* Duplicate entry detected */

      ret = -EINVAL;
    }
  else
    {
      /* If this timer is the earliest, update hardware expiration */

      if (&hrtimer->node == RB_MIN(hrtimer_tree, &upper->tree))
        {
          ret = hrtimer_setexpire(upper, expiration_value);
        }
    }

  spin_unlock_irqrestore(&upper->lock, flags);

  return ret;
}
