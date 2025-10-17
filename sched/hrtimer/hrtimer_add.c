/****************************************************************************
 * sched/hrtimer/hrtimer_add.c
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
 *   Add a high-resolution timer to the active queue using an absolute
 *   expiration time.  If this timer becomes the earliest in the queue,
 *   update the underlying hardware expiration time.
 *
 * Input Parameters:
 *   upper   - High-resolution timer upper-half driver instance
 *   hrtimer - Timer instance to be queued
 *   start   - Absolute expiration time in timer ticks
 *
 * Returned Value:
 *   OK (zero) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   The caller provides valid arguments.  Called from task or interrupt
 *   context.  Internal locking is handled within this function.
 *
 ****************************************************************************/

int hrtimer_upper_add_abs(FAR struct hrtimer_upperhalf_s *upper,
                          FAR struct hrtimer_s *hrtimer,
                          uint64_t start)
{
  irqstate_t flags;
  int ret = OK;
  FAR struct hrtimer_node *inserted;

  DEBUGASSERT(upper != NULL);

  /* Bind this timer to the queue */

  hrtimer->upper              = upper;
  hrtimer->node.expiration_time = start;

  /* Insert the timer into the RB tree */

  flags = spin_lock_irqsave(&upper->lock);

  inserted = RB_INSERT(hrtimer_tree, &upper->tree, &hrtimer->node);
  if (inserted != NULL)
    {
      /* Duplicate entry */

      ret = -EINVAL;
    }
  else
    {
      /* Update hardware if this is the earliest timer */

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
 *   Add a high-resolution timer to the active queue using a relative
 *   expiration time.  The absolute expiration time is calculated as the
 *   current time plus the increment.  If this timer becomes the earliest
 *   in the queue, update the underlying hardware expiration time.
 *
 * Input Parameters:
 *   upper     - High-resolution timer upper-half driver instance
 *   hrtimer   - Timer instance to be queued
 *   increment - Relative time interval in timer ticks
 *
 * Returned Value:
 *   OK (zero) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   The caller provides valid arguments.  Called from task or interrupt
 *   context.  Internal locking is handled within this function.
 *
 ****************************************************************************/

int hrtimer_upper_add_rel(FAR struct hrtimer_upperhalf_s *upper,
                          FAR struct hrtimer_s *hrtimer,
                          uint64_t increment)
{
  irqstate_t flags;
  uint64_t now;
  uint64_t expiration;
  int ret = OK;
  FAR struct hrtimer_node *inserted;

  DEBUGASSERT(upper != NULL);
  DEBUGASSERT(increment <= HRTIMER_MAX_INCREMENT);

  /* Bind this timer to the queue */

  hrtimer->upper = upper;

  /* Get current time from the lower-half driver */

  hrtimer_clock_current(upper, &now);

  /* Calculate absolute expiration */

  expiration = now + increment;
  hrtimer->node.expiration_time = expiration;

  /* Insert the timer into the RB tree */

  flags = spin_lock_irqsave(&upper->lock);

  inserted = RB_INSERT(hrtimer_tree, &upper->tree, &hrtimer->node);
  if (inserted != NULL)
    {
      /* Duplicate entry */

      ret = -EINVAL;
    }
  else
    {
      /* Update hardware if this is the earliest timer */

      if (&hrtimer->node == RB_MIN(hrtimer_tree, &upper->tree))
        {
          ret = hrtimer_setexpire(upper, expiration);
        }
    }

  spin_unlock_irqrestore(&upper->lock, flags);
  return ret;
}
