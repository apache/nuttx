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
 * Name: hrtimer_upper_process
 *
 * Description:
 *   Process expired high-resolution timers in the given queue.  This
 *   function removes each expired timer, invokes its callback, and
 *   continues until either:
 *
 *     1. No more expired timers remain, or
 *     2. The maximum processing limit is reached.
 *
 *   After processing, the next expiration is scheduled as follows:
 *     - The earliest active timer in the queue, or
 *     - A fallback expiration (current time + default increment) if
 *       the queue is empty.
 *
 * Input Parameters:
 *   upper - Pointer to the high-resolution timer queue instance
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void hrtimer_upper_process(FAR struct hrtimer_upperhalf_s *upper)
{
  FAR struct hrtimer_node *node;
  FAR struct hrtimer_s *hrtimer;
  clock_t now;
  clock_t expiration_value;
  uint8_t process_cnt = 0;
  irqstate_t flags;

  DEBUGASSERT(upper != NULL && upper->lower != NULL);

  /* Enter critical section while processing timers */

  flags = spin_lock_irqsave(&upper->lock);

  /* Get the earliest timer in the queue */

  node = RB_MIN(hrtimer_tree, &upper->tree);
  while (node != NULL)
    {
      /* Get current time */

      now = HRTIMER_CURRENT(upper->lower);

      /* Check if this timer has expired */

      if (hrtimer_has_expired(node->expiration_time, now))
        {
          /* Remove timer and invoke its callback */

          RB_REMOVE(hrtimer_tree, &upper->tree, node);
          hrtimer = (FAR struct hrtimer_s *)node;

          DEBUGASSERT(hrtimer->callback != NULL);
          hrtimer->callback(hrtimer);

          process_cnt++;

          /* To avoid holding interrupts disabled for too long,
           * periodically restore and re-enter the critical section.
           */

          if (process_cnt >= MAX_HRTIMER_PROCESS_CNT)
            {
              process_cnt = 0;
              spin_unlock_irqrestore(&upper->lock, flags);
              flags = spin_lock_irqsave(&upper->lock);
            }
        }
      else
        {
          /* Stop processing when the next timer has not yet expired */

          break;
        }

      /* Continue with the next earliest timer */

      node = RB_MIN(hrtimer_tree, &upper->tree);
    }

  /* Update the next expiration time */

  if (node != NULL)
    {
      (void)hrtimer_setexpire(upper, node->expiration_time);
    }
  else
    {
      now             = HRTIMER_CURRENT(upper->lower);
      expiration_value = now + HRTIMER_DEFAULT_INCREMENT;

      (void)hrtimer_setexpire(upper, expiration_value);
    }

  /* Leave critical section */

  spin_unlock_irqrestore(&upper->lock, flags);
}
