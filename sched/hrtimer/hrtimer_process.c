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
 *   retrieves the earliest timer from the active timer queue, checks
 *   if it has expired relative to the current time, removes it from the
 *   queue, and invokes its callback function. Processing continues
 *   until:
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
 *   - This function acquires a spinlock to protect the timer queue.
 *   - Timer callbacks are invoked with interrupts enabled
 *     to avoid deadlocks.
 *   - DEBUGASSERT ensures that timer callbacks are valid.
 ****************************************************************************/

void hrtimer_process(uint64_t now)
{
  FAR hrtimer_t *hrtimer;
  irqstate_t flags;
  hrtimer_entry_t func;
  uint64_t expired;
  uint64_t delay;
  int cpu = this_cpu();

  /* Acquire the lock and seize the ownership of the hrtimer queue. */

  flags = write_seqlock_irqsave(&g_hrtimer_lock);

  for (; ; )
    {
      /* Fetch the earliest active timer */

      hrtimer = hrtimer_get_first();
      expired = hrtimer->expired;

      /* Check if the timer has expired */

      if (!HRTIMER_TIME_BEFORE_EQ(expired, now))
        {
          break;
        }

      /* Remove the expired timer from the timer queue */

      func = hrtimer->func;
      hrtimer_remove(hrtimer);

      hrtimer_mark_running(hrtimer, cpu);

      /* Leave critical section before invoking the callback */

      write_sequnlock_irqrestore(&g_hrtimer_lock, flags);

      /* Invoke the timer callback */

      delay = func(hrtimer, expired);

      /* Ensure the delay is valid. */

      DEBUGASSERT(HRTIMER_TIME_BEFORE_EQ(expired + delay,
                                         now + HRTIMER_MAX_DELAY));

      /* Re-enter critical section to update timer state */

      flags = write_seqlock_irqsave(&g_hrtimer_lock);

      /* If the timer is periodic and has not been rearmed or
       * cancelled concurrently, calculate next expiration and
       * re-insert into the timer queue.
       */

      if (delay != 0u && hrtimer_is_running(hrtimer, cpu))
        {
          hrtimer->expired = expired + delay;
          hrtimer->func    = func;
          hrtimer_insert(hrtimer);
        }
    }

  hrtimer_unmark_running(cpu);

  /* Schedule the next timer expiration */

  if (expired != now)
    {
      /* Start timer for the next earliest expiration */

      hrtimer_reprogram(expired);
    }

  /* Release the lock and give up the ownership of the hrtimer queue. */

  write_sequnlock_irqrestore(&g_hrtimer_lock, flags);
}
