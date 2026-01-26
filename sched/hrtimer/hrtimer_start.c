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
 *   func    - Expiration callback function.
 *   expired - Expiration time in nanoseconds. Interpretation
 *             depends on mode.
 *   mode    - Timer mode (HRTIMER_MODE_ABS or HRTIMER_MODE_REL).
 *
 * Returned Value:
 *   OK (0) on success, or a negated errno value on failure.
 *
 * Assumptions/Notes:
 *   - This function disables interrupts briefly via spinlock to safely
 *     insert the timer into the container.
 *   - Absolute mode sets the timer to expire at the given absolute time.
 *   - Relative mode sets the timer to expire after 'ns'
 *     nanoseconds from the current time.
 ****************************************************************************/

int hrtimer_start(FAR hrtimer_t *hrtimer, hrtimer_entry_t func,
                  uint64_t expired,
                  enum hrtimer_mode_e mode)
{
  uint64_t next_expired;
  irqstate_t flags;
  int ret = OK;

  /* Compute absolute expiration time */

  if (mode == HRTIMER_MODE_ABS)
    {
      next_expired = expired;
    }
  else
    {
      expired = expired <= HRTIMER_MAX_DELAY ? expired : HRTIMER_MAX_DELAY;
      next_expired = clock_systime_nsec() + expired;
    }

  DEBUGASSERT(hrtimer != NULL);

  /* Acquire the lock and seize the ownership of the hrtimer queue. */

  flags = write_seqlock_irqsave(&g_hrtimer_lock);

  /* Ensure no core can write the hrtimer. */

  hrtimer_cancel_running(hrtimer);

  if (hrtimer_is_armed(hrtimer))
    {
      hrtimer_remove(hrtimer);
    }

  hrtimer->func    = func;
  hrtimer->expired = next_expired;

  /* Insert the timer into the hrtimer queue. */

  hrtimer_insert(hrtimer);

  /* If the inserted timer is now the earliest, start hardware timer */

  if (hrtimer_is_first(hrtimer))
    {
      hrtimer_reprogram(hrtimer->expired);
    }

  /* Release the lock and give up the ownership of the hrtimer queue. */

  write_sequnlock_irqrestore(&g_hrtimer_lock, flags);

  return ret;
}
