/****************************************************************************
 * sched/hrtimer/hrtimer_cancel.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file under the Apache License, Version 2.0 (the
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

#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_cancel
 *
 * Description:
 *   Cancel a high-resolution timer asynchronously.
 *
 *   If the timer is currently pending, it will be removed from the
 *   hrtimer queue and will not be executed.
 *
 *   If the timer callback is currently executing. This function set the
 *   timer to the cancelled state. The caller will acquire the limited
 *   ownership of the hrtimer, which allow the caller restart the hrtimer,
 *   but the callback function may still be executing on another CPU,
 *   which prevent the caller from freeing the hrtimer.
 *   The caller must call `hrtimer_cancel_sync` to wait for the callback
 *   to be finished. Please use the function with care.
 *   Concurrency errors are prone to occur in this use case.
 *
 *   If the canceled timer was the earliest expired timer in the queue,
 *   the expiration of the underlying hardware timer will be updated to the
 *   expiration time of the next earliest timer
 *
 *   This function is non-blocking and does not wait for a running callback
 *   to finish.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the high-resolution timer instance to cancel.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno value on failure.
 *   > 0 on if the timer callback is running.
 *
 * Assumptions/Notes:
 *   - This function acquires the global hrtimer spinlock to protect
 *     the container.
 *   - The caller must ensure that the timer structure is not freed until
 *     it is guaranteed that any running callback has returned.
 *
 ****************************************************************************/

int hrtimer_cancel(FAR hrtimer_t *hrtimer)
{
  FAR hrtimer_t *first;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(hrtimer != NULL);

  /* Acquire the lock and seize the ownership of the hrtimer queue. */

  flags = write_seqlock_irqsave(&g_hrtimer_lock);

  /* Ensure no core can write the hrtimer. */

  ret = hrtimer_cancel_running(hrtimer);

  if (hrtimer_is_armed(hrtimer))
    {
      hrtimer_remove(hrtimer);

      /* Update the hardware timer if the queue head changed. */

      if (hrtimer_is_first(hrtimer))
        {
          first = hrtimer_get_first();
          if (first != NULL)
            {
              hrtimer_reprogram(first->expired);
            }
        }
    }

  /* Release the lock and give up the ownership of the hrtimer queue. */

  write_sequnlock_irqrestore(&g_hrtimer_lock, flags);
  return ret;
}

/****************************************************************************
 * Name: hrtimer_cancel_sync
 *
 * Description:
 *   Cancel a high-resolution timer and synchronously wait the callback to
 *   be finished.
 *
 *   If the timer callback is running, this function set the timer to the
 *   cancelled state and wait for all all references to be released.
 *   The caller will then acquire full ownership of the hrtimer. After the
 *   function returns, the caller can safely deallocate the hrtimer.
 *   - If sleeping is allowed (normal task context), yields CPU briefly
 *     to avoid busy-waiting.
 *   - Otherwise (interrupt or idle task context), spins until completion.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the high-resolution timer instance to cancel.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int hrtimer_cancel_sync(FAR hrtimer_t *hrtimer)
{
  int ret = OK;

  DEBUGASSERT(hrtimer != NULL);

  /* Request cancellation of the timer */

  ret = hrtimer_cancel(hrtimer);
  if (ret > 0)
    {
      /* Wait until all the timer callbacks finish. */

       hrtimer_wait(hrtimer);
    }

  return ret;
}
