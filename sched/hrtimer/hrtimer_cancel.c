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
 * Pre-processor Definitions
 ****************************************************************************/

/* Delay used while waiting for a running hrtimer callback to complete */

#define HRTIMER_CANCEL_SYNC_DELAY_MS  5

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_cancel
 *
 * Description:
 *   Cancel a high-resolution timer.
 *
 *   If the timer is currently armed, it will be removed from the active
 *   hrtimer red-black tree and will not be executed.
 *
 *   If the timer callback is currently executing, the timer will be marked
 *   as canceled.  The running callback is allowed to complete, but the timer
 *   will not be re-armed or executed again.
 *
 *   If the canceled timer was the earliest (head) timer in the tree, the
 *   expiration of the underlying hardware timer will be updated to:
 *
 *     1. The expiration time of the next earliest timer, or
 *     2. A safe default expiration if no timers remain.
 *
 *   This function is non-blocking and does not wait for a running callback
 *   to finish.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the high-resolution timer instance to cancel.
 *
 * Returned Value:
 *   OK (0) on success; a negated errno value on failure.
 *
 * Assumptions/Notes:
 *   - This function acquires the global hrtimer spinlock to protect both
 *     the red-black tree and the timer state.
 *   - The caller must ensure that the timer structure is not freed until
 *     it is guaranteed that any running callback has returned.
 *
 ****************************************************************************/

int hrtimer_cancel(FAR hrtimer_t *hrtimer)
{
  FAR hrtimer_t *first;
  irqstate_t     flags;
  int            ret = OK;

  DEBUGASSERT(hrtimer != NULL);

  /* Enter critical section to protect the hrtimer tree and state */

  flags = spin_lock_irqsave(&g_hrtimer_spinlock);

  /* Capture the current earliest timer before any modification */

  first = (FAR hrtimer_t *)RB_MIN(hrtimer_tree_s, &g_hrtimer_tree);

  switch (hrtimer->state)
    {
      case HRTIMER_STATE_ARMED:
        {
          /* Remove the timer from the active tree */

          RB_REMOVE(hrtimer_tree_s, &g_hrtimer_tree, &hrtimer->node);
          hrtimer->state = HRTIMER_STATE_INACTIVE;
          break;
        }

      case HRTIMER_STATE_RUNNING:
        {
          /* The callback is currently executing.
           *
           * Mark the timer as canceled so it will not be re-armed or
           * executed again.  The running callback is allowed to complete.
           *
           * NOTE: The timer node is expected to have already been removed
           *       from the tree when the callback started executing.
           */

          hrtimer->state = HRTIMER_STATE_CANCELED;
          break;
        }

      case HRTIMER_STATE_INACTIVE:
      case HRTIMER_STATE_CANCELED:
      default:
        {
          ret = -EINVAL;
          break;
        }
    }

  /* If the canceled timer was the earliest one, update the hardware timer */

  if ((ret == OK) && (first == hrtimer))
    {
      first = (FAR hrtimer_t *)RB_MIN(hrtimer_tree_s, &g_hrtimer_tree);
      if (first != NULL)
        {
          ret = hrtimer_starttimer(first->expired);
        }
    }

  /* Leave critical section */

  spin_unlock_irqrestore(&g_hrtimer_spinlock, flags);
  return ret;
}

/****************************************************************************
 * Name: hrtimer_cancel_sync
 *
 * Description:
 *   Cancel a high-resolution timer and wait until it becomes inactive.
 *
 *   - Calls hrtimer_cancel() to request timer cancellation.
 *   - If the timer callback is running, waits until it completes and
 *     the timer state transitions to HRTIMER_STATE_INACTIVE.
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
  int  ret;
  bool cansleep;

  DEBUGASSERT(hrtimer != NULL);

  /* Determine whether sleeping is permitted in the current context */

  cansleep = !up_interrupt_context() &&
             !is_idle_task(this_task());

  /* Request cancellation of the timer */

  ret = hrtimer_cancel(hrtimer);
  if (ret < 0)
    {
      return ret;
    }

  /* Wait until the timer transitions to the inactive state.
   *
   * If sleeping is permitted, yield the CPU briefly to avoid
   * busy-waiting.  Otherwise, spin until the callback completes
   * and the state becomes inactive.
   */

  while (hrtimer->state != HRTIMER_STATE_INACTIVE)
    {
      if (cansleep)
        {
          nxsched_msleep(HRTIMER_CANCEL_SYNC_DELAY_MS);
        }
    }

  return OK;
}
