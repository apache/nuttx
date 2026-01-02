/****************************************************************************
 * sched/hrtimer/hrtimer_set.c
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
#include <nuttx/arch.h>

#include "sched/sched.h"
#include "hrtimer/hrtimer.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_set
 *
 * Description:
 *   Set or update the callback function and argument for a high-resolution
 *   timer.
 *
 *   This function only updates the callback context associated with the
 *   timer.  It does not arm, disarm, or otherwise modify the timer's
 *   expiration state.
 *
 *   If the timer callback is currently executing, the updated callback
 *   function will not affect the running invocation, but will be observed
 *   by any subsequent expiration.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the high-resolution timer instance.
 *   func    - Callback function to be invoked on timer expiration.
 *   arg     - Argument passed to the callback function.
 *
 * Returned Value:
 *   None.
 *
 * Assumptions/Notes:
 *   - The global hrtimer spinlock protects access to the timer state.
 *   - The caller must ensure that the timer structure remains valid while
 *     the callback may still be executing.
 *   - This function is safe to call regardless of whether the timer is
 *     currently armed or inactive.
 *
 ****************************************************************************/

int hrtimer_set(FAR hrtimer_t *hrtimer,
                hrtimer_cb func,
                FAR void *arg)
{
  irqstate_t flags;

  DEBUGASSERT(hrtimer != NULL);
  DEBUGASSERT(func != NULL);

  /* Protect timer state against concurrent access */

  flags = write_seqlock_irqsave(&g_hrtimer_spinlock);

  /* Update callback context */

  hrtimer->func = func;
  hrtimer->arg  = arg;

  write_sequnlock_irqrestore(&g_hrtimer_spinlock, flags);

  return OK;
}
