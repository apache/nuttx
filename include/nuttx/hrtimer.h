/****************************************************************************
 * include/nuttx/hrtimer.h
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

#ifndef __INCLUDE_NUTTX_HRTIMER_H
#define __INCLUDE_NUTTX_HRTIMER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/clock.h>
#include <nuttx/compiler.h>
#include <nuttx/spinlock.h>
#include <nuttx/list.h>

#include <stdint.h>
#include <sys/tree.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* High-resolution timer modes
 *
 * HRTIMER_MODE_ABS - Absolute expiration time
 * HRTIMER_MODE_REL - Relative timeout from the current time
 */

enum hrtimer_mode_e
{
  HRTIMER_MODE_ABS = 0,  /* Absolute expiration time */
  HRTIMER_MODE_REL       /* Relative delay from now */
};

/* Forward declarations */

struct hrtimer_s;

/* Callback type for high-resolution timer expiration
 *
 * The callback is invoked when the timer expires. It is executed in
 * timer context and must not block.
 */

typedef CODE uint64_t (*hrtimer_entry_t)(FAR const struct hrtimer_s *hrtimer,
                                         uint64_t expired);

/* Red-black tree node used to order hrtimers by expiration time */

typedef struct hrtimer_node_s
{
#ifdef CONFIG_HRTIMER_TREE
  RB_ENTRY(hrtimer_node_s) entry;  /* RB-tree linkage */
#else
  struct list_node entry;  /* List linkage */
#endif
} hrtimer_node_t;

/* High-resolution timer object
 *
 * The timer is ordered by absolute expiration time and managed by the
 * hrtimer core. The content of this structure should not be accessed
 * directly by users except through the provided APIs.
 */

typedef struct hrtimer_s
{
  hrtimer_node_t node;   /* RB-tree node for sorted insertion */
  hrtimer_entry_t func;  /* Expiration callback function */
  uint64_t expired;      /* Absolute expiration time (ns) */
} hrtimer_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: hrtimer_init
 *
 * Description:
 *   Initialize a high-resolution timer instance. This function sets the
 *   expiration callback and its argument. The timer is initialized in the
 *   inactive state and is not armed until hrtimer_start() is called.
 *
 * Input Parameters:
 *   hrtimer - Pointer to the hrtimer instance to be initialized
 *   func    - Expiration callback function
 *   arg     - Argument passed to the callback
 *
 * Returned Value:
 *   None
 ****************************************************************************/

#define hrtimer_init(hrtimer) memset(hrtimer, 0, sizeof(hrtimer_t))

/****************************************************************************
 * Name: hrtimer_cancel
 *
 * Description:
 *   Cancel a high-resolution timer.
 *
 *   If the timer is armed but has not yet expired, it will be removed from
 *   the timer queue and the callback will not be invoked.
 *
 *   If the timer callback is currently executing, this function will mark
 *   the timer as canceled and return immediately. The running callback is
 *   allowed to complete, but it will not be invoked again.
 *
 *   This function is non-blocking and does not wait for a running callback
 *   to finish.
 *
 * Input Parameters:
 *   hrtimer - Timer instance to cancel
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 ****************************************************************************/

int hrtimer_cancel(FAR hrtimer_t *hrtimer);

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

int hrtimer_cancel_sync(FAR hrtimer_t *hrtimer);

/****************************************************************************
 * Name: hrtimer_start
 *
 * Description:
 *   Start a high-resolution timer with the specified expiration time.
 *
 *   The expiration time may be specified as either an absolute time or
 *   a relative timeout, depending on the selected mode.
 *
 * Input Parameters:
 *   hrtimer - Timer instance to start
 *   func    - Expiration callback function
 *   expired - Expiration time in nanoseconds
 *   mode    - HRTIMER_MODE_ABS or HRTIMER_MODE_REL
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 ****************************************************************************/

int hrtimer_start(FAR hrtimer_t *hrtimer, hrtimer_entry_t func,
                  uint64_t expired,
                  enum hrtimer_mode_e mode);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_HRTIMER_H */
