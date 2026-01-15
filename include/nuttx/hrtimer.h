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
 * Pre-processor Definitions
 ****************************************************************************/

/* The maximum delay tick should be INT64_MAX. However, if there are expired
 * hrtimer in the queue, HRTIMER_TIME_BEFORE/AFTER might be incorrect, so we
 * limited the delay to INT64_MAX >> 1, assuming all expired hrtimer can be
 * processed within HRTIMER_MAX_DELAY.
 */

#define HRTIMER_MAX_DELAY              (INT64_MAX >> 1)

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

/* Hrtimer container node used to order hrtimers by expiration time */

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
  hrtimer_node_t node;   /* Container node for sorted insertion */
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
 *   Cancel a high-resolution timer asynchronously. This function set the
 *   timer to the cancelled state. The caller will acquire the limited
 *   ownership of the hrtimer, which allow the caller restart the hrtimer,
 *   but the callback function may still be executing on another CPU, which
 *   prevent the caller from freeing the hrtimer. The caller must call
 *   `hrtimer_cancel` to wait for the callback to be finished. Please use
 *   the function with care. Concurrency errors are prone to occur in this
 *   use case.
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
 *   - This function acquires the global hrtimer spinlock to protect both
 *     the red-black tree and the timer state.
 *   - The caller must ensure that the timer structure is not freed until
 *     it is guaranteed that any running callback has returned.
 *
 ****************************************************************************/

int hrtimer_cancel(FAR hrtimer_t *hrtimer);

/****************************************************************************
 * Name: hrtimer_cancel_sync
 *
 * Description:
 *   Cancel a high-resolution timer and wait until it becomes inactive.
 *
 *   Cancel a high-resolution timer and synchronously wait the callback to
 *   be finished. This function set the timer to the cancelled state and wait
 *   for all references to be released. The caller will then acquire full
 *   ownership of the hrtimer. After the function returns, the caller can
 *   safely deallocate the hrtimer.
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

/****************************************************************************
 * Name: hrtimer_gettime
 *
 * Description:
 *   Get the rest of the delay time of the hrtimer in nanoseconds.
 *
 * Input Parameters:
 *   timer - The hrtimer to be queried.
 *
 * Returned Value
 *   The time until next expiration in nanoseconds.
 *
 * Assumption:
 *   The timer should not be NULL.
 *
 ****************************************************************************/

uint64_t hrtimer_gettime(FAR hrtimer_t *timer);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_HRTIMER_H */
