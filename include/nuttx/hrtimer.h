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
#include <nuttx/compiler.h>

#include <assert.h>
#include <stdint.h>

#include <nuttx/list.h>
#include <sys/tree.h>

#include <nuttx/seqlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The pending state indicates the timer belongs to the shared hrtimer queue
 * and is waiting for the next hrtimer expiry.
 */

#define HRTIMER_ISPENDING(timer)       ((timer)->func != NULL)

/* The maximum delay tick should be INT64_MAX. However, if there are expired
 * hrtimer in the queue, HRTIMER_TIME_BEFORE/AFTER might be incorrect, so we
 * limited the delay to INT64_MAX >> 1, assuming all expired hrtimer can be
 * processed within HRTIMER_MAX_DELAY.
 */

#define HRTIMER_MAX_DELAY              (INT64_MAX >> 1)

#define HRTIMER_TIME_BEFORE(t1, t2)    ((int64_t)((t2) - (t1)) > 0)
#define HRTIMER_TIME_BEFORE_EQ(t1, t2) ((int64_t)((t2) - (t1)) >= 0)
#define HRTIMER_TIME_AFTER(t1, t2)     ((int64_t)((t2) - (t1)) < 0)
#define HRTIMER_TIME_AFTER_EQ(t1, t2)  ((int64_t)((t2) - (t1)) <= 0)

#define HRTIMER_MODE_ABS               (0)
#define HRTIMER_MODE_REL               (1)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct hrtimer_s;

/* This is the form of the callback function that is called when the
 * hrtimer function expires. The return value is next delay time.
 * If the return value is not equal 0 indicates it is periodic timer.
 * Notice that the timer is only use for offset calculation, the
 * hrtimer itself can not be written by the callback function.
 */

typedef CODE uint64_t (*hrtimer_callback_t)(
        FAR const struct hrtimer_s *timer, uint64_t expired);

RB_HEAD(hrtimer_tree_s, hrtimer_s);

typedef struct hrtimer_s
{
#ifdef CONFIG_HRTIMER_LIST
  struct list_node    node;    /* Supports a doubly linked list, 16-bytes for 64-bit architectures */
#else
  RB_ENTRY(hrtimer_s) node;    /* Supports a RB-tree node, 32-bytes for 64-bit architectures */
#endif
  uint64_t            expired; /* Expired time */
  hrtimer_callback_t  func;    /* Callback function */
} hrtimer_t;

typedef struct hrtimer_queue_s
{
  seqcount_t            lock;                      /* The RW-lock */
#ifdef CONFIG_HRTIMER_LIST
  struct list_node      queue;                     /* HRTimer doubly linked-list queue */
#else
  FAR struct hrtimer_s *first;                     /* Cached left-most timer in the queue */
  struct hrtimer_tree_s root;                      /* HRTimer red-black-tree-based queue */
#endif
  uint64_t              next_expired;              /* Next expired time */
  hrtimer_t             guard_timer;               /* Guard timer for functional-safety. */
  uintptr_t             running[CONFIG_SMP_NCPUS]; /* Hazard pointers for memory reclamation. */
} hrtimer_queue_t;

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#define hrtimer_init(timer) memset(timer, 0, sizeof(timer))

/* Wrapped version for NuttX scheduler. */

/****************************************************************************
 * Name: hrtimer_async_restart/start
 *
 * Description:
 *   Restart the hrtimer with relative or absolute time in nanoseconds.
 *   These functions allow restarting the hrtimer that has been set to
 *   cancelled state via `hrtimer_cancel`.
 *   Please be aware of concurrency issues. Concurrency errors are prone to
 *   occur in this use case.
 *
 * Input Parameters:
 *   timer - The hrtimer to be started.
 *
 * Returned Value:
 *   Zero on success.
 *   -EINVAL on if one of the parameter is invalid.
 *
 * Assumption:
 *   The timer and func should be not NULL.
 *   The timer should be cancelled or completed.
 *
 ****************************************************************************/

int hrtimer_async_restart(FAR hrtimer_t *timer);
int hrtimer_async_start(FAR hrtimer_t *timer);

/****************************************************************************
 * Name: hrtimer_restart/start
 *
 * Description:
 *   Start the hrtimer with absolute or relative time in nanoseconds.
 *   These functions can only be called when the caller has the ownership of
 *   the timer.
 *
 * Input Parameters:
 *   timer - The hrtimer to be started.
 *   func  - The callback function to be called when the hrtimer expires.
 *   time  - The absolute or relative expiration time in nanoseconds.
 *   mode  - The mode of the hrtimer, either HRTIMER_MODE_ABS or
 *           !HRTIMER_MODE_ABS.
 *
 * Returned Value:
 *   Zero on success.
 *   -EINVAL on if one of the parameter is invalid or the timer is in pending
 *   state.
 *
 ****************************************************************************/

static inline_function
void hrtimer_set_timer(FAR hrtimer_t *timer, hrtimer_callback_t func,
                           uint64_t time, uint32_t mode)
{
  uint64_t expired = mode == HRTIMER_MODE_REL ? clock_systime_nsec() +
                     (time <= HRTIMER_MAX_DELAY ? time : HRTIMER_MAX_DELAY) :
                     time;

  DEBUGASSERT(timer && func && !HRTIMER_ISPENDING(timer));

  timer->func    = func;
  timer->expired = expired;
}

static inline_function
int hrtimer_restart(FAR hrtimer_t *timer, hrtimer_callback_t func,
                    uint64_t time, uint32_t mode)
{
  int ret = -EINVAL;

  DEBUGASSERT(timer && func);

  hrtimer_set_timer(timer, func, time, mode);
  ret = hrtimer_async_restart(timer);

  return ret;
}

static inline_function
int hrtimer_start(FAR hrtimer_t *timer, hrtimer_callback_t func,
                  uint64_t time, uint32_t mode)
{
  int ret = -EINVAL;

  if (timer && func)
    {
      hrtimer_set_timer(timer, func, time, mode);
      ret = hrtimer_async_start(timer);
    }

  return ret;
}

/****************************************************************************
 * Name: hrtimer_cancel
 *
 * Description:
 *   Cancel the hrtimer asynchronously. This function set the timer to the
 *   cancelled state. The caller will acquire the limited ownership of the
 *   hrtimer, which allow the caller restart the hrtimer, but the callback
 *   function may still be executing on another CPU, which prevent the caller
 *   from freeing the hrtimer. The caller must call `hrtimer_cancel` to wait
 *   for the callback to be finished. Please use the function with care.
 *   Concurrency errors are prone to occur in this use case.
 *
 * Input Parameters:
 *   timer - The hrtimer to be cancelled.
 *
 * Returned Value
 *   OK on success. > 0 on if the timer callback is running.
 *
 * Assumption:
 *   The timer should not be NULL.
 *
 ****************************************************************************/

int hrtimer_cancel(FAR hrtimer_t *timer);

/****************************************************************************
 * Name: hrtimer_cancel_sync
 *
 * Description:
 *   Cancel the hrtimer and synchronously wait the callback to be finished.
 *   This function set the timer to the cancelled state and wait for all
 *   references to be released. The caller will then acquire full ownership
 *   of the hrtimer. After the function returns, the caller can safely
 *   deallocate the hrtimer.
 *
 * Input Parameters:
 *   timer - The hrtimer to be cancelled.
 *
 * Returned Value
 *   OK on success.
 *
 * Assumption:
 *   The timer should not be NULL.
 *
 ****************************************************************************/

int hrtimer_cancel_sync(FAR hrtimer_t *timer);

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
