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

#include <nuttx/hrtimer_queue_type.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef CONFIG_HRTIMER_LIST
typedef struct hrtimer_list_s hrtimer_t;
typedef struct hrtimer_list_queue_s hrtimer_queue_t;
#else
typedef struct hrtimer_rb_s hrtimer_t;
typedef struct hrtimer_rb_queue_s hrtimer_queue_t;
#endif

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

/* Wrapped version for NuttX scheduler. */

/****************************************************************************
 * Name: hrtimer_async_restart
 *
 * Description:
 *   Restart the hrtimer with relative or absolute time in nanoseconds.
 *   These functions allow restarting the hrtimer that has been set to
 *   cancelled state via `hrtimer_async_cancel`.
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

/****************************************************************************
 * Name: hrtimer_restart/start_absolute
 *
 * Description:
 *   Start the hrtimer with absolute time in nanoseconds.
 *   These functions can only be called when the caller has the ownership of
 *   the timer.
 *
 * Input Parameters:
 *   timer - The hrtimer to be started.
 *   func  - The callback function to be called when the hrtimer expires.
 *   arg   - The argument to be passed to the callback function.
 *   expired - The absolute expiration time in nanoseconds.
 *
 * Returned Value:
 *   Zero on success.
 *   -EINVAL on if one of the parameter is invalid or the timer is in pending
 *   state.
 *
 ****************************************************************************/

static inline_function
int hrtimer_restart_absolute(hrtimer_t *timer, hrtimer_callback_t func,
                             FAR void *arg, uint64_t expired)
{
  DEBUGASSERT(timer && func && !HRTIMER_ISPENDING(timer));
  hrtimer_fill(timer, func, arg, expired);
  return hrtimer_async_restart(timer);
}

static inline_function
int hrtimer_start_absolute(FAR hrtimer_t *timer, hrtimer_callback_t func,
                           FAR void *arg, uint64_t expired)
{
  int ret = -EINVAL;

  /* The run-time checking can be optimized if the func and timer can
   * be evaluated.
   */

  if (timer && func)
    {
      ret = hrtimer_restart_absolute(timer, func, arg, expired);
    }

  return ret;
}

/****************************************************************************
 * Name: hrtimer_restart/start
 *
 * Description:
 *   Start the hrtimer with relative time in nanoseconds.
 *   These functions can only be called when the caller has the ownership of
 *   the timer.
 *
 * Input Parameters:
 *   timer - The hrtimer to be started.
 *   func  - The callback function to be called when the hrtimer expires.
 *   arg   - The argument to be passed to the callback function.
 *   delay - The relative expiration time in nanoseconds.
 *
 * Returned Value:
 *   Zero on success.
 *   -EINVAL on if one of the parameter is invalid or the timer is in pending
 *   state.
 *
 ****************************************************************************/

static inline_function uint64_t hrtimer_delay2absolute(uint64_t delay)
{
  uint64_t expired = delay <= HRTIMER_MAX_DELAY ? delay : HRTIMER_MAX_DELAY;
  return expired + clock_systime_nsec();
}

#define hrtimer_restart(timer, func, arg, delay) \
  hrtimer_restart_absolute(timer, func, arg, hrtimer_delay2absolute(delay))

#define hrtimer_start(timer, func, arg, delay) \
  hrtimer_start_absolute(timer, func, arg, hrtimer_delay2absolute(delay))

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
