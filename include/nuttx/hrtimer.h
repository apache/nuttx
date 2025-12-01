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
 * Name: hrtimer_restart
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
 *   func  - The callback function to be called when the timer expires.
 *   arg   - The argument to be passed to the callback function.
 *   time  - The relative or absolute expiration time in nanoseconds.
 *   mode  - The timer mode, relative or absolute.
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

/****************************************************************************
 * Name: hrtimer_start/start_absolute
 *
 * Description:
 *   Start the hrtimer with relative or absolute time in nanoseconds.
 *   These functions can only be called when the caller has the ownership of
 *   the timer.
 *
 * Input Parameters:
 *   timer - The hrtimer to be started.
 *   func  - The callback function to be called when the hrtimer expires.
 *   arg   - The argument to be passed to the callback function.
 *   time  - The relative or absolute expiration time in nanoseconds.
 *   mode  - The timer mode, relative or absolute.
 *
 * Returned Value:
 *   Zero on success.
 *   -EINVAL on if one of the parameter is invalid or the timer is in pending
 *   state.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_async_cancel
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
 *   OK on success. -EINVAL on if the timer has not been in pending state.
 *
 * Assumption:
 *   The timer should not be NULL.
 *
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_cancel
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
 *   OK on success. -EINVAL on if the timer has not been in pending state.
 *
 * Assumption:
 *   The timer should not be NULL.
 *
 ****************************************************************************/

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

/* Declare all function proto-types above. */

#ifdef CONFIG_HRTIMER_LIST
HRTIMER_QUEUE_PROTOTYPE_LIST(hrtimer, clock_systime_nsec)
#else
HRTIMER_QUEUE_PROTOTYPE_RB(hrtimer, clock_systime_nsec)
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_HRTIMER_H */
