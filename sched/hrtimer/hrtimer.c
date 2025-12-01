/****************************************************************************
 * sched/hrtimer/hrtimer.c
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

#include <assert.h>

#include "sched/sched.h"
#include "hrtimer.h"

#ifdef CONFIG_HRTIMER_LIST
#  define HRTIMER_QUEUE_USE_LIST
#endif

#include <nuttx/hrtimer_queue.h>

#include <debug.h>

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/* The reprogramming function can be fully inlined. */

static inline_function void hrtimer_reprogram(FAR hrtimer_queue_t *queue,
                                              uint64_t next_expired)
{
#ifdef CONFIG_SCHED_TICKLESS
#  ifdef CONFIG_SCHED_TICKLESS_ALARM
#    ifdef CONFIG_SCHED_TICKLESS_TICK_ARGUMENT
  up_alarm_tick_start(div_const_roundup(next_expired, NSEC_PER_TICK));
#    else
  struct timespec ts;
  clock_nsec2time(&ts, next_expired);
  up_alarm_start(&ts);
#    endif
#  else
#    ifdef CONFIG_SCHED_TICKLESS_TICK_ARGUMENT
  uint64_t nsec = clock_systime_nsec();
  next_expired  = HRTIMER_TIME_BEFORE(nsec, next_expired) ?
                  next_expired - nsec : 0u;
  up_timer_tick_start(div_const_roundup(next_expired, NSEC_PER_TICK));
#    else
  struct timespec ts;
  struct timespec current;
  up_timer_gettime(&current);
  clock_nsec2time(&ts, next_expired);
  clock_timespec_subtract(&ts, &current, &ts);
  up_timer_start(&ts);
#    endif
#  endif
#endif
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static hrtimer_queue_t g_hrtimer_queue;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hrtimer_initialize
 *
 * Description:
 *   Initialize the high-resolution timer queue for timing subsystem.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void hrtimer_initialize(void)
{
#if !defined(CONFIG_SCHED_TICKLESS) || \
    defined(CONFIG_SCHED_TICKLESS_TICK_ARGUMENT)
  swarn("WARNING: The system hrtimer is running in \
         low-resolution (tick) mode.");
#endif
  hrtimer_queue_init(&g_hrtimer_queue);
}

/****************************************************************************
 * Name: hrtimer_expiry
 *
 * Description:
 *   This function is called by the timer interrupt handler to handle
 *   if a hrtimer has expired.
 *
 * Input Parameters:
 *   nsec - The expiration time in nanoseconds.
 *   noswitches - True: Disable context switches.
 *
 * Returned Value:
 *   The next expiration time in nanoseconds.
 *
 ****************************************************************************/

uint64_t hrtimer_expiry(uint64_t nsec, bool noswitches)
{
  FAR hrtimer_queue_t *queue = &g_hrtimer_queue;
  return noswitches ? hrtimer_queue_read(queue, &queue->next_expired) :
                      hrtimer_queue_expiry(queue, nsec);
}

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
 * Name: hrtimer_start
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

HRTIMER_QUEUE_GENERATE(hrtimer, &g_hrtimer_queue, clock_systime_nsec)
