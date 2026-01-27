/****************************************************************************
 * sched/wdog/wdog.h
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

#ifndef __SCHED_WDOG_WDOG_H
#define __SCHED_WDOG_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <nuttx/compiler.h>
#include <nuttx/clock.h>
#include <nuttx/queue.h>
#include <nuttx/wdog.h>
#include <nuttx/arch.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* The g_wdactivelist data structure is a singly linked list ordered by
 * watchdog expiration time. When watchdog timers expire,the functions on
 * this linked list are removed and the function is called.
 */

extern struct list_node g_wdactivelist;

#ifdef CONFIG_SCHED_TICKLESS
extern bool g_wdtimernested;
extern clock_t  g_wdexpired;
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
#  define wd_in_callback()          (g_wdtimernested)
#  define wd_set_nested(f)          (g_wdtimernested = (f))
#  define wd_update_expire(expired) (g_wdexpired = (expired))
#else
#  define wd_in_callback() (false)
#  define wd_set_nested(f)
#  define wd_update_expire(expired)
#endif

#ifdef CONFIG_SCHED_TICKLESS
static inline_function clock_t wd_adjust_next_tick(clock_t tick)
{
  clock_t next_tick = tick;
#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
  clock_t interval  = clock_compare(g_wdexpired, tick) ?
                      tick - g_wdexpired : 0u;
  interval          = interval <= g_oneshot_maxticks ?
                      interval : g_oneshot_maxticks;
  next_tick         = g_wdexpired + interval;
#endif

#if CONFIG_TIMER_ADJUST_USEC > 0
  /* Normally, timer event cannot triggered on exact time due to the
   * existence of interrupt latency.
   * Assuming that the interrupt latency is distributed within
   * [Best-Case Execution Time, Worst-Case Execution Time],
   * we can set the timer adjustment value to the BCET to reduce the latency.
   * After the adjustment, the timer interrupt latency will be
   * [0, WCET - BCET].
   * Please use this carefully, if the timer adjustment value is not the
   * best-case interrupt latency, it will immediately fired another timer
   * interrupt, which may result in a much larger timer interrupt latency.
   */

  next_tick -= CONFIG_TIMER_ADJUST_USEC / USEC_PER_TICK;
#endif

  return next_tick;
}

static inline_function void wd_timer_start(clock_t tick)
{
  clock_t next_tick = wd_adjust_next_tick(tick);

#ifdef CONFIG_HRTIMER
  nxsched_hrtimer_tick_start(tick);
#elif defined(CONFIG_SCHED_TICKLESS_ALARM)
  up_alarm_tick_start(next_tick);
#else
  up_timer_tick_start(next_tick - clock_systime_ticks());
#endif
}

static inline_function void wd_timer_cancel(void)
{
  struct timespec ts;
#ifdef CONFIG_SCHED_TICKLESS_ALARM
  up_alarm_cancel(&ts);
#else
  up_timer_cancel(&ts);
#endif
}
#else
#  define wd_timer_start(next_tick)
#  define wd_timer_cancel()
#endif

static inline_function clock_t wd_next_expire(void)
{
  return list_first_entry(&g_wdactivelist, struct wdog_s, node)->expired;
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

static inline_function clock_t wd_get_next_expire(clock_t curr)
{
  clock_t     next = curr;
  irqstate_t flags = enter_critical_section();

  if (!list_is_empty(&g_wdactivelist))
    {
      next = wd_next_expire();
    }

  leave_critical_section(flags);
  return (sclock_t)(next - curr) <= 0 ? 0u : next;
}

/****************************************************************************
 * Name: wd_timer
 *
 * Description:
 *   This function is called from the timer interrupt handler to determine
 *   if it is time to execute a watchdog function.  If so, the watchdog
 *   function will be executed in the context of the timer interrupt
 *   handler.
 *
 * Input Parameters:
 *   ticks - If CONFIG_SCHED_TICKLESS is defined then the number of ticks
 *     in the interval that just expired is provided.  Otherwise,
 *     this function is called on each timer interrupt and a value of one
 *     is implicit.
 *   noswitches - True: Can't do context switches now.
 *
 * Returned Value:
 *   If CONFIG_SCHED_TICKLESS is defined then the number of ticks for the
 *   next delay is provided (zero if no delay).  Otherwise, this function
 *   has no returned value.
 *
 * Assumptions:
 *   Called from interrupt handler logic with interrupts disabled.
 *
 ****************************************************************************/

void wd_timer(clock_t ticks);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __SCHED_WDOG_WDOG_H */
