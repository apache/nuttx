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
#include <nuttx/hrtimer.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/* The g_wdactivelist data structure is a singly linked list ordered by
 * watchdog expiration time. When watchdog timers expire,the functions on
 * this linked list are removed and the function is called.
 */

extern struct list_node g_wdactivelist;

extern struct hrtimer_s g_wdtimer;

extern bool g_wdtimernested;
extern clock_t  g_wdexpired;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

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

uint64_t wd_timer(const hrtimer_t *timer, uint64_t expired);

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#define wd_in_callback()          (g_wdtimernested)
#define wd_set_nested(f)          (g_wdtimernested = (f))
#define wd_update_expire(expired) (g_wdexpired = (expired))

static inline_function void wd_timer_start(clock_t tick)
{
  uint64_t delay = clock_compare(g_wdexpired, tick) ?
                   tick - g_wdexpired : 0u;
  uint64_t nsec  = delay <= HRTIMER_MAX_DELAY / NSEC_PER_TICK ?
                   TICK2NSEC(g_wdexpired + delay) :
                   TICK2NSEC((uint64_t)g_wdexpired) + HRTIMER_MAX_DELAY;
  hrtimer_start(&g_wdtimer, wd_timer, nsec, HRTIMER_MODE_ABS);
}

static inline_function void wd_timer_cancel(void)
{
  hrtimer_cancel(&g_wdtimer);
}

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
  return next - curr <= 0 ? 0 : next;
}

#ifdef __cplusplus
}
#endif

#endif /* __SCHED_WDOG_WDOG_H */
