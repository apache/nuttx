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
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
#  define wd_in_callback() (g_wdtimernested)
#  define wd_set_nested(f) (g_wdtimernested = (f))
#else
#  define wd_in_callback() (false)
#  define wd_set_nested(f)
#endif

#ifdef CONFIG_SCHED_TICKLESS
static inline_function void wd_timer_start(clock_t next_tick)
{
#ifdef CONFIG_SCHED_TICKLESS_ALARM
#  ifndef CONFIG_ALARM_ARCH
  struct timespec ts;
  clock_ticks2time(&ts, next_tick);
  up_alarm_start(&ts);
#  else
  up_alarm_tick_start(next_tick);
#  endif
#else
#  ifndef CONFIG_TIMER_ARCH
  struct timespec ts1;
  struct timespec ts2;
  clock_ticks2time(&ts1, next_tick);
  clock_systime_timespec(&ts2);
  clock_timespec_subtract(&ts1, &ts2, &ts1);
  up_timer_start(&ts1);
#  else
  up_timer_tick_start(next_tick - clock_systime_ticks());
#  endif
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
