/****************************************************************************
 * include/nuttx/wdog.h
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

#ifndef __INCLUDE_NUTTX_WDOG_H
#define __INCLUDE_NUTTX_WDOG_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/compiler.h>
#include <nuttx/clock.h>
#include <errno.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WDOG_ISACTIVE(w)   ((w)->func != NULL)

/* The maximum delay tick are supposed to be CLOCK_MAX >> 1.
 * However, if there are expired wdog timers in the wdog queue,
 * clock_compare might be incorrect when the delay is CLOCK_MAX >> 1.
 * e.g. Current tick is 123, and there is an expired wdog timer with the
 * expired ticks 100. If we insert a wdog timer with delay CLOCK_MAX >> 1,
 * Then clock_compare(100, 123 + CLOCK_MAX >> 1) will return false, leading
 * to the new wdog timer queued before the expired wdog timer.
 * So we limited the delay to CLOCK_MAX >> 2, which is 2^30 - 1 or 2^62 - 1.
 * Assuming all expired wdog timer can be processed within WDOG_MAX_DELAY
 * ticks, this ensure the correct enqueue of the wdog timer.
 */

#define WDOG_MAX_DELAY     (CLOCK_MAX >> 2)

/****************************************************************************
 * Public Type Declarations
 ****************************************************************************/

/* The arguments are passed as scalar wdparm_t values.  For systems where
 * the sizeof(pointer) < sizeof(uint32_t), the following union defines the
 * alignment of the pointer within the uint32_t.  For example, the SDCC
 * MCS51 general pointer is 24-bits, but uint32_t is 32-bits (of course).
 *
 * We always have sizeof(pointer) <= sizeof(uintptr_t) by definition.
 */

#if UINTPTR_MAX >= UINT32_MAX
typedef uintptr_t wdparm_t;
#else
typedef uint32_t  wdparm_t;
#endif

/* This is the form of the function that is called when the
 * watchdog function expires.
 */

typedef CODE void (*wdentry_t)(wdparm_t arg);

/* Avoid the inclusion of nuttx/list.h */

struct wdlist_node
{
  FAR struct wdlist_node *prev;
  FAR struct wdlist_node *next;
};

/* Support a doubly linked list of watchdog timers */

struct wdog_s
{
  struct wdlist_node node;       /* Supports a doubly linked list */
  wdparm_t           arg;        /* Callback argument */
  wdentry_t          func;       /* Function to execute when delay expires */
#ifdef CONFIG_PIC
  FAR void          *picbase;    /* PIC base address */
#endif
  clock_t            expired;    /* Timer associated with the absolute time */
};

/****************************************************************************
 * Pubic Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: wd_start
 *
 * Description:
 *   This function adds a watchdog timer to the active timer queue.  The
 *   specified watchdog function at 'wdentry' will be called from the
 *   interrupt level after the specified number of ticks has elapsed.
 *   Watchdog timers may be started from the interrupt level.
 *
 *   Watchdog timers execute in the address environment that was in effect
 *   when wd_start() is called.
 *
 *   Watchdog timers execute only once.
 *
 *   To replace either the timeout delay or the function to be executed,
 *   call wd_start again with the same wdog; only the most recent wdStart()
 *   on a given watchdog ID has any effect.
 *
 * Input Parameters:
 *   wdog     - Watchdog ID
 *   delay    - Delay count in clock ticks
 *   wdentry  - Function to call on timeout
 *   arg      - Parameter to pass to wdentry.
 *
 *   NOTE:  The parameter must be of type wdparm_t.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return to
 *   indicate the nature of any failure.
 *
 * Assumptions:
 *   The watchdog routine runs in the context of the timer interrupt handler
 *   and is subject to all ISR restrictions.
 *
 ****************************************************************************/

int wd_start(FAR struct wdog_s *wdog, clock_t delay,
             wdentry_t wdentry, wdparm_t arg);

/****************************************************************************
 * Name: wd_start_abstick
 *
 * Description:
 *   This function adds a watchdog timer to the active timer queue.  The
 *   specified watchdog function at 'wdentry' will be called from the
 *   interrupt level after the specified number of ticks has reached.
 *   Watchdog timers may be started from the interrupt level.
 *
 *   Watchdog timers execute in the address environment that was in effect
 *   when wd_start() is called.
 *
 *   Watchdog timers execute only once.
 *
 *   To replace either the timeout delay or the function to be executed,
 *   call wd_start again with the same wdog; only the most recent wdStart()
 *   on a given watchdog ID has any effect.
 *
 * Input Parameters:
 *   wdog     - Watchdog ID
 *   ticks    - Absolute time in clock ticks
 *   wdentry  - Function to call on timeout
 *   arg      - Parameter to pass to wdentry.
 *
 *   NOTE:  The parameter must be of type wdparm_t.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return to
 *   indicate the nature of any failure.
 *
 * Assumptions:
 *   The watchdog routine runs in the context of the timer interrupt handler
 *   and is subject to all ISR restrictions.
 *
 ****************************************************************************/

int wd_start_abstick(FAR struct wdog_s *wdog, clock_t ticks,
                     wdentry_t wdentry, wdparm_t arg);

/****************************************************************************
 * Name: wd_start_abstime
 *
 * Description:
 *   This function adds a watchdog timer to the active timer queue.  The
 *   specified watchdog function at 'wdentry' will be called from the
 *   interrupt level after the specified number of ticks has reached.
 *   Watchdog timers may be started from the interrupt level.
 *
 *   Watchdog timers execute in the address environment that was in effect
 *   when wd_start() is called.
 *
 *   Watchdog timers execute only once.
 *
 *   To replace either the timeout delay or the function to be executed,
 *   call wd_start again with the same wdog; only the most recent wdStart()
 *   on a given watchdog ID has any effect.
 *
 * Input Parameters:
 *   wdog     - Watchdog ID
 *   abstime  - Absolute time with struct timespec
 *   wdentry  - Function to call on timeout
 *   arg      - Parameter to pass to wdentry.
 *
 *   NOTE:  The parameter must be of type wdparm_t.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return to
 *   indicate the nature of any failure.
 *
 * Assumptions:
 *   The watchdog routine runs in the context of the timer interrupt handler
 *   and is subject to all ISR restrictions.
 *
 ****************************************************************************/

#define wd_start_abstime(wdog, abstime, wdentry, arg) \
        wd_start_abstick(wdog, clock_time2ticks(abstime), wdentry, arg)

/****************************************************************************
 * Name: wd_start_realtime
 *
 * Description:
 *   This function adds a watchdog timer to the active timer queue.  The
 *   specified watchdog function at 'wdentry' will be called from the
 *   interrupt level after the specified number of ticks has reached.
 *   Watchdog timers may be started from the interrupt level.
 *
 *   Watchdog timers execute in the address environment that was in effect
 *   when wd_start() is called.
 *
 *   Watchdog timers execute only once.
 *
 *   To replace either the timeout delay or the function to be executed,
 *   call wd_start again with the same wdog; only the most recent wdStart()
 *   on a given watchdog ID has any effect.
 *
 * Input Parameters:
 *   wdog     - Watchdog ID
 *   realtime - Realtime with struct timespec
 *   wdentry  - Function to call on timeout
 *   arg      - Parameter to pass to wdentry.
 *
 *   NOTE:  The parameter must be of type wdparm_t.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return to
 *   indicate the nature of any failure.
 *
 * Assumptions:
 *   The watchdog routine runs in the context of the timer interrupt handler
 *   and is subject to all ISR restrictions.
 *
 ****************************************************************************/

static inline_function
int wd_start_realtime(FAR struct wdog_s *wdog,
                      FAR const struct timespec *realtime,
                      wdentry_t wdentry,
                      wdparm_t arg)
{
#ifdef CONFIG_CLOCK_TIMEKEEPING
  irqstate_t flags;
  clock_t ticks;
  int ret;

  flags = enter_critical_section();
  clock_abstime2ticks(CLOCK_REALTIME, realtime, &ticks);
  ret = wd_start(wdog, ticks, wdentry, arg);
  leave_critical_section(flags);

  return ret;
#else
  clock_t absticks;

  clock_realtime2absticks(realtime, &absticks);
  return wd_start_abstick(wdog, absticks, wdentry, arg);
#endif
}

/****************************************************************************
 * Name: wd_start_next
 *
 * Description:
 *   This function restart watchdog timer based on the last expiration time.
 *   It can be used to implement a periodic watchdog timer. E.g, Call this
 *   function instead of wd_start in the watchdog callback to restart the
 *   next timer for better timing accuracy.
 *   Note that calling this function outside the watchdog callback requires
 *   the wdog->expired being set.
 *
 * Input Parameters:
 *   wdog     - Pointer of the periodic watchdog.
 *   delay    - Delayed time in system ticks.
 *   wdentry  - Function to call on timeout.
 *   arg      - Parameter to pass to wdentry.
 *
 *   NOTE:  The parameter must be of type wdparm_t.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is return to
 *   indicate the nature of any failure.
 *
 * Assumptions:
 *   The watchdog routine runs in the context of the timer interrupt handler
 *   and is subject to all ISR restrictions.
 *
 ****************************************************************************/

static inline_function
int wd_start_next(FAR struct wdog_s *wdog, clock_t delay,
                  wdentry_t wdentry, wdparm_t arg)
{
  /* Ensure delay is within the range the wdog can handle. */

  if (delay > WDOG_MAX_DELAY)
    {
      return -EINVAL;
    }

  return wd_start_abstick(wdog, wdog->expired + delay, wdentry, arg);
}

/****************************************************************************
 * Name: wd_cancel
 *
 * Description:
 *   This function cancels a currently running watchdog timer. Watchdog
 *   timers may be cancelled from the interrupt level.
 *
 * Input Parameters:
 *   wdog - ID of the watchdog to cancel.
 *
 * Returned Value:
 *   Zero (OK) is returned on success;  A negated errno value is returned to
 *   indicate the nature of any failure.
 *
 ****************************************************************************/

int wd_cancel(FAR struct wdog_s *wdog);

/****************************************************************************
 * Name: wd_gettime
 *
 * Description:
 *   This function returns the time remaining before the specified watchdog
 *   timer expires.
 *
 * Input Parameters:
 *   wdog - watchdog ID
 *
 * Returned Value:
 *   The time in system ticks remaining until the watchdog time expires.
 *   Zero means either that wdog is not valid or that the wdog has already
 *   expired.
 *
 ****************************************************************************/

sclock_t wd_gettime(FAR struct wdog_s *wdog);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WDOG_H */
