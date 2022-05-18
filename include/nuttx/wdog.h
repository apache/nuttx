/****************************************************************************
 * include/nuttx/wdog.h
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

#include <nuttx/clock.h>
#include <stdint.h>
#include <queue.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define WDOG_ISACTIVE(w)   ((w)->func != NULL)

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

/* This is the internal representation of the watchdog timer structure. */

struct wdog_s
{
  FAR struct wdog_s *next;       /* Support for singly linked lists. */
  wdentry_t          func;       /* Function to execute when delay expires */
#ifdef CONFIG_PIC
  FAR void          *picbase;    /* PIC base address */
#endif
  sclock_t           lag;        /* Timer associated with the delay */
  wdparm_t           arg;        /* Callback argument */
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

int wd_start(FAR struct wdog_s *wdog, sclock_t delay,
             wdentry_t wdentry, wdparm_t arg);

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
