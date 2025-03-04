/****************************************************************************
 * sched/wdog/wd_start.c
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

#include <sys/types.h>

#include <stdint.h>
#include <stdbool.h>
#include <sys/param.h>
#include <unistd.h>
#include <sched.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>
#include <nuttx/sched_note.h>

#include "sched/sched.h"
#include "wdog/wdog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SCHED_CRITMONITOR_MAXTIME_WDOG
#  define CONFIG_SCHED_CRITMONITOR_MAXTIME_WDOG 0
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_WDOG > 0
#  define CALL_FUNC(func, arg) \
     do \
       { \
         clock_t start; \
         clock_t elapsed; \
         sched_note_wdog(NOTE_WDOG_ENTER, func, (FAR void *)arg); \
         start = perf_gettime(); \
         func(arg); \
         elapsed = perf_gettime() - start; \
         sched_note_wdog(NOTE_WDOG_LEAVE, func, (FAR void *)arg); \
         if (elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_WDOG) \
           { \
             CRITMONITOR_PANIC("WDOG %p, %s IRQ, execute too long %ju\n", \
                               func, up_interrupt_context() ? \
                               "IN" : "NOT", (uintmax_t)elapsed); \
           } \
       } \
     while (0)
#else
#  define CALL_FUNC(func, arg) \
      do \
        { \
          sched_note_wdog(NOTE_WDOG_ENTER, func, (FAR void *)arg); \
          func(arg); \
          sched_note_wdog(NOTE_WDOG_LEAVE, func, (FAR void *)arg); \
        } \
      while (0)

#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
static unsigned int g_wdtimernested;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: wd_expiration
 *
 * Description:
 *   Check if the timer for the watchdog at the head of list is ready to
 *   run. If so, remove the watchdog from the list and execute it.
 *
 * Input Parameters:
 *   ticks - current time in ticks
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline_function void wd_expiration(clock_t ticks)
{
  FAR struct wdog_s *wdog;
  irqstate_t flags;
  wdentry_t func;
  wdparm_t arg;

  flags = spin_lock_irqsave(&g_wdspinlock);

#ifdef CONFIG_SCHED_TICKLESS
  /* Increment the nested watchdog timer count to handle cases where wd_start
   * is called in the watchdog callback functions.
   */

  g_wdtimernested++;
#endif

  /* Process the watchdog at the head of the list as well as any
   * other watchdogs that became ready to run at this time
   */

  while (!list_is_empty(&g_wdactivelist))
    {
      wdog = list_first_entry(&g_wdactivelist, struct wdog_s, node);

      /* Check if expected time is expired */

      if (!clock_compare(wdog->expired, ticks))
        {
          break;
        }

      /* Remove the watchdog from the head of the list */

      list_delete(&wdog->node);

      /* Indicate that the watchdog is no longer active. */

      func = wdog->func;
      arg = wdog->arg;
      wdog->func = NULL;

      /* Execute the watchdog function */

      up_setpicbase(wdog->picbase);
      spin_unlock_irqrestore(&g_wdspinlock, flags);

      CALL_FUNC(func, arg);

      flags = spin_lock_irqsave(&g_wdspinlock);
    }

#ifdef CONFIG_SCHED_TICKLESS
  /* Decrement the nested watchdog timer count */

  g_wdtimernested--;
#endif

  spin_unlock_irqrestore(&g_wdspinlock, flags);
}

/****************************************************************************
 * Name: wd_insert
 *
 * Description:
 *   Insert the timer into the global list to ensure that
 *   the list is sorted in increasing order of expiration absolute time.
 *
 * Input Parameters:
 *   wdog     - Watchdog ID
 *   expired  - expired absolute time in clock ticks
 *   wdentry  - Function to call on timeout
 *   arg      - Parameter to pass to wdentry
 *
 * Assumptions:
 *   wdog and wdentry is not NULL.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static inline_function
void wd_insert(FAR struct wdog_s *wdog, clock_t expired,
               wdentry_t wdentry, wdparm_t arg)
{
  FAR struct wdog_s *curr;

  /* Traverse the watchdog list */

  list_for_every_entry(&g_wdactivelist, curr, struct wdog_s, node)
    {
      /* Until curr->expired has not timed out relative to expired */

      if (!clock_compare(curr->expired, expired))
        {
          break;
        }
    }

  /* There are two cases:
   * - Traverse to the end, where curr == &g_wdactivelist.
   * - Find a curr such that curr->expected has not timed out
   * relative to expired.
   * In either case 1 or 2, we just insert the wdog before curr.
   */

  list_add_before(&curr->node, &wdog->node);

  wdog->func = wdentry;
  up_getpicbase(&wdog->picbase);
  wdog->arg = arg;
  wdog->expired = expired;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
 *   ticks    - Absoulute time in clock ticks
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
                     wdentry_t wdentry, wdparm_t arg)
{
  irqstate_t flags;
  bool reassess = false;

  /* Verify the wdog and setup parameters */

  if (wdog == NULL || wdentry == NULL)
    {
      return -EINVAL;
    }

  /* Calculate ticks+1, forcing the delay into a range that we can handle.
   *
   * NOTE that one is added to the delay.  This is correct and must not be
   * changed:  The contract for the use wdog_start is that the wdog will
   * delay FOR AT LEAST as long as requested, but may delay longer due to
   * variety of factors.  The wdog logic has no knowledge of the the phase
   * of the system timer when it is started:  The next timer interrupt may
   * occur immediately or may be delayed for almost a full cycle. In order
   * to meet the contract requirement, the requested time is also always
   * incremented by one so that the delay is always at least as long as
   * requested.
   *
   * There is extensive documentation about this time issue elsewhere.
   */

  ticks++;

  /* NOTE:  There is a race condition here... the caller may receive
   * the watchdog between the time that wd_start_abstick is called and
   * the critical section is established.
   */

  flags = spin_lock_irqsave(&g_wdspinlock);
#ifdef CONFIG_SCHED_TICKLESS
  /* We need to reassess timer if the watchdog list head has changed. */

  if (WDOG_ISACTIVE(wdog))
    {
      reassess |= list_is_head(&g_wdactivelist, &wdog->node);
      list_delete(&wdog->node);
      wdog->func = NULL;
    }

  wd_insert(wdog, ticks, wdentry, arg);

  if (!g_wdtimernested &&
      (reassess || list_is_head(&g_wdactivelist, &wdog->node)))
    {
      /* Resume the interval timer that will generate the next
       * interval event. If the timer at the head of the list changed,
       * then this will pick that new delay.
       */

      spin_unlock_irqrestore(&g_wdspinlock, flags);
      nxsched_reassess_timer();
    }
  else
    {
      spin_unlock_irqrestore(&g_wdspinlock, flags);
    }
#else
  UNUSED(reassess);

  /* Check if the watchdog has been started. If so, delete it. */

  if (WDOG_ISACTIVE(wdog))
    {
      list_delete(&wdog->node);
      wdog->func = NULL;
    }

  wd_insert(wdog, ticks, wdentry, arg);
  spin_unlock_irqrestore(&g_wdspinlock, flags);
#endif

  sched_note_wdog(NOTE_WDOG_START, wdentry, (FAR void *)(uintptr_t)ticks);
  return OK;
}

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
 *   arg      - Parameter to pass to wdentry
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
             wdentry_t wdentry, wdparm_t arg)
{
  /* Verify the wdog and setup parameters */

  if (delay < 0)
    {
      return -EINVAL;
    }

  return wd_start_abstick(wdog, clock_systime_ticks() + delay,
                          wdentry, arg);
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

#ifdef CONFIG_SCHED_TICKLESS
clock_t wd_timer(clock_t ticks, bool noswitches)
{
  FAR struct wdog_s *wdog;
  irqstate_t flags;
  sclock_t ret;

  /* Check if the watchdog at the head of the list is ready to run */

  if (!noswitches)
    {
      wd_expiration(ticks);
    }

  flags = spin_lock_irqsave(&g_wdspinlock);

  /* Return the delay for the next watchdog to expire */

  if (list_is_empty(&g_wdactivelist))
    {
      spin_unlock_irqrestore(&g_wdspinlock, flags);
      return 0;
    }

  /* Notice that if noswitches, expired - g_wdtickbase
   * may get negative value.
   */

  wdog = list_first_entry(&g_wdactivelist, struct wdog_s, node);
  ret = wdog->expired - ticks;

  spin_unlock_irqrestore(&g_wdspinlock, flags);

  /* Return the delay for the next watchdog to expire */

  return MAX(ret, 1);
}

#else
void wd_timer(clock_t ticks)
{
  /* Check if there are any active watchdogs to process */

  wd_expiration(ticks);
}
#endif /* CONFIG_SCHED_TICKLESS */
