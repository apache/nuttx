/****************************************************************************
 * sched/wdog/wd_start.c
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
         start = perf_gettime(); \
         func(arg); \
         elapsed = perf_gettime() - start; \
         if (elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_WDOG) \
           { \
             CRITMONITOR_PANIC("WDOG %p, %s IRQ, execute too long %ju\n", \
                               func, up_interrupt_context() ? \
                               "IN" : "NOT", (uintmax_t)elapsed); \
           } \
       } \
     while (0)
#else
#  define CALL_FUNC(func, arg) func(arg)
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
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void wd_expiration(void)
{
  FAR struct wdog_s *wdog;
  FAR struct wdog_s *next;
  wdentry_t func;

  /* Process the watchdog at the head of the list as well as any
   * other watchdogs that became ready to run at this time
   */

  list_for_every_entry_safe(&g_wdactivelist, wdog,
                            next, struct wdog_s, node)
    {
      if (wdog->lag > 0)
        {
          break;
        }

      /* Remove the watchdog from the head of the list */

      list_delete(&wdog->node);

      /* If there is another watchdog behind this one, update its
       * its lag (this shouldn't be necessary).
       */

      if (!list_is_empty(&g_wdactivelist))
        {
          next->lag += wdog->lag;
        }

      /* Indicate that the watchdog is no longer active. */

      func = wdog->func;
      wdog->func = NULL;

      /* Execute the watchdog function */

      up_setpicbase(wdog->picbase);
      CALL_FUNC(func, wdog->arg);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  FAR struct wdog_s *curr;
  irqstate_t flags;
  sclock_t now;

  /* Verify the wdog and setup parameters */

  if (wdog == NULL || wdentry == NULL || delay < 0)
    {
      return -EINVAL;
    }

  /* Check if the watchdog has been started. If so, stop it.
   * NOTE:  There is a race condition here... the caller may receive
   * the watchdog between the time that wd_start is called and
   * the critical section is established.
   */

  flags = enter_critical_section();
  if (WDOG_ISACTIVE(wdog))
    {
      wd_cancel(wdog);
    }

  /* Save the data in the watchdog structure */

  wdog->func = wdentry;         /* Function to execute when delay expires */
  up_getpicbase(&wdog->picbase);
  wdog->arg = arg;

  /* Calculate delay+1, forcing the delay into a range that we can handle.
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

  if (delay <= 0)
    {
      delay = 1;
    }
  else if (++delay <= 0)
    {
      delay--;
    }

#ifdef CONFIG_SCHED_TICKLESS
  /* Cancel the interval timer that drives the timing events.  This will
   * cause wd_timer to be called which update the delay value for the first
   * time at the head of the timer list (there is a possibility that it
   * could even remove it).
   */

  nxsched_cancel_timer();
#endif

  /* Do the easy case first -- when the watchdog timer queue is empty. */

  if (list_is_empty(&g_wdactivelist))
    {
#ifdef CONFIG_SCHED_TICKLESS
      /* Update clock tickbase */

      g_wdtickbase = clock_systime_ticks();
#endif

      /* Add the watchdog to the head == tail of the queue. */

      list_add_tail(&g_wdactivelist, &wdog->node);
    }

  /* There are other active watchdogs in the timer queue */

  else
    {
      now = 0;

      /* Advance past shorter delays */

      list_for_every_entry(&g_wdactivelist, curr, struct wdog_s, node)
        {
          now += curr->lag;
          if (now > delay)
            {
              break;
            }
        }

      /* Check if the new wdog must be inserted before the curr. */

      if (delay < now)
        {
          /* The relative delay time is smaller or equal to the current delay
           * time, so decrement the current delay time by the new relative
           * delay time.
           */

          delay -= (now - curr->lag);
          curr->lag -= delay;

          /* Insert the new watchdog in the list */

          list_add_before(&curr->node, &wdog->node);
        }

      /* The new watchdog delay time is greater than the curr delay time,
       * so the new wdog must be inserted after the curr. This only occurs
       * if the wdog is to be added to the end of the list.
       */

      else
        {
          delay -= now;

          list_add_tail(&g_wdactivelist, &wdog->node);
        }
    }

  /* Put the lag into the watchdog structure and mark it as active. */

  wdog->lag = delay;

#ifdef CONFIG_SCHED_TICKLESS
  /* Resume the interval timer that will generate the next interval event.
   * If the timer at the head of the list changed, then this will pick that
   * new delay.
   */

  nxsched_resume_timer();
#endif

  leave_critical_section(flags);
  return OK;
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
unsigned int wd_timer(int ticks, bool noswitches)
{
  FAR struct wdog_s *wdog;
  unsigned int ret;
  int decr;

  /* Update clock tickbase */

  g_wdtickbase += ticks;

  /* Check if there are any active watchdogs to process */

  list_for_every_entry(&g_wdactivelist, wdog, struct wdog_s, node)
    {
      if (ticks <= 0)
        {
          break;
        }

      /* Decrement the lag for this watchdog. */

      decr = MIN(wdog->lag, ticks);

      /* There are.  Decrement the lag counter */

      wdog->lag -= decr;
      ticks     -= decr;
    }

  /* Check if the watchdog at the head of the list is ready to run */

  if (!noswitches)
    {
      wd_expiration();
    }

  /* Return the delay for the next watchdog to expire */

  ret = list_is_empty(&g_wdactivelist) ? 0 :
        list_first_entry(&g_wdactivelist, struct wdog_s, node)->lag;

  /* Return the delay for the next watchdog to expire */

  return ret;
}

#else
void wd_timer(void)
{
  /* Check if there are any active watchdogs to process */

  if (!list_is_empty(&g_wdactivelist))
    {
      /* There are.  Decrement the lag counter */

      --(list_first_entry(&g_wdactivelist, struct wdog_s, node)->lag);

      /* Check if the watchdog at the head of the list is ready to run */

      wd_expiration();
    }
}
#endif /* CONFIG_SCHED_TICKLESS */
