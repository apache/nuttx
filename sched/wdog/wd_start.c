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
#include <unistd.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/wdog.h>

#include "sched/sched.h"
#include "wdog/wdog.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef MIN
#  define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef CONFIG_SCHED_CRITMONITOR_MAXTIME_WDOG
#  define CONFIG_SCHED_CRITMONITOR_MAXTIME_WDOG 0
#endif

#if CONFIG_SCHED_CRITMONITOR_MAXTIME_WDOG > 0
#  define CALL_FUNC(func, arg) \
     do \
       { \
         uint32_t start; \
         uint32_t elapsed; \
         start = up_critmon_gettime(); \
         func(arg); \
         elapsed = up_critmon_gettime() - start; \
         if (elapsed > CONFIG_SCHED_CRITMONITOR_MAXTIME_WDOG) \
           { \
             serr("WDOG %p, %s IRQ, execute too long %"PRIu32"\n", \
                   func, up_interrupt_context() ? "IN" : "NOT", elapsed); \
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
  wdentry_t func;

  /* Check if the watchdog at the head of the list is ready to run */

  if (((FAR struct wdog_s *)g_wdactivelist.head)->lag <= 0)
    {
      /* Process the watchdog at the head of the list as well as any
       * other watchdogs that became ready to run at this time
       */

      while (g_wdactivelist.head &&
             ((FAR struct wdog_s *)g_wdactivelist.head)->lag <= 0)
        {
          /* Remove the watchdog from the head of the list */

          wdog = (FAR struct wdog_s *)sq_remfirst(&g_wdactivelist);

          /* If there is another watchdog behind this one, update its
           * its lag (this shouldn't be necessary).
           */

          if (g_wdactivelist.head)
            {
              ((FAR struct wdog_s *)g_wdactivelist.head)->lag += wdog->lag;
            }

          /* Indicate that the watchdog is no longer active. */

          func = wdog->func;
          wdog->func = NULL;

          /* Execute the watchdog function */

          up_setpicbase(wdog->picbase);
          CALL_FUNC(func, wdog->arg);
        }
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

int wd_start(FAR struct wdog_s *wdog, int32_t delay,
             wdentry_t wdentry, wdparm_t arg)
{
  FAR struct wdog_s *curr;
  FAR struct wdog_s *prev;
  FAR struct wdog_s *next;
  int32_t now;
  irqstate_t flags;

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

  /* Calculate delay+1, forcing the delay into a range that we can handle */

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

  if (g_wdactivelist.head == NULL)
    {
#ifdef CONFIG_SCHED_TICKLESS
      /* Update clock tickbase */

      g_wdtickbase = clock_systime_ticks();
#endif

      /* Add the watchdog to the head == tail of the queue. */

      sq_addlast((FAR sq_entry_t *)wdog, &g_wdactivelist);
    }

  /* There are other active watchdogs in the timer queue */

  else
    {
      now = 0;
      prev = curr = (FAR struct wdog_s *)g_wdactivelist.head;

      /* Advance to positive time */

      while ((now += curr->lag) < 0 && curr->next)
        {
          prev = curr;
          curr = curr->next;
        }

      /* Advance past shorter delays */

      while (now <= delay && curr->next)
        {
          prev = curr;
          curr = curr->next;
          now += curr->lag;
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

          if (curr == (FAR struct wdog_s *)g_wdactivelist.head)
            {
              /* Insert the watchdog at the head of the list */

              sq_addfirst((FAR sq_entry_t *)wdog, &g_wdactivelist);
            }
          else
            {
              /* Insert the watchdog in mid- or end-of-queue */

              sq_addafter((FAR sq_entry_t *)prev, (FAR sq_entry_t *)wdog,
                          &g_wdactivelist);
            }
        }

      /* The new watchdog delay time is greater than the curr delay time,
       * so the new wdog must be inserted after the curr. This only occurs
       * if the wdog is to be added to the end of the list.
       */

      else
        {
          delay -= now;
          if (!curr->next)
            {
              sq_addlast((FAR sq_entry_t *)wdog, &g_wdactivelist);
            }
          else
            {
              next = curr->next;
              next->lag -= delay;
              sq_addafter((FAR sq_entry_t *)curr, (FAR sq_entry_t *)wdog,
                          &g_wdactivelist);
            }
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
unsigned int wd_timer(int ticks)
{
  FAR struct wdog_s *wdog;
#ifdef CONFIG_SMP
  irqstate_t flags;
#endif
  unsigned int ret;
  int decr;

#ifdef CONFIG_SMP
  /* We are in an interrupt handler as, as a consequence, interrupts are
   * disabled.  But in the SMP case, interrupts MAY be disabled only on
   * the local CPU since most architectures do not permit disabling
   * interrupts on other CPUS.
   *
   * Hence, we must follow rules for critical sections even here in the
   * SMP case.
   */

  flags = enter_critical_section();
#endif

  /* Check if there are any active watchdogs to process */

  while (g_wdactivelist.head != NULL && ticks > 0)
    {
      /* Get the watchdog at the head of the list */

      wdog = (FAR struct wdog_s *)g_wdactivelist.head;

#ifndef CONFIG_SCHED_TICKLESS_ALARM
      /* There is logic to handle the case where ticks is greater than
       * the watchdog lag, but if the scheduling is working properly
       * that should never happen.
       */

      DEBUGASSERT(ticks <= wdog->lag);
#endif
      /* Decrement the lag for this watchdog. */

      decr = MIN(wdog->lag, ticks);

      /* There are.  Decrement the lag counter */

      wdog->lag    -= decr;
      ticks        -= decr;
      g_wdtickbase += decr;

      /* Check if the watchdog at the head of the list is ready to run */

      wd_expiration();
    }

  /* Update clock tickbase */

  g_wdtickbase += ticks;

  /* Return the delay for the next watchdog to expire */

  ret = g_wdactivelist.head ?
          ((FAR struct wdog_s *)g_wdactivelist.head)->lag : 0;

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif

  /* Return the delay for the next watchdog to expire */

  return ret;
}

#else
void wd_timer(void)
{
#ifdef CONFIG_SMP
  irqstate_t flags;

  /* We are in an interrupt handler as, as a consequence, interrupts are
   * disabled.  But in the SMP case, interrupts MAY be disabled only on
   * the local CPU since most architectures do not permit disabling
   * interrupts on other CPUS.
   *
   * Hence, we must follow rules for critical sections even here in the
   * SMP case.
   */

  flags = enter_critical_section();
#endif

  /* Check if there are any active watchdogs to process */

  if (g_wdactivelist.head)
    {
      /* There are.  Decrement the lag counter */

      --(((FAR struct wdog_s *)g_wdactivelist.head)->lag);

      /* Check if the watchdog at the head of the list is ready to run */

      wd_expiration();
    }

#ifdef CONFIG_SMP
  leave_critical_section(flags);
#endif
}
#endif /* CONFIG_SCHED_TICKLESS */
