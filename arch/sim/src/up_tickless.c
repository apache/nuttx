/****************************************************************************
 * arch/sim/src/up_tickless.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/****************************************************************************
 * Tickless OS Support.
 *
 * When CONFIG_SCHED_TICKLESS is enabled, all support for timer interrupts
 * is suppressed and the platform specific code is expected to provide the
 * following custom functions.
 *
 *   void sim_timer_initialize(void): Initializes the timer facilities.  Called
 *     early in the intialization sequence (by up_intialize()).
 *   int up_timer_gettime(FAR struct timespec *ts):  Returns the current
 *     time from the platform specific time source.
 *   int up_timer_cancel(void):  Cancels the interval timer.
 *   int up_timer_start(FAR const struct timespec *ts): Start (or re-starts)
 *     the interval timer.
 *
 * The RTOS will provide the following interfaces for use by the platform-
 * specific interval timer implementation:
 *
 *   void sched_timer_expiration(void):  Called by the platform-specific
 *     logic when the interval timer expires.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#ifdef CONFIG_SCHED_TICKLESS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SIM_WALLTIME) || defined(CONFIG_SIM_X11FB)
#  define TICK_USEC (USEC_PER_SEC / CLK_TCK)
#  define TICK_SEC  (TICK_USEC / USEC_PER_SEC)
#  define TICK_NSEC ((TICK_USEC % NSEC_PER_USEC) * NSEC_PER_USEC)
#else
#  define TICK_SEC  0
#  define TICK_NSEC NSEC_PER_TICK
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct timespec g_elapsed_time;
static struct timespec g_interval_delay;
static bool g_timer_active;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sim_timer_initialize
 *
 * Description:
 *   Initializes all platform-specific timer facilities.  This function is
 *   called early in the initialization sequence by up_intialize().
 *   On return, the current up-time should be available from
 *   up_timer_gettime() and the interval timer is ready for use (but not
 *   actively timing.
 *
 *   Provided by platform-specific code and called from the architecture-
 *   specific logic.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *   Called early in the initialization sequence before any special
 *   concurrency protections are required.
 *
 ****************************************************************************/

void sim_timer_initialize(void)
{
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   sim_timer_initialize() was called).  This function is functionally
 *   equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, FAR struct timespec *ts);
 *
 *   when clockid is CLOCK_MONOTONIC.
 *
 *   This function provides the basis for reporting the current time and
 *   also is used to eliminate error build-up from small erros in interval
 *   time calculations.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the location in which to return the up-time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   Called from the normal tasking context.  The implementation must
 *   provide whatever mutual exclusion is necessary for correct operation.
 *   This can include disabling interrupts in order to assure atomic register
 *   operations.
 *
 ****************************************************************************/

int up_timer_gettime(FAR struct timespec *ts)
{
  ts->tv_sec  = g_elapsed_time.tv_sec;
  ts->tv_nsec = g_elapsed_time.tv_nsec;
  return OK;
}

/****************************************************************************
 * Name: up_timer_cancel
 *
 * Description:
 *   Cancel the interval timer and return the time remaining on the timer.
 *   These two steps need to be as nearly atomic as possible.
 *   sched_timer_expiration() will not be called unless the timer is
 *   restarted with up_timer_start().
 *
 *   If, as a race condition, the timer has already expired when this
 *   function is called, then that pending interrupt must be cleared so
 *   that up_timer_start() and the remaining time of zero should be
 *   returned.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the remaining time.  Zero should be returned
 *        if the timer is not active.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
int up_timer_cancel(FAR struct timespec *ts)
{
  /* Return the time remaining on the simulated timer */

  if (g_timer_active)
    {
      ts->tv_sec  = g_interval_delay.tv_sec;
      ts->tv_nsec = g_interval_delay.tv_nsec;
    }
  else
    {
      ts->tv_sec  = 0;
      ts->tv_nsec = 0;
    }

  /* Disable and reset the simulated timer */

  g_interval_delay.tv_sec  = 0;
  g_interval_delay.tv_nsec = 0;
  g_timer_active           = false;
}
#endif

/****************************************************************************
 * Name: up_timer_start
 *
 * Description:
 *   Start the interval timer.  sched_timer_expiration() will be
 *   called at the completion of the timeout (unless up_timer_cancel
 *   is called to stop the timing.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the time interval until sched_timer_expiration() is
 *        called.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
int up_timer_start(FAR const struct timespec *ts)
{
  g_interval_delay.tv_sec  = ts->tv_sec;
  g_interval_delay.tv_nsec = ts->tv_nsec;
  g_timer_active           = true;
}
#endif

/****************************************************************************
 * Name: up_timer_update
 *
 * Description:
 *   Called from the IDLE loop to fake one timer tick.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_timer_update(void)
{
  /* Increment the elapsed time */

  g_elapsed_time.tv_nsec += TICK_NSEC;
  if (g_elapsed_time.tv_nsec >= NSEC_PER_SEC)
    {
      g_elapsed_time.tv_sec++;
      g_elapsed_time.tv_nsec -= NSEC_PER_SEC;
    }

  g_elapsed_time.tv_sec += TICK_SEC;

  /* Is the interval timer active? */

  if (g_timer_active)
    {
      /* Yes... decrement the interval timer */

      if (g_interval_delay.tv_sec < TICK_SEC)
        {
          /* No more seconds left... the timer has expired */

          g_timer_active = false;
          sched_timer_expiration();
        }
      else
        {
          /* Decrement seconds.  May decrement to zero */

          g_interval_delay.tv_sec -= TICK_SEC;

          /* Decrement nanoseconds */

          if (g_interval_delay.tv_nsec >= TICK_NSEC)
            {
              g_interval_delay.tv_nsec -= TICK_NSEC;
            }

          /* Handle borrow from seconds */

          else if (g_interval_delay.tv_sec > 0)
            {
              g_interval_delay.tv_nsec += NSEC_PER_SEC;
              g_interval_delay.tv_sec--;

              g_interval_delay.tv_nsec -= TICK_NSEC;
            }

          /* Otherwise the timer has expired */

          else
            {
              g_timer_active = false;
              sched_timer_expiration();
            }
        }
    }
}

#endif /* CONFIG_SCHED_TICKLESS */
