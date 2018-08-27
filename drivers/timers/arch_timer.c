/****************************************************************************
 * drivers/timers/arch_timer.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/timers/arch_timer.h>

#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS

#  ifdef CONFIG_SCHED_TICKLESS_ALARM
#    error CONFIG_SCHED_TICKLESS_ALARM must be unset to use CONFIG_SCHED_TICKLESS
#  endif

#  ifndef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
#    error CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP must be set to use CONFIG_SCHED_TICKLESS
#  endif

#endif

#define CONFIG_BOARD_LOOPSPER100USEC ((CONFIG_BOARD_LOOPSPERMSEC+5)/10)
#define CONFIG_BOARD_LOOPSPER10USEC  ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)
#define CONFIG_BOARD_LOOPSPERUSEC    ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)

#define TIMER_START(l)               ((l)->ops->start(l))
#define TIMER_GETSTATUS(l,s)         ((l)->ops->getstatus(l,s))
#define TIMER_SETTIMEOUT(l,t)        ((l)->ops->settimeout(l,t))
#define TIMER_SETCALLBACK(l,c,a)     ((l)->ops->setcallback(l,c,a))
#define TIMER_MAXTIMEOUT(l,t)        ((l)->ops->maxtimeout(l,t))

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct arch_timer_s
{
  FAR struct timer_lowerhalf_s *lower;
  uint32_t *next_interval;
  uint32_t maxtimeout;
  uint64_t timebase;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct arch_timer_s g_timer;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
static inline uint64_t timespec_to_usec(const FAR struct timespec *ts)
{
  return (uint64_t)ts->tv_sec * USEC_PER_SEC + ts->tv_nsec / NSEC_PER_USEC;
}

static inline void timespec_from_usec(FAR struct timespec *ts,
                                      uint64_t microseconds)
{
  ts->tv_sec    = microseconds / USEC_PER_SEC;
  microseconds -= (uint64_t)ts->tv_sec * USEC_PER_SEC;
  ts->tv_nsec   = microseconds * NSEC_PER_USEC;
}

static inline bool timeout_diff(uint32_t new, uint32_t old)
{
  return new < old ? old - new >= USEC_PER_TICK : new - old >= USEC_PER_TICK;
}

static uint32_t update_timeout(uint32_t timeout)
{
  struct timer_status_s status;

  /* Don't need critical section here
   * since caller already do it for us
   */

  TIMER_GETSTATUS(g_timer.lower, &status);
  if (g_timer.next_interval)
    {
      /* If the timer interrupt is in the process,
       * let the callback return the right interval.
       */

      *g_timer.next_interval = timeout;
    }
  else if (timeout_diff(timeout, status.timeleft))
    {
      /* Otherwise, update the timeout directly. */

      TIMER_SETTIMEOUT(g_timer.lower, timeout);
      g_timer.timebase += status.timeout - status.timeleft;
    }

  return status.timeleft;
}
#endif

static uint64_t current_usec(void)
{
  struct timer_status_s status;
  uint64_t timebase;
  irqstate_t flags;

  flags = enter_critical_section();
  TIMER_GETSTATUS(g_timer.lower, &status);
  timebase = g_timer.timebase;
  leave_critical_section(flags);

  return timebase + (status.timeout - status.timeleft);
}

static void udelay_accurate(useconds_t microseconds)
{
  uint64_t start = current_usec();
  while (current_usec() - start < microseconds)
    {
      ; /* Wait until the timeout reach */
    }
}

static void udelay_coarse(useconds_t microseconds)
{
  volatile int i;

  /* We'll do this a little at a time because we expect that the
   * CONFIG_BOARD_LOOPSPERUSEC is very inaccurate during to truncation in
   * the divisions of its calculation.  We'll use the largest values that
   * we can in order to prevent significant error buildup in the loops.
   */

  while (microseconds > 1000)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPERMSEC; i++)
        {
        }

      microseconds -= 1000;
    }

  while (microseconds > 100)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPER100USEC; i++)
        {
        }

      microseconds -= 100;
    }

  while (microseconds > 10)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPER10USEC; i++)
        {
        }

      microseconds -= 10;
    }

  while (microseconds > 0)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPERUSEC; i++)
        {
        }

      microseconds--;
    }
}

static bool timer_callback(FAR uint32_t *next_interval_us, FAR void *arg)
{
#ifdef CONFIG_SCHED_TICKLESS
  struct timer_status_s status;
  uint32_t next_interval;

  g_timer.timebase += *next_interval_us;
  next_interval = g_timer.maxtimeout;
  g_timer.next_interval = &next_interval;
  sched_timer_expiration();
  g_timer.next_interval = NULL;

  TIMER_GETSTATUS(g_timer.lower, &status);
  if (timeout_diff(next_interval, status.timeleft))
    {
      g_timer.timebase += status.timeout - status.timeleft;
      *next_interval_us = next_interval;
    }
#else
  g_timer.timebase += USEC_PER_TICK;
  sched_process_timer();
#endif

  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_timer_set_lowerhalf(FAR struct timer_lowerhalf_s *lower)
{
  g_timer.lower = lower;

  TIMER_MAXTIMEOUT(g_timer.lower, &g_timer.maxtimeout);

#ifdef CONFIG_SCHED_TICKLESS
  g_oneshot_maxticks = g_timer.maxtimeout / USEC_PER_TICK;
  TIMER_SETTIMEOUT(g_timer.lower, g_timer.maxtimeout);
#else
  TIMER_SETTIMEOUT(g_timer.lower, USEC_PER_TICK);
#endif

  TIMER_SETCALLBACK(g_timer.lower, timer_callback, NULL);
  TIMER_START(g_timer.lower);
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   the archtecture-specific timer was initialized).  This function is
 *   functionally equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, FAR struct timespec *ts);
 *
 *   when clockid is CLOCK_MONOTONIC.
 *
 *   This function provides the basis for reporting the current time and
 *   also is used to eliminate error build-up from small errors in interval
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

#ifdef CONFIG_CLOCK_TIMEKEEPING
int up_timer_getcounter(FAR uint64_t *cycles)
{
  if (!g_timer.lower)
    {
      *cycles = 0;
      return 0;
    }

  *cycles = current_usec() / USEC_PER_TICK;
  return 0;
}

void up_timer_getmask(FAR uint64_t *mask)
{
  uint32_t maxticks = g_timer.maxtimeout / USEC_PER_TICK;

  *mask = 0;
  while (1)
    {
      uint64_t next = (*mask << 1) | 1;
      if (next > maxticks)
        {
          break;
        }
      *mask = next;
  }
}
#elif defined(CONFIG_SCHED_TICKLESS)
int up_timer_gettime(FAR struct timespec *ts)
{
  if (!g_timer.lower)
    {
      memset(ts, 0, sizeof(*ts));
      return 0;
    }

  timespec_from_usec(ts, current_usec());
  return 0;
}
#endif

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
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the remaining time.  Zero should be returned
 *        if the timer is not active.  ts may be zero in which case the
 *        time remaining is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_timer_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
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
  if (!g_timer.lower)
    {
      return -EAGAIN;
    }

  timespec_from_usec(ts, update_timeout(g_timer.maxtimeout));
  return 0;
}
#endif

/****************************************************************************
 * Name: up_timer_start
 *
 * Description:
 *   Start the interval timer.  sched_timer_expiration() will be called at
 *   the completion of the timeout (unless up_timer_cancel is called to stop
 *   the timing.
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
  if (!g_timer.lower)
    {
      return -EAGAIN;
    }

  update_timeout(timespec_to_usec(ts));
  return 0;
}
#endif

/****************************************************************************
 * Name: up_mdelay
 *
 * Description:
 *   Delay inline for the requested number of milliseconds.
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void up_mdelay(unsigned int milliseconds)
{
  up_udelay(USEC_PER_MSEC * milliseconds);
}

/****************************************************************************
 * Name: up_udelay
 *
 * Description:
 *   Delay inline for the requested number of microseconds.
 *
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void up_udelay(useconds_t microseconds)
{
  if (g_timer.lower)
    {
      udelay_accurate(microseconds);
    }
  else /* period timer doesn't init yet */
    {
      udelay_coarse(microseconds);
    }
}
