/****************************************************************************
 * drivers/timers/arch_timer.c
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

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/timers/arch_timer.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SCHED_TICKLESS) && defined(CONFIG_SCHED_TICKLESS_ALARM)
#  error CONFIG_SCHED_TICKLESS_ALARM must be unset to use the arch timer
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

static inline void timespec_from_usec(FAR struct timespec *ts,
                                      uint64_t microseconds)
{
  ts->tv_sec    = microseconds / USEC_PER_SEC;
  microseconds -= (uint64_t)ts->tv_sec * USEC_PER_SEC;
  ts->tv_nsec   = microseconds * NSEC_PER_USEC;
}

#ifdef CONFIG_SCHED_TICKLESS
static inline uint64_t timespec_to_usec(FAR const struct timespec *ts)
{
  return (uint64_t)ts->tv_sec * USEC_PER_SEC + ts->tv_nsec / NSEC_PER_USEC;
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

  do
    {
      timebase = g_timer.timebase;
      TIMER_GETSTATUS(g_timer.lower, &status);
    }
  while (timebase != g_timer.timebase);

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

  g_timer.timebase     += *next_interval_us;
  next_interval         = g_timer.maxtimeout;
  g_timer.next_interval = &next_interval;
  nxsched_timer_expiration();
  g_timer.next_interval = NULL;

  TIMER_GETSTATUS(g_timer.lower, &status);
  if (timeout_diff(next_interval, status.timeleft))
    {
      g_timer.timebase += status.timeout - status.timeleft;
      *next_interval_us = next_interval;
    }

#else
  g_timer.timebase += USEC_PER_TICK;
  nxsched_process_timer();
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
 *   the architecture-specific timer was initialized).  This function is
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
int weak_function up_timer_getcounter(FAR uint64_t *cycles)
{
  int ret = -EAGAIN;

  if (g_timer.lower != NULL)
    {
      *cycles = current_usec() / USEC_PER_TICK;
      ret = 0;
    }

  return ret;
}

void weak_function up_timer_getmask(FAR uint64_t *mask)
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
#endif

#if defined(CONFIG_SCHED_TICKLESS)
int weak_function up_timer_gettime(FAR struct timespec *ts)
{
  int ret = -EAGAIN;

  if (g_timer.lower != NULL)
    {
      timespec_from_usec(ts, current_usec());
      ret = 0;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_timer_cancel
 *
 * Description:
 *   Cancel the interval timer and return the time remaining on the timer.
 *   These two steps need to be as nearly atomic as possible.
 *   nxsched_timer_expiration() will not be called unless the timer is
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
int weak_function up_timer_cancel(FAR struct timespec *ts)
{
  int ret = -EAGAIN;

  if (g_timer.lower != NULL)
    {
      timespec_from_usec(ts, update_timeout(g_timer.maxtimeout));
      ret = 0;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_timer_start
 *
 * Description:
 *   Start the interval timer.  nxsched_timer_expiration() will be called at
 *   the completion of the timeout (unless up_timer_cancel is called to stop
 *   the timing.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the time interval until nxsched_timer_expiration() is
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
int weak_function up_timer_start(FAR const struct timespec *ts)
{
  int ret = -EAGAIN;

  if (g_timer.lower != NULL)
    {
      update_timeout(timespec_to_usec(ts));
      ret = 0;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_perf_*
 *
 * Description:
 *   The first interface simply provides the current time value in unknown
 *   units.  NOTE:  This function may be called early before the timer has
 *   been initialized.  In that event, the function should just return a
 *   start time of zero.
 *
 *   Nothing is assumed about the units of this time value.  The following
 *   are assumed, however: (1) The time is an unsigned integer value, (2)
 *   the time is monotonically increasing, and (3) the elapsed time (also
 *   in unknown units) can be obtained by subtracting a start time from
 *   the current time.
 *
 *   The second interface simple converts an elapsed time into well known
 *   units.
 ****************************************************************************/

uint32_t weak_function up_perf_gettime(void)
{
  uint32_t ret = 0;

  if (g_timer.lower != NULL)
    {
      ret = current_usec();
    }

  return ret;
}

uint32_t weak_function up_perf_getfreq(void)
{
  return USEC_PER_SEC;
}

void weak_function up_perf_convert(uint32_t elapsed, FAR struct timespec *ts)
{
  timespec_from_usec(ts, elapsed);
}

/****************************************************************************
 * Name: up_mdelay
 *
 * Description:
 *   Delay inline for the requested number of milliseconds.
 *   *** NOT multi-tasking friendly ***
 *
 ****************************************************************************/

void weak_function up_mdelay(unsigned int milliseconds)
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

void weak_function up_udelay(useconds_t microseconds)
{
  if (g_timer.lower != NULL)
    {
      udelay_accurate(microseconds);
    }
  else /* Period timer hasn't been initialized yet */
    {
      udelay_coarse(microseconds);
    }
}
