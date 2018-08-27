/****************************************************************************
 * drivers/timers/arch_alarm.c
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
#include <nuttx/timers/arch_alarm.h>

#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS

#  ifndef CONFIG_SCHED_TICKLESS_ALARM
#    error CONFIG_SCHED_TICKLESS_ALARM must be set to use CONFIG_SCHED_TICKLESS
#  endif

#  ifndef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
#    error CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP must be set to use CONFIG_SCHED_TICKLESS
#  endif

#endif

#define CONFIG_BOARD_LOOPSPER100USEC ((CONFIG_BOARD_LOOPSPERMSEC+5)/10)
#define CONFIG_BOARD_LOOPSPER10USEC  ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)
#define CONFIG_BOARD_LOOPSPERUSEC    ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)

#define timespec_to_usec(ts) \
    ((uint64_t)(ts)->tv_sec * USEC_PER_SEC + (ts)->tv_nsec / NSEC_PER_USEC)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct oneshot_lowerhalf_s *g_oneshot_lower;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void timespec_from_usec(FAR struct timespec *ts,
                                      unsigned int microseconds)
{
  ts->tv_sec    = microseconds / USEC_PER_SEC;
  microseconds -= (uint64_t)ts->tv_sec * USEC_PER_SEC;
  ts->tv_nsec   = microseconds * NSEC_PER_USEC;
}

static inline int timespec_compare(FAR const struct timespec *ts1,
                                   FAR const struct timespec *ts2)
{
  if (ts1->tv_sec != ts2->tv_sec)
    {
      return ts1->tv_sec - ts2->tv_sec;
    }
  else
    {
      return ts1->tv_nsec - ts2->tv_nsec;
    }
}

static void udelay_accurate(useconds_t microseconds)
{
  struct timespec now;
  struct timespec end;
  struct timespec delta;

  ONESHOT_CURRENT(g_oneshot_lower, &now);
  timespec_from_usec(&delta, microseconds);
  clock_timespec_add(&now, &delta, &end);

  while (timespec_compare(&now, &end) < 0)
    {
      ONESHOT_CURRENT(g_oneshot_lower, &now);
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

static void oneshot_callback(FAR struct oneshot_lowerhalf_s *lower, FAR void *arg)
{
#ifdef CONFIG_SCHED_TICKLESS
  struct timespec now;

  ONESHOT_CURRENT(g_oneshot_lower, &now);
  sched_alarm_expiration(&now);
#else
  struct timespec now;
  struct timespec next;
  struct timespec delta;
  static uint64_t tick = 1;

  timespec_from_usec(&next, ++tick * USEC_PER_TICK);
  ONESHOT_CURRENT(g_oneshot_lower, &now);
  clock_timespec_subtract(&next, &now, &delta);
  ONESHOT_START(g_oneshot_lower, oneshot_callback, NULL, &delta);
  sched_process_timer();
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_alarm_set_lowerhalf(FAR struct oneshot_lowerhalf_s *lower)
{
#ifdef CONFIG_SCHED_TICKLESS
  struct timespec maxts;
  uint64_t maxticks;

  g_oneshot_lower = lower;
  ONESHOT_MAX_DELAY(g_oneshot_lower, &maxts);
  maxticks = timespec_to_usec(&maxts) / USEC_PER_TICK;
  g_oneshot_maxticks = maxticks < UINT32_MAX ? maxticks : UINT32_MAX;
#else
  struct timespec ts;

  g_oneshot_lower = lower;
  timespec_from_usec(&ts, USEC_PER_TICK);
  ONESHOT_START(g_oneshot_lower, oneshot_callback, NULL, &ts);
#endif
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
  struct timespec now;

  if (!g_oneshot_lower)
    {
      *cycles = 0;
      return 0;
    }

  ONESHOT_CURRENT(g_oneshot_lower, &now);
  *cycles = timespec_to_usec(&now) / USEC_PER_TICK;
  return 0;
}

void up_timer_getmask(FAR uint64_t *mask)
{
  struct timespec maxts;
  uint64_t maxticks = 0;

  if (g_oneshot_lower)
    {
      ONESHOT_MAX_DELAY(g_oneshot_lower, &maxts);
      maxticks = timespec_to_usec(&maxts) / USEC_PER_TICK;
    }

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
  if (!g_oneshot_lower)
    {
      memset(ts, 0, sizeof(*ts));
      return 0;
    }

  ONESHOT_CURRENT(g_oneshot_lower, ts);
  return 0;
}
#endif

/****************************************************************************
 * Name: up_alarm_cancel
 *
 * Description:
 *   Cancel the alarm and return the time of cancellation of the alarm.
 *   These two steps need to be as nearly atomic as possible.
 *   sched_alarm_expiration() will not be called unless the alarm is
 *   restarted with up_alarm_start().
 *
 *   If, as a race condition, the alarm has already expired when this
 *   function is called, then time returned is the current time.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the expiration time.  The current time should
 *        returned if the alarm is not active.  ts may be NULL in which
 *        case the time is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_alarm_cancel() when
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
int up_alarm_cancel(FAR struct timespec *ts)
{
  if (!g_oneshot_lower)
    {
      return -EAGAIN;
    }

  ONESHOT_CANCEL(g_oneshot_lower, ts);
  ONESHOT_CURRENT(g_oneshot_lower, ts);
  return 0;
}
#endif

/****************************************************************************
 * Name: up_alarm_start
 *
 * Description:
 *   Start the alarm.  sched_alarm_expiration() will be called when the
 *   alarm occurs (unless up_alaram_cancel is called to stop it).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - The time in the future at the alarm is expected to occur.  When
 *        the alarm occurs the timer logic will call sched_alarm_expiration().
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
int up_alarm_start(FAR const struct timespec *ts)
{
  struct timespec now;
  struct timespec delta;

  if (!g_oneshot_lower)
    {
      return -EAGAIN;
    }

  ONESHOT_CURRENT(g_oneshot_lower, &now);
  clock_timespec_subtract(ts, &now, &delta);
  ONESHOT_START(g_oneshot_lower, oneshot_callback, NULL, &delta);
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
  if (g_oneshot_lower)
    {
      udelay_accurate(microseconds);
    }
  else /* oneshot timer doesn't init yet */
    {
      udelay_coarse(microseconds);
    }
}
