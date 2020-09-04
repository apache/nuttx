/****************************************************************************
 * drivers/timers/arch_alarm.c
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
#include <nuttx/timers/arch_alarm.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
                                      uint64_t microseconds)
{
  ts->tv_sec    = microseconds / USEC_PER_SEC;
  microseconds -= (uint64_t)ts->tv_sec * USEC_PER_SEC;
  ts->tv_nsec   = microseconds * NSEC_PER_USEC;
}

static void udelay_accurate(useconds_t microseconds)
{
  struct timespec now;
  struct timespec end;
  struct timespec delta;

  ONESHOT_CURRENT(g_oneshot_lower, &now);
  timespec_from_usec(&delta, microseconds);
  clock_timespec_add(&now, &delta, &end);

  while (clock_timespec_compare(&now, &end) < 0)
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

static void oneshot_callback(FAR struct oneshot_lowerhalf_s *lower,
                             FAR void *arg)
{
  struct timespec now;

#ifdef CONFIG_SCHED_TICKLESS
  ONESHOT_CURRENT(g_oneshot_lower, &now);
  nxsched_alarm_expiration(&now);
#else
  struct timespec delta;

  do
    {
      static uint64_t tick = 1;
      struct timespec next;

      nxsched_process_timer();
      timespec_from_usec(&next, ++tick * USEC_PER_TICK);
      ONESHOT_CURRENT(g_oneshot_lower, &now);
      clock_timespec_subtract(&next, &now, &delta);
    }
  while (delta.tv_sec == 0 && delta.tv_nsec == 0);

  ONESHOT_START(g_oneshot_lower, oneshot_callback, NULL, &delta);
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
  int ret = -EAGAIN;

  if (g_oneshot_lower != NULL)
    {
      struct timespec now;

      ret = ONESHOT_CURRENT(g_oneshot_lower, &now);
      if (ret == 0)
        {
          *cycles = timespec_to_usec(&now) / USEC_PER_TICK;
        }
    }

  return ret;
}

void up_timer_getmask(FAR uint64_t *mask)
{
  *mask = 0;

  if (g_oneshot_lower != NULL)
    {
      struct timespec maxts;
      uint64_t maxticks;

      ONESHOT_MAX_DELAY(g_oneshot_lower, &maxts);
      maxticks = timespec_to_usec(&maxts) / USEC_PER_TICK;

      for (; ; )
        {
          uint64_t next = (*mask << 1) | 1;
          if (next > maxticks)
            {
              break;
            }

          *mask = next;
        }
    }
}
#elif defined(CONFIG_SCHED_TICKLESS)
int up_timer_gettime(FAR struct timespec *ts)
{
  int ret = -EAGAIN;

  if (g_oneshot_lower != NULL)
    {
      ret = ONESHOT_CURRENT(g_oneshot_lower, ts);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_alarm_cancel
 *
 * Description:
 *   Cancel the alarm and return the time of cancellation of the alarm.
 *   These two steps need to be as nearly atomic as possible.
 *   nxsched_alarm_expiration() will not be called unless the alarm is
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
  int ret = -EAGAIN;

  if (g_oneshot_lower != NULL)
    {
      ret = ONESHOT_CANCEL(g_oneshot_lower, ts);
      ONESHOT_CURRENT(g_oneshot_lower, ts);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_alarm_start
 *
 * Description:
 *   Start the alarm.  nxsched_alarm_expiration() will be called when the
 *   alarm occurs (unless up_alaram_cancel is called to stop it).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - The time in the future at the alarm is expected to occur. When the
 *        alarm occurs the timer logic will call nxsched_alarm_expiration().
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
  int ret = -EAGAIN;

  if (g_oneshot_lower != NULL)
    {
      struct timespec now;
      struct timespec delta;

      ONESHOT_CURRENT(g_oneshot_lower, &now);
      clock_timespec_subtract(ts, &now, &delta);
      ret = ONESHOT_START(g_oneshot_lower, oneshot_callback, NULL, &delta);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_critmon_*
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

#ifdef CONFIG_SCHED_CRITMONITOR
uint32_t up_critmon_gettime(void)
{
  uint32_t ret = 0;

  if (g_oneshot_lower != NULL)
    {
      struct timespec ts;

      ONESHOT_CURRENT(g_oneshot_lower, &ts);
      ret = timespec_to_usec(&ts);
    }

  return ret;
}

void up_critmon_convert(uint32_t elapsed, FAR struct timespec *ts)
{
  timespec_from_usec(ts, elapsed);
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
  if (g_oneshot_lower != NULL)
    {
      udelay_accurate(microseconds);
    }
  else /* Oneshot timer hasn't been initialized yet */
    {
      udelay_coarse(microseconds);
    }
}
