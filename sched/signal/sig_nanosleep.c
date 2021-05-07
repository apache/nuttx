/****************************************************************************
 * sched/signal/sig_nanosleep.c
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

#include <time.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/cancelpt.h>

#include "clock/clock.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_nanosleep
 *
 * Description:
 *   The nxsig_nanosleep() function causes the current thread to be
 *   suspended from execution until either the time interval specified by
 *   the rqtp argument has elapsed or a signal is delivered to the calling
 *   thread and its action is to invoke a signal-catching function or to
 *   terminate the process. The suspension time may be longer than requested
 *   because the argument value is rounded up to an integer multiple of the
 *   sleep resolution or because of the scheduling of other activity by the
 *   system. But, except for the case of being interrupted by a signal, the
 *   suspension time will not be less than the time specified by rqtp, as
 *   measured by the system clock, CLOCK_REALTIME.
 *
 *   The use of the nxsig_nanosleep() function has no effect on the action
 *   or blockage of any signal.
 *
 * Input Parameters:
 *   rqtp - The amount of time to be suspended from execution.
 *   rmtp - If the rmtp argument is non-NULL, the timespec structure
 *          referenced by it is updated to contain the amount of time
 *          remaining in the interval (the requested time minus the time
 *          actually slept)
 *
 * Returned Value:
 *   If the nxsig_nanosleep() function returns because the requested time
 *   has elapsed, its return value is zero.
 *
 *   If the nxsig_nanosleep() function returns because it has been
 *   interrupted by a signal, the function returns a negated errno value
 *   indicate the interruption. If the rmtp argument is non-NULL, the
 *   timespec structure referenced by it is updated to contain the amount
 *   of time remaining in the interval (the requested time minus the time
 *   actually slept). If the rmtp argument is NULL, the remaining time is
 *   not returned.
 *
 *   If nxsig_nanosleep() fails, it returns a negated errno indicating the
 *   cause of the failure. The nxsig_nanosleep() function will fail if:
 *
 *     EINTR - The nxsig_nanosleep() function was interrupted by a signal.
 *     EINVAL - The rqtp argument specified a nanosecond value less than
 *       zero or greater than or equal to 1000 million.
 *     ENOSYS - The nxsig_nanosleep() function is not supported by this
 *       implementation.
 *
 ****************************************************************************/

int nxsig_nanosleep(FAR const struct timespec *rqtp,
                    FAR struct timespec *rmtp)
{
  irqstate_t flags;
  clock_t starttick;
  sigset_t set;
  int ret;

  /* Sanity check */

  if (rqtp == NULL || rqtp->tv_nsec < 0 || rqtp->tv_nsec >= 1000000000)
    {
      return -EINVAL;
    }

  /* Get the start time of the wait.  Interrupts are disabled to prevent
   * timer interrupts while we do tick-related calculations before and
   * after the wait.
   */

  flags     = enter_critical_section();
  starttick = clock_systime_ticks();

  /* Set up for the sleep.  Using the empty set means that we are not
   * waiting for any particular signal.  However, any unmasked signal can
   * still awaken nxsig_timedwait().
   */

  sigemptyset(&set);

  /* nxsig_nanosleep is a simple application of nxsig_timedwait. */

  ret = nxsig_timedwait(&set, NULL, rqtp);

  /* nxsig_timedwait() cannot succeed.  It should always return error with
   * either (1) EAGAIN meaning that the timeout occurred, or (2) EINTR
   * meaning that some other unblocked signal was caught.
   */

  if (ret == -EAGAIN)
    {
      /* The timeout "error" is the normal, successful result */

      leave_critical_section(flags);
      return OK;
    }

  /* If we get there, the wait has failed because we were awakened by a
   * signal.  Return the amount of "unwaited" time if rmtp is non-NULL.
   */

  if (rmtp)
    {
      clock_t elapsed;
      clock_t remaining;
      sclock_t ticks;

      /* REVISIT: The conversion from time to ticks and back could
       * be avoided.  clock_timespec_subtract() would be used instead
       * to get the time difference.
       */

      /* First get the number of clock ticks that we were requested to
       * wait.
       */

      clock_time2ticks(rqtp, &ticks);

      /* Get the number of ticks that we actually waited */

      elapsed = clock_systime_ticks() - starttick;

      /* The difference between the number of ticks that we were requested
       * to wait and the number of ticks that we actually waited is that
       * amount of time that we failed to wait.
       */

      if (elapsed >= (clock_t)ticks)
        {
          remaining = 0;
        }
      else
        {
          remaining = (clock_t)ticks - elapsed;
        }

      clock_ticks2time((sclock_t)remaining, rmtp);
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: clock_nanosleep
 *
 * Description:
 *   If the flag TIMER_ABSTIME is not set in the flags argument, the
 *   clock_nanosleep() function will cause the current thread to be suspended
 *   from execution until either the time interval specified by the rqtp
 *   argument has elapsed, or a signal is delivered to the calling thread
 *   and its action is to invoke a signal-catching function, or the process
 *   is terminated. The clock used to measure the time will be the clock
 *   specified by clock_id.
 *
 *   If the flag TIMER_ABSTIME is set in the flags argument, the
 *   clock_nanosleep() function will cause the current thread to be
 *   suspended from execution until either the time value of the clock
 *   specified by clock_id reaches the absolute time specified by the rqtp
 *   argument, or a signal is delivered to the calling thread and its action
 *   is to invoke a signal-catching function, or the process is terminated.
 *   If, at the time of the call, the time value specified by rqtp is less
 *   than or equal to the time value of the specified clock, then
 *   clock_nanosleep() will return immediately and the calling process will
 *   not be suspended.
 *
 *   The suspension time caused by this function may be longer than requested
 *   because the argument value is rounded up to an integer multiple of the
 *   sleep resolution, or because of the scheduling of other activity by the
 *   system. But, except for the case of being interrupted by a signal, the
 *   suspension time for the relative clock_nanosleep() function (that is,
 *   with the TIMER_ABSTIME flag not set) will not be less than the time
 *   interval specified by rqtp, as measured by the corresponding clock. The
 *   suspension for the absolute clock_nanosleep() function (that is, with
 *   the TIMER_ABSTIME flag set) will be in effect at least until the value
 *   of the corresponding clock reaches the absolute time specified by rqtp,
 *   except for the case of being interrupted by a signal.
 *
 *   The use of the clock_nanosleep() function will have no effect on the
 *   action or blockage of any signal.
 *
 *   The clock_nanosleep() function will fail if the clock_id argument refers
 *   to the CPU-time clock of the calling thread. It is unspecified whether
 *   clock_id values of other CPU-time clocks are allowed.
 *
 * Input Parameters:
 *   clockid - The clock to use to interpret the absolute time
 *   flags   - Open flags.  TIMER_ABSTIME  is the only supported flag.
 *   rqtp    - The amount of time to be suspended from execution.
 *   rmtp    - If the rmtp argument is non-NULL, the timespec structure
 *             referenced by it is updated to contain the amount of time
 *             remaining in the interval (the requested time minus the time
 *             actually slept)
 *
 * Returned Value:
 *   If the clock_nanosleep() function returns because the requested time has
 *   elapsed, its return value is zero.
 *
 *   If the clock_nanosleep() function returns because it has been
 *   interrupted by a signal, the function returns a value of -1 and sets
 *   errno to indicate the interruption. If the rmtp argument is non-NULL,
 *   the timespec structure referenced by it is updated to contain the amount
 *   of time remaining in the interval (the requested time minus the time
 *   actually slept). If the rmtp argument is NULL, the remaining time is not
 *   returned.
 *
 *   If clock_nanosleep() fails, it returns a value of -1 and sets errno to
 *   indicate the error. The clock_nanosleep() function will fail if:
 *
 *     EINTR - The clock_nanosleep() function was interrupted by a signal.
 *     EINVAL - The rqtp argument specified a nanosecond value less than
 *       zero or greater than or equal to 1000 million.
 *     ENOSYS - The clock_nanosleep() function is not supported by this
 *       implementation.
 *
 ****************************************************************************/

int clock_nanosleep(clockid_t clockid, int flags,
                    FAR const struct timespec *rqtp,
                    FAR struct timespec *rmtp)
{
  int ret;

  /* clock_nanosleep() is a cancellation point */

  enter_cancellation_point();

  /* Check if absolute time is selected */

  if ((flags & TIMER_ABSTIME) != 0)
    {
      struct timespec reltime;
      struct timespec now;
      irqstate_t irqstate;

      /* Calculate the relative time delay.  We need to enter a critical
       * section early to assure the relative time is valid from this
       * point in time.
       */

      irqstate = enter_critical_section();
      ret = clock_gettime(clockid, &now);
      if (ret < 0)
        {
          /* clock_gettime() sets the errno variable */

          leave_critical_section(irqstate);
          leave_cancellation_point();
          return ERROR;
        }

      clock_timespec_subtract(rqtp, &now, &reltime);

      /* Now that we have the relative time, the remaining operations
       * are equivalent to nxsig_nanosleep().
       */

      ret = nxsig_nanosleep(&reltime, rmtp);
      leave_critical_section(irqstate);
    }
  else
    {
      /* In the relative time case, clock_nanosleep() is equivalent to
       * nanosleep.  In this case, it is a paper thin wrapper around
       * nxsig_nanosleep().
       */

      ret = nxsig_nanosleep(rqtp, rmtp);
    }

  /* Check if nxsig_nanosleep() succeeded */

  if (ret < 0)
    {
      /* If not set the errno variable and return -1 */

      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
