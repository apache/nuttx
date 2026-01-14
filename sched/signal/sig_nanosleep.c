/****************************************************************************
 * sched/signal/sig_nanosleep.c
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
  int ret;

  ret = nxsig_clockwait(CLOCK_REALTIME, 0, rqtp, rmtp);

  return ret == -EAGAIN ? 0 : ret;
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
 *   If clock_nanosleep() fails, it returns a value of errno. The
 *   clock_nanosleep() function will fail if:
 *
 *     EINTR - The clock_nanosleep() function was interrupted by a signal.
 *     EINVAL - The rqtp argument specified a nanosecond value less than
 *       zero or greater than or equal to 1000 million. Or the clockid that
 *       does not specify a known clock.
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

  if (clockid < CLOCK_REALTIME || clockid > CLOCK_BOOTTIME)
    {
      leave_cancellation_point();
      return EINVAL;
    }

  /* Just a wrapper around nxsig_clockwait() */

  ret = nxsig_clockwait(clockid, flags, rqtp, rmtp);

  /* Check if nxsig_clockwait() succeeded */

  if (ret == -EAGAIN)
    {
      ret = OK;
    }
  else if (ret < 0)
    {
      /* If not return the errno */

      ret = -ret;
    }

  leave_cancellation_point();
  return ret;
}
