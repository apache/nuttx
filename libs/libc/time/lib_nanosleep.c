/****************************************************************************
 * libs/libc/time/lib_nanosleep.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nanosleep
 *
 * Description:
 *   The nanosleep() function causes the current thread to be suspended from
 *   execution until either the time interval specified by the rqtp argument
 *   has elapsed or a signal is delivered to the calling thread and its
 *   action is to invoke a signal-catching function or to terminate the
 *   process. The suspension time may be longer than requested because the
 *   argument value is rounded up to an integer multiple of the sleep
 *   resolution or because of the scheduling of other activity by the
 *   system. But, except for the case of being interrupted by a signal, the
 *   suspension time will not be less than the time specified by rqtp, as
 *   measured by the system clock, CLOCK_REALTIME.
 *
 *   The use of the nanosleep() function has no effect on the action or
 *   blockage of any signal.
 *
 * Input Parameters:
 *   rqtp - The amount of time to be suspended from execution.
 *   rmtp - If the rmtp argument is non-NULL, the timespec structure
 *          referenced by it is updated to contain the amount of time
 *          remaining in the interval (the requested time minus the time
 *          actually slept)
 *
 * Returned Value:
 *   If the nanosleep() function returns because the requested time has
 *   elapsed, its return value is zero.
 *
 *   If the nanosleep() function returns because it has been interrupted by
 *   a signal, the function returns a value of -1 and sets errno to indicate
 *   the interruption. If the rmtp argument is non-NULL, the timespec
 *   structure referenced by it is updated to contain the amount of time
 *   remaining in the interval (the requested time minus the time actually
 *   slept). If the rmtp argument is NULL, the remaining time is not
 *   returned.
 *
 *   If nanosleep() fails, it returns a value of -1 and sets errno to
 *   indicate the error. The nanosleep() function will fail if:
 *
 *     EINTR - The nanosleep() function was interrupted by a signal.
 *     EINVAL - The rqtp argument specified a nanosecond value less than
 *       zero or greater than or equal to 1000 million.
 *     ENOSYS - The nanosleep() function is not supported by this
 *       implementation.
 *
 ****************************************************************************/

int nanosleep(FAR const struct timespec *rqtp, FAR struct timespec *rmtp)
{
  /* Calling clock_nanosleep() with the value TIMER_ABSTIME not set in the
   * flags argument and with a clock_id of CLOCK_REALTIME is equivalent t
   * calling nanosleep() with the same rqtp and rmtp arguments.
   */

  return clock_nanosleep(CLOCK_REALTIME, 0, rqtp, rmtp);
}
