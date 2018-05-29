/****************************************************************************
 * libs/libc/time/nanosleep.c
 *
 *   Copyright (C) 2017 Gregory Nutt. All rights reserved.
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
