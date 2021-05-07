/****************************************************************************
 * libs/libc/unistd/lib_usleep.c
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
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usleep
 *
 * Description:
 *   The usleep() function will cause the calling thread to be suspended
 *   from execution until either the number of real-time microseconds
 *   specified by the argument 'usec' has elapsed or a signal is delivered
 *   to the calling thread. The suspension time may be longer than requested
 *   due to the scheduling of other activity by the system.
 *
 *   The 'usec' argument must be less than 1,000,000. If the value of
 *   'usec' is 0, then the call has no effect.
 *
 *   If a SIGALRM signal is generated for the calling process during
 *   execution of usleep() and if the SIGALRM signal is being ignored or
 *   blocked from delivery, it is unspecified whether usleep() returns
 *   when the SIGALRM signal is scheduled. If the signal is being blocked, it
 *   is also unspecified whether it remains pending after usleep() returns or
 *   it is discarded.
 *
 *   If a SIGALRM signal is generated for the calling process during
 *   execution of usleep(), except as a result of a prior call to alarm(),
 *   and if the SIGALRM signal is not being ignored or blocked from delivery,
 *   it is unspecified whether that signal has any effect other than causing
 *   usleep() to return.
 *
 *   If a signal-catching function interrupts usleep() and examines or
 *   changes either the time a SIGALRM is scheduled to be generated, the
 *   action associated with the SIGALRM signal, or whether the SIGALRM signal
 *   is blocked from delivery, the results are unspecified.
 *
 *   If a signal-catching function interrupts usleep() and calls siglongjmp()
 *   or longjmp() to restore an environment saved prior to the usleep() call,
 *   the action associated with the SIGALRM signal and the time at which a
 *   SIGALRM signal is scheduled to be generated are unspecified. It is also
 *   unspecified whether the SIGALRM signal is blocked, unless the process'
 *   signal mask is restored as part of the environment.
 *
 *   Implementations may place limitations on the granularity of timer
 *   values. For each interval timer, if the requested timer value requires a
 *   finer granularity than the implementation supports, the actual timer
 *   value will be rounded up to the next supported value.
 *
 *   Interactions between usleep() and any of the following are unspecified:
 *
 *   nanosleep(), setitimer(), timer_create(), timer_delete(),
 *   timer_getoverrun(), timer_gettime(), timer_settime(), ualarm(), sleep()
 *
 * Input Parameters:
 *   usec - the number of microseconds to wait.
 *
 * Returned Value:
 *   On successful completion, usleep() returns 0. Otherwise, it returns -1
 *   and sets errno to indicate the error.
 *
 * Assumptions:
 *
 ****************************************************************************/

int usleep(useconds_t usec)
{
  struct timespec rqtp;
  time_t sec;
  int ret = 0;

  if (usec)
    {
      /* Let clock_nanosleep() do all of the work. */

      sec          = usec / 1000000;
      rqtp.tv_sec  = sec;
      rqtp.tv_nsec = (usec - (sec * 1000000)) * 1000;

      ret = clock_nanosleep(CLOCK_REALTIME, 0, &rqtp, NULL);
    }

  return ret;
}
