/****************************************************************************
 * libs/libc/unistd/lib_sleep.c
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

#include <unistd.h>
#include <signal.h>

#include <nuttx/clock.h>
#include <arch/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sleep
 *
 * Description:
 *   The sleep() function will cause the calling thread to be suspended from
 *   execution until either the number of real-time seconds specified by the
 *   argument 'seconds' has elapsed or a signal is delivered to the calling
 *   thread and its action is to invoke a signal-catching function or to
 *   terminate the process. The suspension time may be longer than requested
 *   due to the scheduling of other activity by the system.
 *
 *   If a SIGALRM signal is generated for the calling process during
 *   execution of sleep() and if the SIGALRM signal is being ignored or
 *   blocked from delivery, it is unspecified whether sleep() returns
 *   when the SIGALRM signal is scheduled. If the signal is being blocked, it
 *   is also unspecified whether it remains pending after sleep() returns or
 *   it is discarded.
 *
 *   If a SIGALRM signal is generated for the calling process during
 *   execution of sleep(), except as a result of a prior call to alarm(),
 *   and if the SIGALRM signal is not being ignored or blocked from delivery,
 *   it is unspecified whether that signal has any effect other than causing
 *   sleep() to return.
 *
 *   If a signal-catching function interrupts sleep() and examines or changes
 *   either the time a SIGALRM is scheduled to be generated, the action
 *   associated with the SIGALRM signal, or whether the SIGALRM signal is
 *   blocked from delivery, the results are unspecified.
 *
 *   If a signal-catching function interrupts sleep() and calls siglongjmp()
 *   or longjmp() to restore an environment saved prior to the sleep() call,
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
 *   Interactions between sleep() and any of setitimer(), ualarm() or sleep()
 *   are unspecified.
 *
 * Input Parameters:
 *   seconds - The number of seconds to sleep
 *
 * Returned Value:
 *   If sleep() returns because the requested time has elapsed, the value
 *   returned will be 0. If sleep() returns because of premature arousal due
 *   to delivery of a signal, the return value will be the "unslept" amount
 *   (the requested time minus the time actually slept) in seconds.
 *
 * Assumptions:
 *
 ****************************************************************************/

unsigned int sleep(unsigned int seconds)
{
  struct timespec rqtp;
  struct timespec rmtp;
  unsigned int remaining = 0;
  int ret;

  /* Don't sleep if seconds == 0 */

  if (seconds > 0)
    {
      /* Let clock_nanosleep() do all of the work. */

      rqtp.tv_sec  = seconds;
      rqtp.tv_nsec = 0;

      ret = clock_nanosleep(CLOCK_REALTIME, 0, &rqtp, &rmtp);

      /* clock_nanosleep() should only fail if it was interrupted by a
       * signal, but we treat all errors the same,
       */

      if (ret < 0)
        {
          remaining = rmtp.tv_sec;
          if (remaining < seconds && rmtp.tv_nsec >= 500000000)
            {
              /* Round up */

              remaining++;
            }
        }

      return remaining;
    }

  return 0;
}
