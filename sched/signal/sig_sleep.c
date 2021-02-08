/****************************************************************************
 * sched/signal/sig_sleep.c
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

#include <signal.h>

#include <nuttx/clock.h>
#include <nuttx/signal.h>

#include <arch/irq.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_sleep
 *
 * Description:
 *   The nxsig_sleep() function will cause the calling thread to be
 *   suspended from execution until either the number of real-time seconds
 *   specified by the argument 'seconds' has elapsed or a signal is
 *   delivered to the calling thread.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   the standard sleep() application interface except that:
 *
 *   - It is not a cancellation point, and
 *   - There is no check that the action of the signal is to invoke a
 *     signal-catching function or to terminate the process.
 *
 *   See the description of sleep() for additional information that is not
 *   duplicated here.
 *
 * Input Parameters:
 *   seconds - The number of seconds to sleep
 *
 * Returned Value:
 *   If nxsig_sleep() returns because the requested time has elapsed, the
 *   value returned will be zero (OK). If nxsig_sleep() returns because of
 *   premature arousal due to delivery of a signal, the return value will
 *   be the "unslept" amount (the requested time minus the time actually
 *   slept) in seconds.
 *
 ****************************************************************************/

unsigned int nxsig_sleep(unsigned int seconds)
{
  struct timespec rqtp;
  struct timespec rmtp;
  unsigned int remaining = 0;
  int ret;

  /* Don't sleep if seconds == 0 */

  if (seconds > 0)
    {
      /* Let nxsig_nanosleep() do all of the work. */

      rqtp.tv_sec  = seconds;
      rqtp.tv_nsec = 0;

      ret = nxsig_nanosleep(&rqtp, &rmtp);

      /* nanosleep() should only fail if it was interrupted by a signal,
       * but we treat all errors the same,
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
