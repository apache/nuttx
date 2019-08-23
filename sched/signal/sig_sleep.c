/****************************************************************************
 * sched/signal/sig_sleep.c
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
