/****************************************************************************
 * sched/sig_usleep.c
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

#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/signal.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_usleep
 *
 * Description:
 *   The nxsig_usleep() function will cause the calling thread to be
 *   suspended from execution until either the number of real-time
 *   microseconds specified by the argument 'usec' has elapsed or a signal
 *   is delivered to the calling thread. The suspension time may be longer
 *   than requested due to the scheduling of other activity by the system.
 *
 *   This is an internal OS interface.  It is functionally equivalent to
 *   the standard nxsig_usleep() application interface except that:
 *
 *   - It is not a cancellation point, and
 *   - It does not modify the errno value.
 *
 *   See the description of usleep() for additional information that is not
 *   duplicated here.
 *
 * Input Parameters:
 *   usec - the number of microseconds to wait.
 *
 * Returned Value:
 *   This is an internal OS interface and should not be used by applications.
 *   It follows the NuttX internal error return policy:  Zero (OK) is
 *   returned on success.  A negated errno value is returned on failure.
 *
 ****************************************************************************/

int nxsig_usleep(useconds_t usec)
{
  struct timespec rqtp;
  time_t sec;
  int ret = 0;

  if (usec)
    {
      /* Let nxsig_nanosleep() do all of the work. */

      sec          = usec / 1000000;
      rqtp.tv_sec  = sec;
      rqtp.tv_nsec = (usec - (sec * 1000000)) * 1000;

      ret = nxsig_nanosleep(&rqtp, NULL);
    }

  return ret;
}
