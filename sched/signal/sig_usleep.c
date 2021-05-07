/****************************************************************************
 * sched/signal/sig_usleep.c
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
