/****************************************************************************
 * sched/signal/sig_pause.c
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
#include <errno.h>

#include <nuttx/signal.h>
#include <nuttx/cancelpt.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pause
 *
 * Description:
 *   The pause() function will suspend the calling thread until delivery of a
 *   non-blocked signal.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Since pause() suspends thread execution indefinitely unless interrupted
 *   a signal, there is no successful completion return value. A value of -1
 *   will always be returned and errno set to indicate the error (EINTR).
 *
 * POSIX compatibility:
 *   In the POSIX description of this function is the pause() function will
 *   suspend the calling thread until delivery of a signal whose action is
 *   either to execute a signal-catching function or to terminate the
 *   process.  This implementation only waits for any non-blocked signal
 *   to be received.
 *
 ****************************************************************************/

int pause(void)
{
  sigset_t set;
  int ret;

  /* pause() is a cancellation point */

  enter_cancellation_point();

  /* Set up for the sleep.  Using the empty set means that we are not
   * waiting for any particular signal.  However, any unmasked signal
   * can still awaken sigtimedwait().
   */

  sigemptyset(&set);

  /* sigtwaitinfo() cannot succeed.  It should always return error EINTR
   * meaning that some unblocked signal was caught.
   */

  ret = nxsig_waitinfo(&set, NULL);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  leave_cancellation_point();
  return ret;
}
