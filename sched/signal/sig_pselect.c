/****************************************************************************
 * sched/signal/sig_pselect.c
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

#include <sys/select.h>
#include <sys/time.h>

#include <nuttx/fs/fs.h>

#include "sched/sched.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pselect
 *
 * Description:
 *   pselect() allows a program to monitor multiple file descriptors, waiting
 *   until one or more of the file descriptors become "ready" for some class
 *   of I/O operation (e.g., input possible).  A file descriptor is
 *   considered  ready if it is possible to perform the corresponding I/O
 *   operation (e.g., read(2)) without blocking.
 *
 * Input Parameters:
 *   nfds - the maximum fd number (+1) of any descriptor in any of the
 *     three sets.
 *   readfds - the set of descriptions to monitor for read-ready events
 *   writefds - the set of descriptions to monitor for write-ready events
 *   exceptfds - the set of descriptions to monitor for error events
 *   timeout - Return at this time if none of these events of interest
 *     occur.
 *   sigmask - Replace the current signal mask temporarily during execution
 *
 *  Returned Value:
 *   0: Timer expired
 *  >0: The number of bits set in the three sets of descriptors
 *  -1: An error occurred (errno will be set appropriately)
 *
 ****************************************************************************/

int pselect(int nfds, FAR fd_set *readfds, FAR fd_set *writefds,
            FAR fd_set *exceptfds, FAR const struct timespec *timeout,
            FAR const sigset_t *sigmask)
{
  FAR struct timeval *timeval = NULL;
  struct timeval timeval_buf;
  sigset_t mask;
  int ret;

  if (timeout)
    {
      timeval_buf.tv_sec  = timeout->tv_sec;
      timeval_buf.tv_usec = timeout->tv_nsec / NSEC_PER_USEC;
      timeval = &timeval_buf;
    }

  /* A NULL mask keeps the caller's mask but still checks pending signals.
   * nx_poll() sets the mask after the poll setup: the poll methods do not
   * run in the critical section that closes the window to the wait.
   */

  if (sigmask == NULL)
    {
      mask = this_task()->sigprocmask;
      sigmask = &mask;
    }

  ret = nx_select(nfds, readfds, writefds, exceptfds, timeval, sigmask);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return ret;
}
