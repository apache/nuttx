/****************************************************************************
 * sched/signal/sig_pselect.c
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

#include "sched/sched.h"
#include "signal/signal.h"

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
  FAR struct tcb_s *rtcb = this_task();
  sigset_t saved_sigprocmask;
  irqstate_t flags;
  int ret = ERROR;

  /* Several operations must be performed below:  We must determine if any
   * signal is pending and, if not, wait for the signal.  Since signals can
   * be posted from the interrupt level, there is a race condition that
   * can only be eliminated by disabling interrupts!
   */

  flags = enter_critical_section();

  /* Save a copy of the old sigprocmask and install
   * the new (temporary) sigprocmask
   */

  saved_sigprocmask = rtcb->sigprocmask;
  if (sigmask)
    {
      rtcb->sigprocmask = *sigmask;
    }

  rtcb->sigwaitmask = NULL_SIGNAL_SET;

  /* Check if there is a pending signal corresponding to one of the
   * signals that will be unblocked by the new sigprocmask.
   */

  if (nxsig_unmask_pendingsignal())
    {
      /* Dispatching one or more of the signals is sufficient to cause
       * us to not wait. Restore the original sigprocmask.
       */

      rtcb->sigprocmask = saved_sigprocmask;
      leave_critical_section(flags);
      set_errno(EINTR);
    }
  else
    {
      FAR struct timeval *timeval = NULL;
      struct timeval timeval_buf;

      /* And call select to do the real work */

      if (timeout)
        {
          timeval_buf.tv_sec  = timeout->tv_sec;
          timeval_buf.tv_usec = timeout->tv_nsec / 1000;
          timeval = &timeval_buf;
        }

      ret = select(nfds, readfds, writefds, exceptfds, timeval);

      /* We are running again, restore the original sigprocmask */

      rtcb->sigprocmask = saved_sigprocmask;
      leave_critical_section(flags);

      /* Now, handle the (rare?) case where (a) a blocked signal was received
       * while the task was suspended but (b) restoring the original
       * sigprocmask will unblock the signal.
       */

      nxsig_unmask_pendingsignal();
    }

  return ret;
}
