/****************************************************************************
 * sched/signal/sig_ppoll.c
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

#include <poll.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ppoll
 *
 * Description:
 *   ppoll() waits for one of a set of file descriptors to become ready to
 *   perform I/O.  If none of the events requested (and no error) has
 *   occurred for any of  the  file  descriptors,  then ppoll() blocks until
 *   one of the events occurs.
 *
 * Input Parameters:
 *   fds  - List of structures describing file descriptors to be monitored
 *   nfds - The number of entries in the list
 *   timeout - Specifies an upper limit on the time for which ppoll() will
 *     block.  A NULL pointer means an infinite timeout.
 *   sigmask - Replace the current signal mask temporarily during execution
 *
 * Returned Value:
 *   On success, the number of structures that have non-zero revents fields.
 *   A value of 0 indicates that the call timed out and no file descriptors
 *   were ready.  On error, -1 is returned, and errno is set appropriately:
 *
 *   EBADF  - An invalid file descriptor was given in one of the sets.
 *   EFAULT - The fds address is invalid
 *   EINTR  - A signal occurred before any requested event.
 *   EINVAL - The nfds value exceeds a system limit.
 *   ENOMEM - There was no space to allocate internal data structures.
 *   ENOSYS - One or more of the drivers supporting the file descriptor
 *     does not support the poll method.
 *
 ****************************************************************************/

int ppoll(FAR struct pollfd *fds, nfds_t nfds,
          FAR const struct timespec *timeout_ts,
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

  sigemptyset(&rtcb->sigwaitmask);

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
      int timeout = -1;

      /* And call poll to do the real work */

      if (timeout_ts)
        {
          timeout = timeout_ts->tv_sec * 1000 +
                    timeout_ts->tv_nsec / 1000000;
        }

      ret = poll(fds, nfds, timeout);

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
