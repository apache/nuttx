/****************************************************************************
 * sched/signal/sig_pselect.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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
