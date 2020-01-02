/****************************************************************************
 * sched/signal/sig_suspend.c
 *
 *   Copyright (C) 2007-2009, 2013, 2016 Gregory Nutt. All rights reserved.
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
#include <assert.h>
#include <debug.h>
#include <sched.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sigsuspend
 *
 * Description:
 *
 *   The sigsuspend() function replaces the signal mask of the task with the
 *   set of signals pointed to by the argument 'set' and then suspends the
 *   process until delivery of a signal to the task.
 *
 *   If the effect of the set argument is to unblock a pending signal, then
 *   no wait is performed.
 *
 *   The original signal mask is restored when this function returns.
 *
 *   Waiting for an empty signal set stops a task without freeing any
 *   resources.
 *
 * Input Parameters:
 *   set - signal mask to use while suspended.
 *
 * Returned Value:
 *   -1 (ERROR) always
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 *   int sigsuspend(const sigset_t *set);
 *
 *   POSIX states that sigsuspend() "suspends the process until delivery of
 *   a signal whose action is either to execute a signal-catching function
 *   or to terminate the process."  Only the deliver of a signal is required
 *   in the present implementation (even if the signal is ignored).
 *
 ****************************************************************************/

int sigsuspend(FAR const sigset_t *set)
{
  FAR struct tcb_s *rtcb = this_task();
  sigset_t saved_sigprocmask;
  irqstate_t flags;

  /* sigsuspend() is a cancellation point */

  enter_cancellation_point();

  /* Several operations must be performed below:  We must determine if any
   * signal is pending and, if not, wait for the signal.  Since signals can
   * be posted from the interrupt level, there is a race condition that
   * can only be eliminated by disabling interrupts!
   */

  sched_lock();  /* Not necessary */
  flags = enter_critical_section();

  /* Save a copy of the old sigprocmask and install
   * the new (temporary) sigprocmask
   */

  saved_sigprocmask = rtcb->sigprocmask;
  rtcb->sigprocmask = *set;
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
    }
  else
    {
      /* Its time to wait until one of the unblocked signals is posted,
       * but first, ensure this is not the idle task, descheduling that
       * isn't going to end well.
       */

      DEBUGASSERT(NULL != rtcb->flink);
      up_block_task(rtcb, TSTATE_WAIT_SIG);

      /* We are running again, restore the original sigprocmask */

      rtcb->sigprocmask = saved_sigprocmask;
      leave_critical_section(flags);

      /* Now, handle the (rare?) case where (a) a blocked signal was received
       * while the task was suspended but (b) restoring the original
       * sigprocmask will unblock the signal.
       */

      nxsig_unmask_pendingsignal();
    }

  sched_unlock();
  leave_cancellation_point();
  set_errno(EINTR);
  return ERROR;
}
