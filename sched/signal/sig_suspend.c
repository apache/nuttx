/****************************************************************************
 * sched/signal/sig_suspend.c
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

      DEBUGASSERT(!is_idle_task(rtcb));
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
