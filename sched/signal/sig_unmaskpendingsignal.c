/****************************************************************************
 * sched/signal/sig_unmaskpendingsignal.c
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

#include <sched.h>

#include <nuttx/signal.h>

#include "sched/sched.h"
#include "signal/signal.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_dispatch_unmasked
 *
 * Description:
 *   Based upon the current setting of the sigprocmask, this function
 *   unmasks and processes any pending signals.  This function should
 *   be called whenever the sigprocmask is changed.
 *
 ****************************************************************************/

static bool nxsig_dispatch_unmasked(FAR struct tcb_s *rtcb)
{
  sigset_t unmaskedset;
  FAR sigpendq_t *pendingsig;
  FAR const void *source;
  siginfo_t info;
  bool group;
  int signo;

  /* Prohibit any context switches until we are done with this.
   * We may still be performing signal operations from interrupt
   * handlers, however, none of the pending signals that we
   * are concerned with here should be effected.
   */

  sched_lock();

  /* Get the set of pending signals that were just unmasked.  The
   * following operation should be safe because the sigprocmask
   * can only be changed on this thread of execution.
   */

  unmaskedset = nxsig_pendingset(rtcb);
  nxsig_nandset(&unmaskedset, &unmaskedset, &rtcb->sigprocmask);
  if (sigisemptyset(&unmaskedset))
    {
      sched_unlock();
      return false;
    }

  /* Loop while there are unmasked pending signals to be processed. */

  do
    {
      /* Pending signals will be processed from lowest numbered signal
       * to highest
       */

      signo = nxsig_lowest(&unmaskedset);
      if (signo != ERROR)
        {
          /* Remove the signal from the set of unmasked signals.  NOTE:
           * this implicitly assumes that only one instance for a given
           * signal number is pending.
           */

          nxsig_delset(&unmaskedset, signo);

          /* Remove the pending signal from the list of pending signals */

          if ((pendingsig = nxsig_remove_pendingsignal(rtcb, signo)) != NULL)
            {
              /* Release the entry first: the action can end the process
               * (the default action of SIGTERM, for example), and then
               * nxsig_tcbdispatch() does not return.
               */

              info   = pendingsig->info;
              group  = pendingsig->tcb == NULL;
              source = pendingsig->source;
              nxsig_release_pendingsignal(pendingsig);

              /* Process it like a normal signal.  Since the signal was
               * pending, then unblocked on this thread, we can skip the
               * normal group signal dispatching rules; there can be no
               * other recipient for the signal other than this thread.
               */

              nxsig_tcbdispatch_source(rtcb, &info, group, source);
            }
        }
    }
  while (!sigisemptyset(&unmaskedset));

  sched_unlock();
  return true;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_unmask_pendingsignal
 *
 * Description:
 *   Dispatch the pending signals that the signal mask of the calling thread
 *   does not block.  At the end of a system call, also restore the mask
 *   that nxsig_restoremask() saved, after the signals that the temporary
 *   mask of pselect(), ppoll() or sigsuspend() unblocks are dispatched.
 *
 * Returned Value:
 *   true if one or more signals were dispatched.
 *
 ****************************************************************************/

bool nxsig_unmask_pendingsignal(void)
{
  FAR struct tcb_s *rtcb = this_task();
  bool dispatched;

  dispatched = nxsig_dispatch_unmasked(rtcb);

  if ((rtcb->flags & (TCB_FLAG_SYSCALL | TCB_FLAG_SIGMASK_SAVED)) ==
      TCB_FLAG_SIGMASK_SAVED)
    {
      rtcb->sigprocmask = rtcb->sigsavedmask;
      rtcb->flags &= ~TCB_FLAG_SIGMASK_SAVED;

      if (nxsig_dispatch_unmasked(rtcb))
        {
          dispatched = true;
        }
    }

  return dispatched;
}

/****************************************************************************
 * Name: nxsig_restoremask
 *
 * Description:
 *   Restore the signal mask after the temporary mask of pselect(), ppoll()
 *   or sigsuspend().  In a system call, a signal that the temporary mask
 *   unblocked is still on the pending list: it is dispatched only at the
 *   end of the system call.  So keep the temporary mask until then, and
 *   let nxsig_unmask_pendingsignal() restore this one.
 *
 * Input Parameters:
 *   rtcb - The calling thread.
 *   mask - The signal mask to restore.
 *
 ****************************************************************************/

void nxsig_restoremask(FAR struct tcb_s *rtcb, FAR const sigset_t *mask)
{
  if ((rtcb->flags & TCB_FLAG_SYSCALL) != 0)
    {
      rtcb->sigsavedmask = *mask;
      rtcb->flags |= TCB_FLAG_SIGMASK_SAVED;
    }
  else
    {
      rtcb->sigprocmask = *mask;
    }
}
