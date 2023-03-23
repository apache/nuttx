/****************************************************************************
 * sched/signal/sig_unmaskpendingsignal.c
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
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_unmask_pendingsignal
 *
 * Description:
 *   Based upon the current setting of the sigprocmask, this function
 *   unmasks and processes any pending signals.  This function should
 *   be called whenever the sigprocmask is changed.
 *
 ****************************************************************************/

bool nxsig_unmask_pendingsignal(void)
{
  FAR struct tcb_s *rtcb = this_task();
  sigset_t unmaskedset;
  FAR sigpendq_t *pendingsig;
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
              /* If there is one, then process it like a normal signal.
               * Since the signal was pending, then unblocked on this
               * thread, we can skip the normal group signal dispatching
               * rules; there can be no other recipient for the signal
               * other than this thread.
               */

              nxsig_tcbdispatch(rtcb, &pendingsig->info);

              /* Then remove it from the pending signal list */

              nxsig_release_pendingsignal(pendingsig);
            }
        }
    }
  while (!sigisemptyset(&unmaskedset));

  sched_unlock();
  return true;
}
