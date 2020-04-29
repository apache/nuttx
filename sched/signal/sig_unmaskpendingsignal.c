/****************************************************************************
 * sched/signal/sig_unmaskpendingsignal.c
 *
 *   Copyright (C) 2007, 2009, 2013, 2017 Gregory Nutt. All rights reserved.
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

  unmaskedset = ~(rtcb->sigprocmask) & nxsig_pendingset(rtcb);
  if (unmaskedset == NULL_SIGNAL_SET)
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
  while (unmaskedset != NULL_SIGNAL_SET);

  sched_unlock();
  return true;
}
