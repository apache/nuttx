/****************************************************************************
 * sched/signal/sig_deliver.c
 *
 *   Copyright (C) 2007, 2008, 2012-2013, 2016, 2019 Gregory Nutt.  All
 *     rights reserved.
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

#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <sched.h>
#include <string.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "semaphore/semaphore.h"
#include "signal/signal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_deliver
 *
 * Description:
 *   This function is called on the thread of execution of the signal
 *   receiving task.  It processes all queued signals then returns.
 *
 ****************************************************************************/

void nxsig_deliver(FAR struct tcb_s *stcb)
{
  FAR sigq_t *sigq;
  sigset_t    savesigprocmask;
  sigset_t    newsigprocmask;
  sigset_t    altsigprocmask;
  irqstate_t  flags;
  int         saved_errno;

  /* Loop while there are signals to be delivered */

  for (; ; )
    {
      /* Test if this task is already handling a signal (we don't permit
       * nested signals on the same thread).
       */

      flags = enter_critical_section();
      if ((stcb->flags & TCB_FLAG_SIGNAL_ACTION) != 0)
        {
          /* Yes.. then we must wait for the signal handler to return */

          leave_critical_section(flags);
          break;
        }

      /* Remove the signal structure from the head of the sigpendactionq. */

      sigq  = (FAR sigq_t *)sq_remfirst(&stcb->sigpendactionq);
      if (sigq == NULL)
        {
          /* All queued signal actions have been dispatched */

          leave_critical_section(flags);
          break;
        }

      /* Indicate that a signal is being delivered */

      stcb->flags |= TCB_FLAG_SIGNAL_ACTION;

      sinfo("Deliver signal %d to PID %d\n",
            sigq->info.si_signo, stcb->pid);

      /* Add the signal structure to the sigpostedq.  NOTE:  Since signals
       * are processed one at a time, there should never be more than one
       * signal in the sigpostedq
       */

      sq_addlast((FAR sq_entry_t *)sigq, &(stcb->sigpostedq));

      /* Save the thread errno.  When we finished dispatching the
       * signal actions and resume the task, the errno value must
       * be unchanged by the operation of the signal handling.  In
       * particular, the EINTR indication that says that the task
       * was reawakened by a signal must be retained.
       */

      saved_errno       = stcb->pterrno;

      /* Save a copy of the old sigprocmask and install the new
       * (temporary) sigprocmask.  The new sigprocmask is the union
       * of the current sigprocmask and the sa_mask for the signal being
       * delivered plus the signal being delivered.
       */

      savesigprocmask   = stcb->sigprocmask;
      newsigprocmask    = savesigprocmask | sigq->mask |
                          SIGNO2SET(sigq->info.si_signo);
      stcb->sigprocmask = newsigprocmask;

#if defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)
      /* In the kernel build this has to be handled differently if we are
       * dispatching to a signal handler in a user-space task or thread; we
       * have to switch to user-mode before calling the task.
       */

#ifdef CONFIG_SIG_DEFAULT
      /* The default signal action handlers, however always reside in the
       * kernel address space, regardless of configuration.
       */

      if (nxsig_isdefault(stcb, sigq->info.si_signo))
        {
          /* Leave the critical section befor calling the handler */

          leave_critical_section(flags);
          (*sigq->action.sighandler)(sigq->info.si_signo, &sigq->info,
                                     NULL);
        }
      else
#endif
      if ((stcb->flags & TCB_FLAG_TTYPE_MASK) != TCB_FLAG_TTYPE_KERNEL)
        {
          siginfo_t info;

          /* The sigq_t pointed to by sigq resides in kernel space.  So we
           * cannot pass a reference to sigq->info to the user application.
           * Instead, we will copy the siginfo_t structure onto the stack.
           * We are currently executing on the stack of the user thread
           * (albeit temporarily in kernel mode), so the copy of the
           * siginfo_t structure will be accessible by the user thread.
           */

          memcpy(&info, &sigq->info, sizeof(siginfo_t));

          /* Leave the critical section befor calling the handler */

          leave_critical_section(flags);
          up_signal_dispatch(sigq->action.sighandler, sigq->info.si_signo,
                             &info, NULL);
        }
      else
#endif
        {
          /* The kernel thread signal handler is much simpler. */

          leave_critical_section(flags);
          (*sigq->action.sighandler)(sigq->info.si_signo, &sigq->info,
                                     NULL);
        }

      /* Indicate that a signal has been delivered */

      flags             = enter_critical_section();
      stcb->flags       &= ~TCB_FLAG_SIGNAL_ACTION;

      /* Restore the original errno value and sigprocmask. */

      stcb->pterrno     = saved_errno;

      /* What if the signal handler changed the sigprocmask?  Try to retain
       * any such changes here.
       *
       * REVISIT: This logic is imperfect.  It will fail to detect bits set
       * in the current sigprocmask that were already set by newsigprocmask.
       */

      altsigprocmask    = stcb->sigprocmask ^ newsigprocmask;
      stcb->sigprocmask = (stcb->sigprocmask & altsigprocmask) |
                          (savesigprocmask & ~altsigprocmask);

      /* Remove the signal structure from the sigpostedq */

      sq_rem((FAR sq_entry_t *)sigq, &(stcb->sigpostedq));
      leave_critical_section(flags);

      /* Now, handle the (rare?) case where (a) a blocked signal was
       * received while the signal handling executed but (b) restoring the
       * original sigprocmask will unblock the signal.
       */

      nxsig_unmask_pendingsignal();

      /* Then deallocate the signal structure */

      nxsig_release_pendingsigaction(sigq);
    }
}
