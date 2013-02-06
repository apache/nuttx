/*****************************************************************************
 * sched/group_signal.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <sched.h>
#include <assert.h>
#include <signal.h>
#include <errno.h>
#include <debug.h>

#include "os_internal.h"
#include "group_internal.h"
#include "sig_internal.h"

#if defined(HAVE_TASK_GROUP) && !defined(CONFIG_DISABLE_SIGNALS)

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/*****************************************************************************
 * Private Types
 *****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
struct group_signal_s
{
  FAR siginfo_t *siginfo; /* Signal to be dispatched */
  FAR struct tcb_s *dtcb; /* Default, valid TCB */
  FAR struct tcb_s *utcb; /* TCB with this signal unblocked */
  FAR struct tcb_s *atcb; /* This TCB was awakened */
  FAR struct tcb_s *ptcb; /* This TCB received the signal */
};
#endif

/*****************************************************************************
 * Private Data
 *****************************************************************************/

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: group_signal_handler
 *
 * Description:
 *   Callback from group_foreachchild that handles one member of the group.
 *
 * Parameters:
 *   pid - The ID of the group member that may be signalled.
 *   arg - A pointer to a struct group_signal_s instance.
 *
 * Return Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 *****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
static int group_signal_handler(pid_t pid, FAR void *arg)
{
  FAR struct group_signal_s *info = (FAR struct group_signal_s *)arg;
  FAR struct tcb_s *tcb;
  FAR sigactq_t *sigact;
  int ret;

  DEBUGASSERT(info);

  /* Get the TCB associated with the group member */

  tcb = sched_gettcb(pid);
  DEBUGASSERT(tcb);
  if (tcb)
    {
      /* Set this one as the default if we have not already set the default. */

      if (!info->dtcb)
        {
          info->dtcb = tcb;
        }

      /* Is the thread waiting for this signal (in this case, the signal is
       * probably blocked).
       */

      if (sigismember(&tcb->sigwaitmask, info->siginfo->si_signo) && !info->atcb)
        {
          /* Yes.. This means that the task is suspended, waiting for this
           * signal to occur. Stop looking and use this TCB.  The
           * requirement is this:  If a task group receives a signal and
           * more than one thread is waiting on that signal, then one and
           * only one indeterminate thread out of that waiting group will
           * receive the signal.
           */

          ret = sig_tcbdispatch(tcb, info->siginfo);
          if (ret < 0)
            {
              return ret;
            }

          /* Limit to one thread */

          info->atcb = tcb;
          if (info->ptcb != NULL);
            {
              return 1; /* Terminate the search */
            }
        }

      /* Is this signal unblocked on this thread? */

      if (!sigismember(&tcb->sigprocmask, info->siginfo->si_signo) &&
          !info->ptcb && tcb != info->atcb)
        {
          /* Yes.. remember this TCB if we have not encountered any
           * other threads that have the signal unblocked.
           */

          if (!info->utcb)
            {
              info->utcb = tcb;
            }

          /* Is there also an action associated with the task? */

          sigact = sig_findaction(tcb, info->siginfo->si_signo);
          if (sigact)
            {
              /* Yes.. then use this thread.  The requirement is this:
               * If a task group receives a signal then one and only one
               * indeterminate thread in the task group which is not
               * blocking the signal will receive the signal.
               */

              ret = sig_tcbdispatch(tcb, info->siginfo);
              if (ret < 0)
                {
                  return ret;
                }

              /* Limit to one thread */

              info->ptcb = tcb;
              if (info->atcb != NULL)
                {
                  return 1; /* Terminate the search */
                }
            }
        }
    }

  return 0; /* Keep searching */
}
#endif

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/*****************************************************************************
 * Name: group_signal
 *
 * Description:
 *   Send a signal to every member of the group.
 *
 * Parameters:
 *   group - The task group that needs to be signalled.
 *
 * Return Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called during task termination in a safe context.  No special precautions
 *   are required here.  Because signals can be sent from interrupt handlers,
 *   this function may be called indirectly in the context of an interrupt
 *   handler.
 *
 *****************************************************************************/

int group_signal(FAR struct task_group_s *group, FAR siginfo_t *siginfo)
{
#ifdef HAVE_GROUP_MEMBERS
  struct group_signal_s info;
  FAR struct tcb_s *tcb;
  int ret;

  DEBUGASSERT(group && siginfo);

  info.siginfo = siginfo;
  info.dtcb    = NULL;     /* Default, valid TCB */
  info.utcb    = NULL;     /* TCB with this signal unblocked */
  info.atcb    = NULL;     /* This TCB was awakened */
  info.ptcb    = NULL;     /* This TCB received the signal */

  /* Make sure that pre-emption is disabled to that we signal all of the
   * members of the group before any of them actually run. (This does
   * nothing if were were called from an interrupt handler).
   */

  sched_lock();

  /* Now visit each member of the group and perform signal handling checks. */

  ret = group_foreachchild(group, group_signal_handler, &info);
  if (ret < 0)
    {
      goto errout;
    }

  /* We need to dispatch the signal in any event (if nothing else so that it
   * can be added to the pending signal list). If we found a thread with the
   * signal unblocked, then use that thread.
   */

  if (info.atcb == NULL && info.ptcb == NULL)
    {
      if (info.utcb)
        {
          tcb = info.utcb;
        }

      /* Otherwise use the default TCB.  There should always be a default
       * TCB. It will have the signal blocked, but can be used to get the
       * signal to a pending state.
       */

      else /* if (info.dtcb) */
        {
          DEBUGASSERT(info.dtcb);
          tcb = info.dtcb;
        }

      /* Now deliver the signal to the selected group member */

      ret = sig_tcbdispatch(tcb, siginfo);
    }

errout:
  sched_unlock();
  return ret;

#else

  return -ENOSYS;
  
#endif
}

#endif /* HAVE_TASK_GROUP && !CONFIG_DISABLE_SIGNALS */
