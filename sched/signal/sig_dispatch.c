/****************************************************************************
 * sched/signal/sig_dispatch.c
 *
 *   Copyright (C) 2007, 2009, 2011 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sched.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "sched/sched.h"
#include "group/group.h"
#include "semaphore/semaphore.h"
#include "signal/signal.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sig_queueaction
 *
 * Description:
 *   Queue a signal action for delivery to a task.
 *
 * Returned Value:
 *   Returns 0 (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int sig_queueaction(FAR struct tcb_s *stcb, siginfo_t *info)
{
  FAR sigactq_t *sigact;
  FAR sigq_t    *sigq;
  irqstate_t     saved_state;
  int            ret = OK;

  sched_lock();

  /* Find the sigaction associated with this signal */

  sigact = sig_findaction(stcb, info->si_signo);

  /* Check if a valid signal handler is available and if the signal is
   * unblocked.  NOTE:  There is no default action.
   */

  if ((sigact) && (sigact->act.sa_u._sa_sigaction))
    {
      /* Allocate a new element for the signal queue.  NOTE:
       * sig_allocatependingsigaction will force a system crash if it is
       * unable to allocate memory for the signal data */

      sigq = sig_allocatependingsigaction();
      if (!sigq)
        {
          ret = -ENOMEM;
        }
      else
        {
          /* Populate the new signal queue element */

           sigq->action.sighandler = sigact->act.sa_u._sa_sigaction;
           sigq->mask = sigact->act.sa_mask;
           memcpy(&sigq->info, info, sizeof(siginfo_t));

           /* Put it at the end of the pending signals list */

           saved_state = irqsave();
           sq_addlast((FAR sq_entry_t*)sigq, &(stcb->sigpendactionq));
           irqrestore(saved_state);
        }
    }

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: sig_allocatependingsignal
 *
 * Description:
 *   Allocate a pending signal list entry
 *
 ****************************************************************************/

static FAR sigpendq_t *sig_allocatependingsignal(void)
{
  FAR sigpendq_t *sigpend;
  irqstate_t      saved_state;

  /* Check if we were called from an interrupt handler. */

  if (up_interrupt_context())
    {
      /* Try to get the pending signal structure from the free list */

      sigpend = (FAR sigpendq_t*)sq_remfirst(&g_sigpendingsignal);
      if (!sigpend)
        {
          /* If no pending signal structure is available in the free list,
           * then try the special list of structures reserved for
           * interrupt handlers
           */

          sigpend = (FAR sigpendq_t*)sq_remfirst(&g_sigpendingirqsignal);
        }
    }

  /* If we were not called from an interrupt handler, then we are
   * free to allocate pending action structures if necessary. */

  else
    {
      /* Try to get the pending signal structure from the free list */

      saved_state = irqsave();
      sigpend = (FAR sigpendq_t*)sq_remfirst(&g_sigpendingsignal);
      irqrestore(saved_state);

      /* Check if we got one. */

      if (!sigpend)
        {
          /* No... Allocate the pending signal */

          if (!sigpend)
            {
              sigpend = (FAR sigpendq_t *)kmm_malloc((sizeof (sigpendq_t)));
            }

          /* Check if we got an allocated message */

          if (sigpend)
            {
              sigpend->type = SIG_ALLOC_DYN;
            }
        }
    }

  return sigpend;
}

/****************************************************************************
 * Name: sig_findpendingsignal
 *
 * Description:
 *   Find a specified element in the pending signal list
 *
 ****************************************************************************/

static FAR sigpendq_t *sig_findpendingsignal(FAR struct task_group_s *group,
                                             int signo)
{
  FAR sigpendq_t *sigpend = NULL;
  irqstate_t saved_state;

  DEBUGASSERT(group);

  /* Pending sigals can be added from interrupt level. */

  saved_state = irqsave();

  /* Seach the list for a sigpendion on this signal */

  for (sigpend = (FAR sigpendq_t*)group->sigpendingq.head;
       (sigpend && sigpend->info.si_signo != signo);
       sigpend = sigpend->flink);

  irqrestore(saved_state);
  return sigpend;
}

/****************************************************************************
 * Name: sig_addpendingsignal
 *
 * Description:
 *   Add the specified signal to the signal pending list. NOTE:  This
 *   function will queue only one entry for each pending signal.  This
 *   was done intentionally so that a run-away sender cannot consume
 *   all of memory.
 *
 ****************************************************************************/

static FAR sigpendq_t *sig_addpendingsignal(FAR struct tcb_s *stcb,
                                            FAR siginfo_t *info)
{
  FAR struct task_group_s *group = stcb->group;
  FAR sigpendq_t *sigpend;
  irqstate_t saved_state;

  DEBUGASSERT(group);

  /* Check if the signal is already pending */

  sigpend = sig_findpendingsignal(group, info->si_signo);
  if (sigpend)
    {
      /* The signal is already pending... retain only one copy */

      memcpy(&sigpend->info, info, sizeof(siginfo_t));
    }

  /* No... There is nothing pending for this signo */

  else
    {
      /* Allocate a new pending signal entry */

      sigpend = sig_allocatependingsignal();
      if (sigpend)
        {
          /* Put the signal information into the allocated structure */

          memcpy(&sigpend->info, info, sizeof(siginfo_t));

          /* Add the structure to the pending signal list */

          saved_state = irqsave();
          sq_addlast((FAR sq_entry_t*)sigpend, &group->sigpendingq);
          irqrestore(saved_state);
        }
    }

  return sigpend;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sig_tcbdispatch
 *
 * Description:
 *   All signals received the task (whatever the source) go through this
 *   function to be processed.  This function is responsible for:
 *
 *   - Determining if the signal is blocked.
 *   - Queuing and dispatching signal actions
 *   - Unblocking tasks that are waiting for signals
 *   - Queuing pending signals.
 *
 *   This function will deliver the signal to the task associated with
 *   the specified TCB.  This function should *not* typically be used
 *   to dispatch signals since it will *not* follow the group signal
 *   deliver algorithms.
 *
 * Returned Value:
 *   Returns 0 (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

int sig_tcbdispatch(FAR struct tcb_s *stcb, siginfo_t *info)
{
  irqstate_t saved_state;
  int ret = OK;

  sdbg("TCB=0x%08x signo=%d code=%d value=%d mask=%08x\n",
       stcb, info->si_signo, info->si_code,
       info->si_value.sival_int, stcb->sigprocmask);

  DEBUGASSERT(stcb && info);

  /************************* MASKED SIGNAL HANDLING ************************/

  /* Check if the signal is masked -- if it is, it will be added to the list
   * of pending signals.
   */

  if (sigismember(&stcb->sigprocmask, info->si_signo))
    {
      /* Check if the task is waiting for this pending signal.  If so, then unblock it.
       * This must be performed in a critical section because signals can be queued
       * from the interrupt level.
       */

      saved_state = irqsave();
      if (stcb->task_state == TSTATE_WAIT_SIG &&
          sigismember(&stcb->sigwaitmask, info->si_signo))
        {
          memcpy(&stcb->sigunbinfo, info, sizeof(siginfo_t));
          stcb->sigwaitmask = NULL_SIGNAL_SET;
          up_unblock_task(stcb);
          irqrestore(saved_state);
        }

      /* Its not one we are waiting for... Add it to the list of pending
       * signals.
       */

      else
        {
          irqrestore(saved_state);
          ASSERT(sig_addpendingsignal(stcb, info));
        }
    }

  /************************ UNMASKED SIGNAL HANDLING ***********************/

  else
    {
      /* Queue any sigaction's requested by this task. */

      ret = sig_queueaction(stcb, info);

      /* Then schedule execution of the signal handling action on the
       * recipient's thread.
       */

      up_schedule_sigaction(stcb, sig_deliver);

      /* Check if the task is waiting for an unmasked signal.  If so, then
       * unblock it. This must be performed in a critical section because
       * signals can be queued from the interrupt level.
       */

      saved_state = irqsave();
      if (stcb->task_state == TSTATE_WAIT_SIG)
        {
          memcpy(&stcb->sigunbinfo, info, sizeof(siginfo_t));
          stcb->sigwaitmask = NULL_SIGNAL_SET;
          up_unblock_task(stcb);
        }

      irqrestore(saved_state);

      /* If the task neither was waiting for the signal nor had a signal
       * handler attached to the signal, then the default action is
       * simply to ignore the signal
       */

      /*********************** OTHER SIGNAL HANDLING ***********************/

      /* If the task is blocked waiting for a semaphore, then that task must
       * be unblocked when a signal is received.
       */

      if (stcb->task_state == TSTATE_WAIT_SEM)
        {
          sem_waitirq(stcb, EINTR);
        }

      /* If the task is blocked waiting on a message queue, then that task
       * must be unblocked when a signal is received.
       */

#ifndef CONFIG_DISABLE_MQUEUE
      if (stcb->task_state == TSTATE_WAIT_MQNOTEMPTY ||
          stcb->task_state == TSTATE_WAIT_MQNOTFULL)
        {
          mq_waitirq(stcb, EINTR);
        }
#endif
    }

  return ret;
}

/****************************************************************************
 * Name: sig_dispatch
 *
 * Description:
 *   This is the front-end for sig_tcbdispatch that should be typically
 *   be used to dispatch a signal.  If HAVE_GROUP_MEMBERS is defined,
 *   then function will follow the group signal delivery algorthrims:
 *
 *   This front-end does the following things before calling
 *   sig_tcbdispatch.
 *
 *     With HAVE_GROUP_MEMBERS defined:
 *     - Get the TCB associated with the pid.
 *     - If the TCB was found, get the group from the TCB.
 *     - If the PID has already exited, lookup the group that that was
 *       started by this task.
 *     - Use the group to pick the TCB to receive the signal
 *     - Call sig_tcbdispatch with the TCB
 *
 *     With HAVE_GROUP_MEMBERS *not* defined
 *     - Get the TCB associated with the pid.
 *     - Call sig_tcbdispatch with the TCB
 *
 * Returned Value:
 *   Returns 0 (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

int sig_dispatch(pid_t pid, FAR siginfo_t *info)
{
#ifdef HAVE_GROUP_MEMBERS
  FAR struct tcb_s *stcb;
  FAR struct task_group_s *group;

  /* Get the TCB associated with the pid */

  stcb = sched_gettcb(pid);
  if (stcb)
    {
      /* The task/thread associated with this PID is still active.  Get its
       * task group.
       */

      group = stcb->group;
    }
  else
    {
      /* The task/thread associated with this PID has exited.  In the normal
       * usage model, the PID should correspond to the PID of the task that
       * created the task group.  Try looking it up.
       */

      group = group_findbypid(pid);
    }

  /* Did we locate the group? */

  if (group)
    {
      /* Yes.. call group_signal() to send the signal to the correct group
       * member.
       */

      return group_signal(group, info);
    }
  else
    {
      return -ESRCH;
    }

#else
  FAR struct tcb_s *stcb;

  /* Get the TCB associated with the pid */

  stcb = sched_gettcb(pid);
  if (!stcb)
    {
      return -ESRCH;
    }

  return sig_tcbdispatch(stcb, info);

#endif
}
