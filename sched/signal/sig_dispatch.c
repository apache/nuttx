/****************************************************************************
 * sched/signal/sig_dispatch.c
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

#include <inttypes.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/signal.h>

#include "sched/sched.h"
#include "group/group.h"
#include "semaphore/semaphore.h"
#include "signal/signal.h"
#include "mqueue/mqueue.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_queue_action
 *
 * Description:
 *   Queue a signal action for delivery to a task.
 *
 * Returned Value:
 *   Returns 0 (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

static int nxsig_queue_action(FAR struct tcb_s *stcb, siginfo_t *info)
{
  FAR sigactq_t *sigact;
  FAR sigq_t    *sigq;
  irqstate_t     flags;
  int            ret = OK;

  sched_lock();
  DEBUGASSERT(stcb != NULL && stcb->group != NULL);

  /* Find the group sigaction associated with this signal */

  sigact = nxsig_find_action(stcb->group, info->si_signo);

  /* Check if a valid signal handler is available and if the signal is
   * unblocked. NOTE: There is no default action.
   */

  if ((sigact) && (sigact->act.sa_u._sa_sigaction))
    {
      /* Allocate a new element for the signal queue. NOTE:
       * nxsig_alloc_pendingsigaction will force a system crash if it is
       * unable to allocate memory for the signal data.
       */

      sigq = nxsig_alloc_pendingsigaction();
      if (!sigq)
        {
          ret = -ENOMEM;
        }
      else
        {
          /* Populate the new signal queue element */

          sigq->action.sighandler = sigact->act.sa_u._sa_sigaction;
          sigq->mask = sigact->act.sa_mask;
          if ((sigact->act.sa_flags & SA_NODEFER) == 0)
            {
              sigq->mask |= SIGNO2SET(info->si_signo);
            }

          memcpy(&sigq->info, info, sizeof(siginfo_t));

          /* Put it at the end of the pending signals list */

          flags = enter_critical_section();
          sq_addlast((FAR sq_entry_t *)sigq, &(stcb->sigpendactionq));

          /* Then schedule execution of the signal handling action on the
           * recipient's thread. SMP related handling will be done in
           * up_schedule_sigaction()
           */

          up_schedule_sigaction(stcb, nxsig_deliver);
          leave_critical_section(flags);
        }
    }

  sched_unlock();
  return ret;
}

/****************************************************************************
 * Name: nxsig_alloc_pendingsignal
 *
 * Description:
 *   Allocate a pending signal list entry
 *
 ****************************************************************************/

static FAR sigpendq_t *nxsig_alloc_pendingsignal(void)
{
  FAR sigpendq_t *sigpend;
  irqstate_t      flags;

  /* Check if we were called from an interrupt handler. */

  if (up_interrupt_context())
    {
      /* Try to get the pending signal structure from the free list */

      sigpend = (FAR sigpendq_t *)sq_remfirst(&g_sigpendingsignal);
      if (!sigpend)
        {
          /* If no pending signal structure is available in the free list,
           * then try the special list of structures reserved for
           * interrupt handlers
           */

          sigpend = (FAR sigpendq_t *)sq_remfirst(&g_sigpendingirqsignal);
        }
    }

  /* If we were not called from an interrupt handler, then we are
   * free to allocate pending action structures if necessary.
   */

  else
    {
      /* Try to get the pending signal structure from the free list */

      flags = enter_critical_section();
      sigpend = (FAR sigpendq_t *)sq_remfirst(&g_sigpendingsignal);
      leave_critical_section(flags);

      /* Check if we got one. */

      if (!sigpend)
        {
          /* No... Allocate the pending signal */

          sigpend = (FAR sigpendq_t *)kmm_malloc((sizeof (sigpendq_t)));

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
 * Name: nxsig_find_pendingsignal
 *
 * Description:
 *   Find a specified element in the pending signal list
 *
 ****************************************************************************/

static FAR sigpendq_t *
  nxsig_find_pendingsignal(FAR struct task_group_s *group, int signo)
{
  FAR sigpendq_t *sigpend = NULL;
  irqstate_t flags;

  DEBUGASSERT(group != NULL);

  /* Pending signals can be added from interrupt level. */

  flags = enter_critical_section();

  /* Search the list for a action pending on this signal */

  for (sigpend = (FAR sigpendq_t *)group->tg_sigpendingq.head;
       (sigpend && sigpend->info.si_signo != signo);
       sigpend = sigpend->flink);

  leave_critical_section(flags);
  return sigpend;
}

/****************************************************************************
 * Name: nxsig_add_pendingsignal
 *
 * Description:
 *   Add the specified signal to the signal pending list. NOTE: This
 *   function will queue only one entry for each pending signal. This
 *   was done intentionally so that a run-away sender cannot consume
 *   all of memory.
 *
 ****************************************************************************/

static void nxsig_add_pendingsignal(FAR struct tcb_s *stcb,
                                    FAR siginfo_t *info)
{
  FAR struct task_group_s *group;
  FAR sigpendq_t *sigpend;
  irqstate_t flags;

  DEBUGASSERT(stcb != NULL && stcb->group != NULL);
  group = stcb->group;

  /* Check if the signal is already pending for the group */

  sigpend = nxsig_find_pendingsignal(group, info->si_signo);
  if (sigpend != NULL)
    {
      /* The signal is already pending... retain only one copy */

      memcpy(&sigpend->info, info, sizeof(siginfo_t));
    }

  /* No... There is nothing pending in the group for this signo */

  else
    {
      /* Allocate a new pending signal entry */

      sigpend = nxsig_alloc_pendingsignal();
      if (sigpend != NULL)
        {
          /* Put the signal information into the allocated structure */

          memcpy(&sigpend->info, info, sizeof(siginfo_t));

          /* Add the structure to the group pending signal list */

          flags = enter_critical_section();
          sq_addlast((FAR sq_entry_t *)sigpend, &group->tg_sigpendingq);
          leave_critical_section(flags);
        }
    }

  DEBUGASSERT(sigpend);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_tcbdispatch
 *
 * Description:
 *   All signals received the task (whatever the source) go through this
 *   function to be processed. This function is responsible for:
 *
 *   - Determining if the signal is blocked.
 *   - Queuing and dispatching signal actions
 *   - Unblocking tasks that are waiting for signals
 *   - Queuing pending signals.
 *
 *   This function will deliver the signal to the task associated with
 *   the specified TCB. This function should *not* typically be used
 *   to dispatch signals since it will *not* follow the group signal
 *   deliver algorithms.
 *
 * Returned Value:
 *   Returns 0 (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

int nxsig_tcbdispatch(FAR struct tcb_s *stcb, siginfo_t *info)
{
  irqstate_t flags;
  int masked;
  int ret = OK;

  sinfo("TCB=%p signo=%d code=%d value=%d mask=%08" PRIx32 "\n",
        stcb, info->si_signo, info->si_code,
        info->si_value.sival_int, stcb->sigprocmask);

  DEBUGASSERT(stcb != NULL && info != NULL);

  /* Don't actually send a signal for signo 0. */

  if (info->si_signo == 0)
    {
      return OK;
    }

  /************************** MASKED SIGNAL ACTIONS *************************/

  masked = nxsig_ismember(&stcb->sigprocmask, info->si_signo);

#ifdef CONFIG_LIB_SYSCALL
  /* Check if the signal is masked OR if the signal is received while we are
   * processing a system call -- in either case, it will be added to the
   * list of pending signals. Unmasked user signal actions will be deferred
   * while we process the system call.
   *
   * If a thread calls a blocking system call, the thread will still be
   * unblocked when the signal occurs (see OTHER SIGNAL HANDLING below), but
   * any associated user signal action will be deferred until the system
   * call returns. For example, if the application calls sem_wait(), the
   * following would occur:
   *
   *   1. System call entry logic will block user signal handling and call
   *      sem_wait() in kernel mode.
   *   2. sem_wait() will block,
   *   3. The receipt of the signal will cause any signal action to pend
   *      but will unblock sem_wait(),
   *   4. The sem_wait() system call will awaken and return EINTR,
   *   5. The pending signal action will occur after the sem_wait() system
   *      call returns to user mode.
   *
   * Syscall handlers (and logic-in-general within the OS) should not use
   * signal handlers.
   */

  if ((masked == 1) || (stcb->flags & TCB_FLAG_SYSCALL) != 0)
#else
  /* Check if the signal is masked. In that case, it will be added to the
   * list of pending signals.
   */

  if (masked == 1)
#endif
    {
      /* Check if the task is waiting for this pending signal. If so, then
       * unblock it. This must be performed in a critical section because
       * signals can be queued from the interrupt level.
       */

      flags = enter_critical_section();
      if (stcb->task_state == TSTATE_WAIT_SIG &&
          (masked == 0 ||
           nxsig_ismember(&stcb->sigwaitmask, info->si_signo)))
        {
          memcpy(&stcb->sigunbinfo, info, sizeof(siginfo_t));
          stcb->sigwaitmask = NULL_SIGNAL_SET;
          up_unblock_task(stcb);
          leave_critical_section(flags);
        }

      /* Its not one we are waiting for... Add it to the list of pending
       * signals.
       */

      else
        {
          leave_critical_section(flags);
          nxsig_add_pendingsignal(stcb, info);
        }
    }

  /************************* UNMASKED SIGNAL ACTIONS ************************/

  else
    {
      /* Queue any sigaction's requested by this task. */

      ret = nxsig_queue_action(stcb, info);

      /* Deliver of the signal must be performed in a critical section */

      flags = enter_critical_section();

      /* Check if the task is waiting for an unmasked signal. If so, then
       * unblock it. This must be performed in a critical section because
       * signals can be queued from the interrupt level.
       */

      if (stcb->task_state == TSTATE_WAIT_SIG)
        {
          memcpy(&stcb->sigunbinfo, info, sizeof(siginfo_t));
          stcb->sigwaitmask = NULL_SIGNAL_SET;
          up_unblock_task(stcb);
        }

      leave_critical_section(flags);

      /* If the task neither was waiting for the signal nor had a signal
       * handler attached to the signal, then the default action is
       * simply to ignore the signal
       */
    }

  /************************* OTHER SIGNAL HANDLING **************************/

  /* Performed only if the signal is unmasked. These actions also must
   * happen within a system call.
   */

  if (masked == 0)
    {
      /* If the task is blocked waiting for a semaphore, then that task must
       * be unblocked when a signal is received.
       */

      if (stcb->task_state == TSTATE_WAIT_SEM)
        {
          nxsem_wait_irq(stcb, EINTR);
        }

#ifndef CONFIG_DISABLE_MQUEUE
      /* If the task is blocked waiting on a message queue, then that task
       * must be unblocked when a signal is received.
       */

      if (stcb->task_state == TSTATE_WAIT_MQNOTEMPTY ||
          stcb->task_state == TSTATE_WAIT_MQNOTFULL)
        {
          nxmq_wait_irq(stcb, EINTR);
        }
#endif

#ifdef CONFIG_SIG_SIGSTOP_ACTION
      /* If the task was stopped by SIGSTOP or SIGTSTP, then unblock the task
       * if SIGCONT is received.
       */

      if (stcb->task_state == TSTATE_TASK_STOPPED &&
          info->si_signo == SIGCONT)
        {
#ifdef HAVE_GROUP_MEMBERS
          group_continue(stcb);
#else
          nxsched_continue(stcb);
#endif
        }
#endif
    }

  /* In case nxsig_ismember failed due to an invalid signal number */

  if (masked < 0)
    {
      ret = -EINVAL;
    }

  return ret;
}

/****************************************************************************
 * Name: nxsig_dispatch
 *
 * Description:
 *   This is the front-end for nxsig_tcbdispatch that should be typically
 *   be used to dispatch a signal. If HAVE_GROUP_MEMBERS is defined,
 *   then function will follow the group signal delivery algorithms:
 *
 *   This front-end does the following things before calling
 *   nxsig_tcbdispatch.
 *
 *     With HAVE_GROUP_MEMBERS defined:
 *     - Get the TCB associated with the pid.
 *     - If the TCB was found, get the group from the TCB.
 *     - If the PID has already exited, lookup the group that that was
 *       started by this task.
 *     - Use the group to pick the TCB to receive the signal
 *     - Call nxsig_tcbdispatch with the TCB
 *
 *     With HAVE_GROUP_MEMBERS *not* defined
 *     - Get the TCB associated with the pid.
 *     - Call nxsig_tcbdispatch with the TCB
 *
 * Returned Value:
 *   Returns 0 (OK) on success or a negated errno value on failure.
 *
 ****************************************************************************/

int nxsig_dispatch(pid_t pid, FAR siginfo_t *info)
{
#ifdef HAVE_GROUP_MEMBERS
  FAR struct tcb_s *stcb;
  FAR struct task_group_s *group;

  /* Get the TCB associated with the pid */

  stcb = nxsched_get_tcb(pid);
  if (stcb != NULL)
    {
      /* The task/thread associated with this PID is still active. Get its
       * task group.
       */

      group = stcb->group;
    }
  else
    {
      /* The task/thread associated with this PID has exited. In the normal
       * usage model, the PID should correspond to the PID of the task that
       * created the task group. Try looking it up.
       */

      group = group_findbypid(pid);
    }

  /* Did we locate the group? */

  if (group != NULL)
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

  stcb = nxsched_get_tcb(pid);
  if (stcb == NULL)
    {
      return -ESRCH;
    }

  return nxsig_tcbdispatch(stcb, info);

#endif
}
