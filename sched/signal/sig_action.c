/****************************************************************************
 * sched/signal/sig_action.c
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

#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include <queue.h>
#include <sched.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "sched/sched.h"
#include "group/group.h"
#include "signal/signal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_sigaction_spin;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_alloc_actionblock
 *
 * Description:
 *   Allocate a block of signal actions and place them
 *   on the free list.
 *
 ****************************************************************************/

static void nxsig_alloc_actionblock(void)
{
  FAR sigactq_t *sigact;
  irqstate_t flags;
  int i;

  /* Allocate a block of signal actions */

  sigact = kmm_malloc((sizeof(sigactq_t)) * NUM_SIGNAL_ACTIONS);
  if (sigact != NULL)
    {
      flags = spin_lock_irqsave(&g_sigaction_spin);

      for (i = 0; i < NUM_SIGNAL_ACTIONS; i++)
        {
          sq_addlast((FAR sq_entry_t *)sigact++, &g_sigfreeaction);
        }

      spin_unlock_irqrestore(&g_sigaction_spin, flags);
    }
}

/****************************************************************************
 * Name: nxsig_alloc_action
 *
 * Description:
 *   Allocate a new element for a sigaction queue
 *
 ****************************************************************************/

static FAR sigactq_t *nxsig_alloc_action(void)
{
  FAR sigactq_t *sigact;
  irqstate_t flags;

  /* Try to get the signal action structure from the free list */

  flags = spin_lock_irqsave(&g_sigaction_spin);
  sigact = (FAR sigactq_t *)sq_remfirst(&g_sigfreeaction);
  spin_unlock_irqrestore(&g_sigaction_spin, flags);

  /* Check if we got one. */

  if (!sigact)
    {
      /* Add another block of signal actions to the list */

      nxsig_alloc_actionblock();

      /* And try again */

      flags = spin_lock_irqsave(&g_sigaction_spin);
      sigact = (FAR sigactq_t *)sq_remfirst(&g_sigfreeaction);
      spin_unlock_irqrestore(&g_sigaction_spin, flags);
      DEBUGASSERT(sigact);
    }

  return sigact;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_action and sigaction
 *
 * Description:
 *   This function allows the calling process to examine and/or specify the
 *   action to be associated with a specific signal.
 *
 *   The structure sigaction, used to describe an action to be taken, is
 *   defined to include the following members:
 *
 *   - sa_u.sa_handler:  Pointer to a signal-catching function
 *   - sa_u.sa_sigaction:  Alternative form of the signal-catching function
 *   - sa_mask: An additional set of signals to be blocked during execution
 *       of a signal catching function
 *   - sa_flags.  Special flags to affect the behavior of a signal.
 *
 *   If the argument 'act' is not NULL, it points to a structure specifying
 *   the action to be associated with the specified signal.  If the argument
 *   'oact' is not NULL, the action previously associated with the signal
 *   is stored in the location pointed to by the argument 'oact.'
 *
 *   When a signal is caught by a signal-catching function installed by
 *   sigaction() function, a new signal mask is calculated and installed for
 *   the duration of the signal-catching function.  This mask is formed by
 *   taking the union of the current signal mask and the value of the
 *   sa_mask for the signal being delivered and then including the signal
 *   being delivered.  If and when the user's signal handler returns, the
 *   original signal mask is restored.
 *
 *   Once an action is installed for a specific signal, it remains installed
 *   until another action is explicitly requested by another call to
 *   sigaction().
 *
 *   nxsig_action() is an internal version of sigaction that adds an
 *   additional parameter, force, that is used to set default signal actions
 *   (which may not normally be settable).  nxsig_action() does not alter the
 *   errno variable.
 *
 * Input Parameters:
 *   sig   - Signal of interest
 *   act   - Location of new handler
 *   oact  - Location to store only handler
 *   force - Force setup of the signal handler, even if it cannot normally
 *           be caught or ignored (nxsig_action only)
 *
 * Returned Value:
 *   nxsig_action:
 *     Zero (OK) is returned on success; a negated errno value is returned
 *     on failure
 *   sigaction:
 *     Zero (OK) is returned on success; -1 (ERROR) is returned on any
 *     failure if the signal number is invalid with the errno set
 *     appropriately
 *
 * Assumptions:
 *
 * POSIX Compatibility:
 * - If CONFIG_SIG_DEFAULT is not defined, then there are no default actions
 *   so the special value SIG_DFL is treated like SIG_IGN.
 * - All sa_flags in struct sigaction of act input are ignored (all
 *   treated like SA_SIGINFO). The one exception is if
 *   CONFIG_SCHED_CHILD_STATUS is defined; then SA_NOCLDWAIT is supported but
 *   only for SIGCHLD
 *
 ****************************************************************************/

int nxsig_action(int signo, FAR const struct sigaction *act,
                 FAR struct sigaction *oact, bool force)
{
  FAR struct tcb_s *rtcb = this_task();
  FAR struct task_group_s *group;
  FAR sigactq_t *sigact;
  _sa_handler_t handler;

  /* Since sigactions can only be installed from the running thread of
   * execution, no special precautions should be necessary.
   */

  DEBUGASSERT(rtcb != NULL && rtcb->group != NULL);
  group = rtcb->group;

  /* Verify the signal number */

  if (!GOOD_SIGNO(signo))
    {
      return -EINVAL;
    }

#ifdef CONFIG_SIG_DEFAULT
  /* Check if the user is trying to catch or ignore a signal that cannot be
   * caught or ignored.
   */

  if (act != NULL && !force && act->sa_handler != SIG_DFL &&
      !nxsig_iscatchable(signo))
    {
      return -EINVAL;
    }
#endif

  /* Find the signal in the signal action queue */

  sigact = nxsig_find_action(group, signo);

  /* Return the old sigaction value if so requested */

  if (oact != NULL)
    {
#ifdef CONFIG_SIG_DEFAULT
      if (nxsig_isdefault(rtcb, signo))
        {
          /* Return SIG_DFL if the default signal is attached */

          oact->sa_handler = SIG_DFL;
          oact->sa_mask    = NULL_SIGNAL_SET;
          oact->sa_flags   = SA_SIGINFO;
        }
      else
#endif
      if (sigact)
        {
          /* Return the old signal action */

          oact->sa_handler = sigact->act.sa_handler;
          oact->sa_mask    = sigact->act.sa_mask;
          oact->sa_flags   = sigact->act.sa_flags;
        }
      else
        {
          /* There isn't an old value */

          oact->sa_handler = NULL;
          oact->sa_mask    = NULL_SIGNAL_SET;
          oact->sa_flags   = 0;
        }
    }

  /* If the argument act is a null pointer, signal handling is unchanged;
   * thus, the call can be used to inquire about the current handling of
   * a given signal.
   */

  if (act == NULL)
    {
      return OK;
    }

#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
  /* Handle a special case.  Retention of child status can be suppressed
   * if signo == SIGCHLD and sa_flags == SA_NOCLDWAIT.
   *
   * POSIX.1 leaves it unspecified whether a SIGCHLD signal is generated
   * when a child process terminates.  In NuttX, a SIGCHLD signal is
   * generated in this case; but in some other implementations, it may not
   * be.
   */

  if (signo == SIGCHLD && (act->sa_flags & SA_NOCLDWAIT) != 0)
    {
      irqstate_t flags;

      /* We do require a critical section to muck with the TCB values that
       * can be modified by the child thread.
       */

      flags = enter_critical_section();

      /* Mark that status should be not be retained */

      rtcb->group->tg_flags |= GROUP_FLAG_NOCLDWAIT;

      /* Free all pending exit status */

      group_remove_children(rtcb->group);
      leave_critical_section(flags);
    }
#endif

  handler = act->sa_handler;

#ifdef CONFIG_SIG_DEFAULT
  /* If the caller is setting the handler to SIG_DFL, then we need to
   * replace this with the correct, internal default signal action handler.
   */

  if (handler == SIG_DFL)
    {
      /* nxsig_default() may returned SIG_IGN */

      handler = nxsig_default(rtcb, signo, true);
    }
  else
    {
      /* We will be replacing the default action (or ignoring it) */

      nxsig_default(rtcb, signo, false);
    }
#endif

  /* Handle the case where no sigaction is supplied (SIG_IGN) */

  if (handler == SIG_IGN)
    {
      /* Do we still have a sigaction container from the previous setting? */

      if (sigact)
        {
          /* Yes.. Remove it from signal action queue */

          sq_rem((FAR sq_entry_t *)sigact, &group->tg_sigactionq);

          /* And deallocate it */

          nxsig_release_action(sigact);
        }
    }

  /* A sigaction has been supplied */

  else
    {
      /* Do we still have a sigaction container from the previous setting?
       * If so, then re-use for the new signal action.
       */

      if (sigact == NULL)
        {
          /* No.. Then we need to allocate one for the new action. */

          sigact = nxsig_alloc_action();

          /* An error has occurred if we could not allocate the sigaction */

          if (!sigact)
            {
              return -ENOMEM;
            }

          /* Put the signal number in the queue entry */

          sigact->signo = (uint8_t)signo;

          /* Add the new sigaction to signal action queue */

          sq_addlast((FAR sq_entry_t *)sigact, &group->tg_sigactionq);
        }

      /* Set the new sigaction */

      sigact->act.sa_handler = handler;
      sigact->act.sa_mask    = act->sa_mask;
      sigact->act.sa_flags   = act->sa_flags;
    }

  return OK;
}

int sigaction(int signo, FAR const struct sigaction *act,
              FAR struct sigaction *oact)
{
  int ret;

  /* nxsig_action() does all of the work */

  ret = nxsig_action(signo, act, oact, false);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: nxsig_release_action
 *
 * Description:
 *   Deallocate a sigaction Q entry
 *
 ****************************************************************************/

void nxsig_release_action(FAR sigactq_t *sigact)
{
  irqstate_t flags;

  /* Just put it back on the free list */

  flags = spin_lock_irqsave(&g_sigaction_spin);
  sq_addlast((FAR sq_entry_t *)sigact, &g_sigfreeaction);
  spin_unlock_irqrestore(&g_sigaction_spin, flags);
}
