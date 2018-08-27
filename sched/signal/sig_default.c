/****************************************************************************
 * sched/signal/sig_default.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <assert.h>

#include <nuttx/sched.h>

#include "signal/signal.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This table is used to make the default action for a signal to the correct
 * signal handler.
 */

#if 0 /* Not used */
static const struct _sa_sigaction_t g_defactions[MAX_SIGNO + 1] =
{
  /* To be provided */
}
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* TODO:  Take into account pthread-specific signal behaviors */

/* Default Actions:
 *
 * - Abnormal termination of the process. The process is terminated with
 *   all the consequences of _exit() except that the status made available
 *   to wait() and waitpid() indicates abnormal termination by the
 *   specified signal.
 * - Abnormal termination of the process. Additionally with the XSI
 *   extension, implementation-defined abnormal termination actions, such
 *   as creation of a core file, may occur.
 * - Ignore the signal.
 * - Stop the process.
 * - Continue the process, if it is stopped; otherwise, ignore the signal.
 */

/****************************************************************************
 * Name: nxsig_abnormal_termination
 *
 * Description:
 *   This is the abnormal termination default action for the SIGKILL signal.
 *
 * Input Parameters:
 *   Standard signal handler parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxsig_abnormal_termination(int signo, FAR siginfo_t *siginfo,
                                       FAR void *arg)
{
#ifdef HAVE_GROUP_MEMBERS
  FAR struct tcb_s *tcb = (FAR struct tcb_s *)this_task();

  /* Kill of of the children of the task */

  group_killchildren(tcb);
#endif

  /* And exit to terminate the task */

  exit(EXIT_FAILURE);
}

/****************************************************************************
 * Name: nxsig_default_action
 *
 * Description:
 *   Look up the default action associated with this signal
 *
 * Input Parameters:
 *   signo - The signal number to use in the query
 *
 * Returned Value:
 *   The default handler associated with this signal
 *
 ****************************************************************************/

static _sa_sigaction_t nxsig_default_action(int signo)
{
#if 0 /* Not implemented */
  return g_defactions[signo];
#else
  /* Currently only SIGKILL and the abnormal exit signal action are supported */

  return signo == SIGKILL ? nxsig_abnormal_termination : NULL;
#endif
}

/****************************************************************************
 * Name: nxsig_setup_default_action
 *
 * Description:
 *   Setup the default action for the SIGKILL signal
 *
 * Input Parameters:
 *   group  - The group that the task belongs in.
 *   action - The new default signal action
 *   signo  - The signal that will produce this default action
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void nxsig_setup_default_action(FAR struct task_group_s *group,
                                       int signo)
{
  struct sigaction sa;
  _sa_sigaction_t action;

  DEBUGASSERT(group != NULL && GOOD_SIGNO(signo));

  /* Get the address of the handler for this signals default action. */

  action = nxsig_default_action(signo);
  if (action != (_sa_sigaction_t)SIG_IGN)
    {
      /* Attach the signal handler */

      memset(&sa, 0, sizeof(sa));
      sa.sa_sigaction = action;
      sa.sa_flags     = SA_SIGINFO;
      (void)sigaction(SIGKILL, &sa, NULL);

      /* Indicate that the default signal handler has been attached */

      (void)sigaddset(&group->tg_sigdefault, signo);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsig_isdefault
 *
 * Description:
 *   Return true if the specified signal is set to the default action.
 *
 * Input Parameters:
 *   tcb   - Identifies the thread associated with the default handler
 *   signo - The signal number to be queried
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

bool nxsig_isdefault(FAR struct tcb_s *tcb, int signo)
{
  FAR struct task_group_s *group;
  int ret;

  DEBUGASSERT(tcb != NULL && tcb->group != NULL && GOOD_SIGNO(signo));
  group = tcb->group;

  /* Return true if the signo is marked as using the default action.  Return
   * false in all other cases.
   */

  ret = sigismember(&group->tg_sigdefault, signo);
  return ret < 0 ? false : (bool)ret;
}

/****************************************************************************
 * Name: nxsig_default
 *
 * Description:
 *   If 'defaction' is true, then return the default signal handler action
 *   for the specified signal and mark that the default signal hander is
 *   in place (it is not yet).
 *
 *   If 'defaction' is false, then mark that the default signal handler is
 *   NOT in place and return SIG_IGN.
 *
 *   This function is called form sigaction() to handle actions = SIG_DFL.
 *
 * Input Parameters:
 *   tcb       - Identifies the thread associated with the default handler
 *   signo     - The signal number whose behavior will be modified.
 *   defaction - True: the default action is in place
 *
 * Returned Value:
 *   The address of the default signal action handler is returne on success.
 *   SIG_IGN is returned if there is no default action.
 *
 ****************************************************************************/

 _sa_sigaction_t nxsig_default(FAR struct tcb_s *tcb, int signo,
                               bool defaction)
{
  FAR struct task_group_s *group;
  _sa_sigaction_t handler = (_sa_sigaction_t)SIG_IGN;

  DEBUGASSERT(tcb != NULL && tcb->group != NULL);
  group = tcb->group;

  /* Are we setting or unsetting the default action? */

  if (defaction)
    {
      /* We are setting the default action.  Look up the default action
       * associated with signo.
       */

      handler = nxsig_default_action(signo);
      if (handler != (_sa_sigaction_t)SIG_IGN)
        {
          (void)sigaddset(&group->tg_sigdefault, signo);
        }
    }

  if (handler == (_sa_sigaction_t)SIG_IGN)
    {
      /* We are unsetting the default action */

      (void)sigdelset(&group->tg_sigdefault, signo);
    }

  return handler;
}

/****************************************************************************
 * Name: nxsig_default_initialize
 *
 * Description:
 *   Set all signals to their default action.  This is called from task_start
 *   to configure the newly started task.
 *
 * Input Parameters:
 *   tcb - Identifies the thread associated with the default handlers
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int nxsig_default_initialize(FAR struct tcb_s *tcb)
{
  FAR struct task_group_s *group;

  DEBUGASSERT(tcb != NULL && tcb->group != NULL);
  group = tcb->group;

  /* Initialize the set of default signal handlers */

  (void)sigemptyset(&group->tg_sigdefault);

#if 0
  /* TODO Currently only SIGKILL is supported.  The following needs to be
   * in a loop that instantiates the default action for all signals (other
   * that those that are ignored).
   */

  for (signo = 0; signo <= MAX_SIGNO; signo++)
    {
      nxsig_setup_default_action(group, signo);
    }

#else
   /* For now */

  nxsig_setup_default_action(group, SIGKILL);
#endif

  return OK;
}
