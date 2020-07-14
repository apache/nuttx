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

#include <sys/wait.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/irq.h>
#include <nuttx/signal.h>

#include "group/group.h"
#include "sched/sched.h"
#include "task/task.h"
#include "signal/signal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_SIG_SIGUSR1_ACTION) || defined(CONFIG_SIG_SIGUSR2_ACTION) || \
    defined(CONFIG_SIG_SIGALRM_ACTION) || defined(CONFIG_SIG_SIGPOLL_ACTION) || \
    defined(CONFIG_SIG_SIGKILL_ACTION) || defined(CONFIG_SIG_SIGPIPE_ACTION)
#  define HAVE_NXSIG_ABNORMAL_TERMINANTION 1
#endif

/* Bit definitions for the struct nxsig_defaction_s 'flags' field */

#define SIG_FLAG_NOCATCH (1 << 0)  /* Signal cannot be caught or ignored */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type provides the default action associated with a signal */

struct nxsig_defaction_s
{
  uint8_t       signo;   /* Signal number.  Range 1..MAX_SIGNO */
  uint8_t       flags;   /* See SIG_FLAG_ definitions */
  _sa_handler_t action;  /* Default signal action */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Default actions */

#ifdef HAVE_NXSIG_ABNORMAL_TERMINANTION
static void nxsig_abnormal_termination(int signo);
#endif
#ifdef CONFIG_SIG_SIGSTOP_ACTION
static void nxsig_null_action(int signo);
static void nxsig_stop_task(int signo);
#endif

/* Helpers */

static _sa_handler_t nxsig_default_action(int signo);
static void nxsig_setup_default_action(FAR struct task_group_s *group,
                                       FAR const struct nxsig_defaction_s *
                                           info);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* NOTE:  Default actions are not currently supported for very many signals.
 * However, this array is set up to support an indefinite number of default
 * signal actions.
 */

/* This table is used to make the default action for a signal to the correct
 * signal handler.  Signals whose default action is be ignored do not need
 * to be in this table (e.g., SIGCHLD).
 */

static const struct nxsig_defaction_s g_defactions[] =
{
#ifdef CONFIG_SIG_SIGUSR1_ACTION
  { SIGUSR1, 0,                nxsig_abnormal_termination },
#endif
#ifdef CONFIG_SIG_SIGUSR2_ACTION
  { SIGUSR2, 0,                nxsig_abnormal_termination },
#endif
#ifdef CONFIG_SIG_SIGALRM_ACTION
  { SIGALRM, 0,                nxsig_abnormal_termination },
#endif
#ifdef CONFIG_SIG_SIGPOLL_ACTION
  { SIGPOLL, 0,                nxsig_abnormal_termination },
#endif
#ifdef CONFIG_SIG_SIGSTOP_ACTION
  { SIGSTOP, SIG_FLAG_NOCATCH, nxsig_stop_task },
  { SIGSTP,  0,                nxsig_stop_task },
  { SIGCONT, SIG_FLAG_NOCATCH, nxsig_null_action },
#endif
#ifdef CONFIG_SIG_SIGKILL_ACTION
  { SIGINT,  0,                nxsig_abnormal_termination },
  { SIGKILL, SIG_FLAG_NOCATCH, nxsig_abnormal_termination },
  { SIGQUIT, 0,                nxsig_abnormal_termination },
  { SIGTERM, 0,                nxsig_abnormal_termination },
#endif
#ifdef CONFIG_SIG_SIGPIPE_ACTION
  { SIGPIPE, 0,                nxsig_abnormal_termination }
#endif
};

#define NACTIONS (sizeof(g_defactions) / sizeof(struct nxsig_defaction_s))

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
 * Name: nxsig_null_action
 *
 * Description:
 *   The do-nothing default signal action handler.
 *
 * Input Parameters:
 *   Standard signal handler parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SIG_SIGSTOP_ACTION
static void nxsig_null_action(int signo)
{
}
#endif

/****************************************************************************
 * Name: nxsig_abnormal_termination
 *
 * Description:
 *   This is the handler for the abnormal termination default action.
 *
 * Input Parameters:
 *   Standard signal handler parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef HAVE_NXSIG_ABNORMAL_TERMINANTION
static void nxsig_abnormal_termination(int signo)
{
  FAR struct tcb_s *rtcb = (FAR struct tcb_s *)this_task();

  /* Notify the target if the non-cancelable or deferred cancellation set */

  if (nxnotify_cancellation(rtcb))
    {
      return;
    }

  /* Careful:  In the multi-threaded task, the signal may be handled on a
   * child pthread.
   */

#ifdef HAVE_GROUP_MEMBERS
  /* Kill all of the children of the task.  This will not kill the currently
   * running task/pthread (this_task).  It will kill the main thread of the
   * task group if this_task is a pthread.
   */

  group_kill_children(rtcb);
#endif

#ifndef CONFIG_DISABLE_PTHREAD
  /* Check if the currently running task is actually a pthread */

  if ((rtcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_PTHREAD)
    {
      /* Exit the final thread of the task group.
       *
       * REVISIT:  This will not work if HAVE_GROUP_MEMBERS is not set.
       */

      pthread_exit(NULL);
    }
  else
#endif
    {
      /* Exit to terminate the task (note that exit() vs. _exit() is used. */

      exit(EXIT_FAILURE);
    }
}
#endif

/****************************************************************************
 * Name: nxsig_stop_task
 *
 * Description:
 *   This is the handler for the abnormal termination default action.
 *
 * Input Parameters:
 *   Standard signal handler parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SIG_SIGSTOP_ACTION
static void nxsig_stop_task(int signo)
{
  FAR struct tcb_s *rtcb = (FAR struct tcb_s *)this_task();
#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  FAR struct task_group_s *group;

  DEBUGASSERT(rtcb != NULL && rtcb->group != NULL);
  group = rtcb->group;
#endif

  /* Careful:  In the multi-threaded task, the signal may be handled on a
   * child pthread.
   */

#ifdef HAVE_GROUP_MEMBERS
  /* Suspend all of the children of the task.  This will not suspend the
   * currently running task/pthread (this_task).  It will suspend the
   * main thread of the task group if this_task is a pthread.
   */

  group_suspend_children(rtcb);
#endif

  /* Lock the scheduler so this thread is not pre-empted until after we
   * call nxsched_suspend().
   */

  sched_lock();

#if defined(CONFIG_SCHED_WAITPID) && !defined(CONFIG_SCHED_HAVE_PARENT)
  /* Notify via waitpid if any parent is waiting for this task to EXIT
   * or STOP.  This action is only performed if WUNTRACED is set in the
   * waitpid flags.
   */

  if ((group->tg_waitflags & WUNTRACED) != 0)
    {
      /* Return zero for exit status (we are not exiting, however) */

      if (group->tg_statloc != NULL)
        {
          *group->tg_statloc = 0;
           group->tg_statloc = NULL;
        }

      /* tg_waitflags == 0 means that the flags are available to another
       * caller of waitpid().
       */

      group->tg_waitflags = 0;

      /* YWakeup any tasks waiting for this task to exit or stop. */

      while (group->tg_exitsem.semcount < 0)
        {
          /* Wake up the thread */

          nxsem_post(&group->tg_exitsem);
        }
    }
#endif

  /* Then, finally, suspend this the final thread of the task group */

  nxsched_suspend(rtcb);
  sched_unlock();
}
#endif

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

static _sa_handler_t nxsig_default_action(int signo)
{
  int i;

  /* Search the default action table for the entry associated with this
   * signal.
   */

  for (i = 0; i < NACTIONS; i++)
    {
      if (g_defactions[i].signo == signo)
        {
          return g_defactions[i].action;
        }
    }

  /* No default action */

  return SIG_IGN;
}

/****************************************************************************
 * Name: nxsig_setup_default_action
 *
 * Description:
 *   Setup the default action for a signal.
 *
 *   This function is called early in task setup, prior to the creation of
 *   any pthreads so we should have exclusive access to the group structure.
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
                                       FAR const struct nxsig_defaction_s *
                                           info)
{
  /* Get the address of the handler for this signals default action. */

  if (info->action != SIG_IGN)
    {
      struct sigaction sa;

      /* Attach the signal handler.
       *
       * NOTE: nxsig_action will call nxsig_default(tcb, action, false).
       * Don't be surprised.
       */

      memset(&sa, 0, sizeof(sa));
      sa.sa_handler = info->action;
      sa.sa_flags   = SA_SIGINFO;
      nxsig_action(info->signo, &sa, NULL, true);

      /* Indicate that the default signal handler has been attached */

      nxsig_addset(&group->tg_sigdefault, (int)info->signo);
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

  ret = nxsig_ismember(&group->tg_sigdefault, signo);
  return ret < 0 ? false : (bool)ret;
}

/****************************************************************************
 * Name: nxsig_iscatchable
 *
 * Description:
 *   Return true if the specified signal can be caught or ignored.
 *
 * Input Parameters:
 *   signo - The signal number to be queried
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

bool nxsig_iscatchable(int signo)
{
  int i;

  /* Search the default action table for the entry associated with this
   * signal.
   */

  for (i = 0; i < NACTIONS; i++)
    {
      if (g_defactions[i].signo == signo)
        {
          return (g_defactions[i].flags & SIG_FLAG_NOCATCH) == 0;
        }
    }

  /* If it is not in the table, then it is catchable */

  return true;
}

/****************************************************************************
 * Name: nxsig_default
 *
 * Description:
 *   If 'defaction' is true, then return the default signal handler action
 *   for the specified signal and mark that the default signal handler is
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

_sa_handler_t nxsig_default(FAR struct tcb_s *tcb, int signo, bool defaction)
{
  FAR struct task_group_s *group;
  _sa_handler_t handler = SIG_IGN;
  irqstate_t flags;

  DEBUGASSERT(tcb != NULL && tcb->group != NULL);
  group = tcb->group;

  /* Are we setting or unsetting the default action? */

  if (defaction)
    {
      /* We are setting the default action.  Look up the default action
       * associated with signo.
       */

      handler = nxsig_default_action(signo);
      if (handler != SIG_IGN)
        {
          /* nxsig_addset() is not atomic (but neither is sigaction()) */

          flags = spin_lock_irqsave();
          nxsig_addset(&group->tg_sigdefault, signo);
          spin_unlock_irqrestore(flags);
        }
    }

  if (handler == SIG_IGN)
    {
      /* We are unsetting the default action. NOTE that nxsig_delset() is not
       * atomic (but neither is sigaction()).
       */

      flags = spin_lock_irqsave();
      nxsig_delset(&group->tg_sigdefault, signo);
      spin_unlock_irqrestore(flags);
    }

  return handler;
}

/****************************************************************************
 * Name: nxsig_default_initialize
 *
 * Description:
 *   Set all signals to their default action.  This is called from
 *   nxtask_start() to configure the newly started task.
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
  int i;

  DEBUGASSERT(tcb != NULL && tcb->group != NULL);
  group = tcb->group;

  /* Initialize the set of default signal handlers */

  sigemptyset(&group->tg_sigdefault);

  /* Setup the default action for each signal in g_defactions[] */

  for (i = 0; i < NACTIONS; i++)
    {
      nxsig_setup_default_action(group, &g_defactions[i]);
    }

  return OK;
}
