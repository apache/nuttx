/****************************************************************************
 * libs/libc/signal/sig_default.c
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

#include <sys/wait.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <signal.h>
#include <assert.h>

#include <nuttx/sched.h>
#include <nuttx/spinlock.h>
#include <nuttx/signal.h>
#include <nuttx/tls.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SIG_DEFAULT

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

#ifdef CONFIG_SIG_SIGSTOP_ACTION
static void sig_null_action(int signo);
#endif

#ifdef CONFIG_SIG_SIGKILL_ACTION
static void sig_kill_action(int signo, FAR siginfo_t *siginfo,
                            FAR void *context);
#endif

/* Helpers */

static _sa_handler_t nxsig_default_action(int signo);
static void nxsig_setup_default_action(FAR struct task_info_s *tinfo,
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
  { SIGUSR1, 0,                sig_kill_action },
#endif
#ifdef CONFIG_SIG_SIGUSR2_ACTION
  { SIGUSR2, 0,                sig_kill_action },
#endif
#ifdef CONFIG_SIG_SIGALRM_ACTION
  { SIGALRM, 0,                sig_kill_action },
#endif
#ifdef CONFIG_SIG_SIGPOLL_ACTION
  { SIGPOLL, 0,                sig_kill_action },
#endif
#ifdef CONFIG_SIG_SIGSTOP_ACTION
  { SIGSTOP, SIG_FLAG_NOCATCH, nxsig_stop_task },
  { SIGTSTP, 0,                nxsig_stop_task },
  { SIGCONT, SIG_FLAG_NOCATCH, sig_null_action },
#endif
#ifdef CONFIG_SIG_SIGKILL_ACTION
  { SIGINT,  0,                (_sa_handler_t)sig_kill_action },
  { SIGKILL, SIG_FLAG_NOCATCH, (_sa_handler_t)sig_kill_action },
  { SIGQUIT, 0,                (_sa_handler_t)sig_kill_action },
  { SIGTERM, 0,                (_sa_handler_t)sig_kill_action },
#endif
#ifdef CONFIG_SIG_SIGPIPE_ACTION
  { SIGPIPE, 0,                (_sa_handler_t)sig_kill_action }
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
 * Name: sig_null_action
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
static void sig_null_action(int signo)
{
}
#endif

/****************************************************************************
 * Name: sig_kill_action
 *
 * Description:
 *   The kill default signal action handler.
 *
 * Input Parameters:
 *   Standard signal handler parameters
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_SIG_SIGKILL_ACTION
static void sig_kill_action(int signo, FAR siginfo_t *siginfo,
                            FAR void *context)
{
  int flag = nxsig_abnormal_termination(signo);
  if (TCB_FLAG_TTYPE_PTHREAD == flag)
    {
      pthread_exit(PTHREAD_CANCELED);
    }
  else
    {
      exit(siginfo->si_value.sival_int);
    }
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

static void nxsig_setup_default_action(FAR struct task_info_s *tinfo,
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

      nxsig_addset(&tinfo->ta_sigdefault, (int)info->signo);
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
  FAR struct task_info_s *info;
  int ret;

  DEBUGASSERT(tcb && tcb->group && tcb->group->tg_info && GOOD_SIGNO(signo));
  info = tcb->group->tg_info;

  /* Return true if the signo is marked as using the default action.  Return
   * false in all other cases.
   */

  ret = nxsig_ismember(&info->ta_sigdefault, signo);
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
  FAR struct task_info_s *info;
  _sa_handler_t handler = SIG_IGN;
  irqstate_t flags;

  DEBUGASSERT(tcb && tcb->group && tcb->group->tg_info);
  info = tcb->group->tg_info;

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

          flags = spin_lock_irqsave(NULL);
          nxsig_addset(&info->ta_sigdefault, signo);
          spin_unlock_irqrestore(NULL, flags);
        }
    }

  if (handler == SIG_IGN)
    {
      /* We are unsetting the default action. NOTE that nxsig_delset() is not
       * atomic (but neither is sigaction()).
       */

      flags = spin_lock_irqsave(NULL);
      nxsig_delset(&info->ta_sigdefault, signo);
      spin_unlock_irqrestore(NULL, flags);
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
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   on failure.
 *
 ****************************************************************************/

int nxsig_default_initialize(void)
{
  FAR struct task_info_s *info = task_get_info();
  int i;

  /* Initialize the set of default signal handlers */

  sigemptyset(&info->ta_sigdefault);

  /* Setup the default action for each signal in g_defactions[] */

  for (i = 0; i < NACTIONS; i++)
    {
      nxsig_setup_default_action(info, &g_defactions[i]);
    }

  return OK;
}

#endif
