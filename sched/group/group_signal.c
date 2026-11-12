/****************************************************************************
 * sched/group/group_signal.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <assert.h>
#include <signal.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/signal.h>

#include "sched/sched.h"
#include "group/group.h"
#include "signal/signal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
struct group_signal_s
{
  FAR siginfo_t *siginfo; /* Signal to be dispatched */
  FAR const void *source; /* The POSIX timer of the signal, or NULL */
  bool limit;             /* Apply SIGQUEUE_MAX (sigqueue()) */
  pid_t dpid;             /* Default, valid pid */
  pid_t upid;             /* pid with this signal unblocked */
  pid_t apid;             /* This pid was awakened */
  pid_t ppid;             /* This pid received the signal */
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_signal_handler
 *
 * Description:
 *   Callback from group_foreachchild that handles one member of the group.
 *
 * Input Parameters:
 *   pid - The ID of the group member that may be signalled.
 *   arg - A pointer to a struct group_signal_s instance.
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

#ifdef HAVE_GROUP_MEMBERS
static int group_signal_handler(pid_t pid, FAR void *arg)
{
  FAR struct group_signal_s *info = (FAR struct group_signal_s *)arg;
  FAR struct tcb_s *tcb;
#ifdef CONFIG_ENABLE_ALL_SIGNALS
  FAR sigactq_t *sigact;
#endif
  int ret = 0;

  /* Get the TCB associated with the group member */

  tcb = nxsched_get_tcb(pid);
  DEBUGASSERT(tcb != NULL && tcb->group != NULL && info != NULL);

  /* Set this one as the default if we have not already set the
   * default.
   */

  if (!info->dpid)
    {
      info->dpid = pid;
    }

  /* Is the thread waiting for this signal (in this case, the signal is
   * probably blocked).
   */

  ret = nxsig_ismember(&tcb->sigwaitmask, info->siginfo->si_signo);
  if (ret == 1 && (!info->apid || info->siginfo->si_signo == SIGCHLD))
    {
      /* Yes.. This means that the task is suspended, waiting for this
       * signal to occur. Stop looking and use this TCB.  The
       * requirement is this:  If a task group receives a signal and
       * more than one thread is waiting on that signal, then one and
       * only one indeterminate thread out of that waiting group will
       * receive the signal.
       */

      ret = nxsig_tcbdispatch(tcb, info->siginfo, true, info->source,
                              info->limit);
      if (ret < 0)
        {
          goto errout;
        }

      /* Limit to one thread */

      info->apid = pid;

      if (info->ppid != 0 && info->siginfo->si_signo != SIGCHLD)
        {
          ret = 1; /* Terminate the search */
          goto errout;
        }
    }

  /* Is this signal unblocked on this thread? */

  if ((nxsig_ismember(&tcb->sigprocmask, info->siginfo->si_signo) != 1) &&
      !info->ppid && pid != info->apid)
    {
      /* Yes.. remember this TCB if we have not encountered any
       * other threads that have the signal unblocked.
       */

      if (!info->upid)
        {
          info->upid = pid;
        }

      /* Is there also a action associated with the task group? */

#ifdef CONFIG_ENABLE_ALL_SIGNALS
      sigact = nxsig_find_action(tcb->group, info->siginfo->si_signo);
      if (sigact)
        {
          /* Yes.. then use this thread.  The requirement is this:
           * If a task group receives a signal then one and only one
           * indeterminate thread in the task group which is not
           * blocking the signal will receive the signal.
           */

          ret = nxsig_tcbdispatch(tcb, info->siginfo, true, info->source,
                                  info->limit);
          if (ret < 0)
            {
              goto errout;
            }

          /* Limit to one thread */

          info->ppid = pid;
          if (info->apid != 0)
            {
              ret = 1; /* Terminate the search */
              goto errout;
            }
        }
#endif
    }

errout:
  nxsched_put_tcb(tcb);
  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_signal
 *
 * Description:
 *   Send a signal to every member of the group.
 *
 * Input Parameters:
 *   group - The task group that needs to be signalled.
 *   limit - true for sigqueue(): fail with -EAGAIN at SIGQUEUE_MAX.
 *
 * Returned Value:
 *   0 (OK) on success; a negated errno value on failure.
 *
 * Assumptions:
 *   Called during task termination in a safe context. No special precautions
 *   are required here.  Because signals can be sent from interrupt handlers,
 *   this function may be called indirectly in the context of an interrupt
 *   handler.
 *
 ****************************************************************************/

int group_signal(FAR struct task_group_s *group, FAR siginfo_t *siginfo,
                 FAR const void *source, bool limit)
{
#ifdef HAVE_GROUP_MEMBERS
  struct group_signal_s info;
  FAR struct tcb_s *tcb;
  pid_t pid;
  int ret;

  DEBUGASSERT(group && siginfo);

  info.siginfo = siginfo;
  info.source  = source;
  info.limit   = limit;
  info.dpid    = 0;       /* Default, valid pid */
  info.upid    = 0;       /* pid with this signal unblocked */
  info.apid    = 0;       /* This pid was awakened */
  info.ppid    = 0;       /* This pid received the signal */

  /* Make sure that pre-emption is disabled to that we signal all of the
   * members of the group before any of them actually run. (This does
   * nothing if were were called from an interrupt handler).
   */

#ifdef CONFIG_SMP
  irqstate_t flags = enter_critical_section();
#else
  sched_lock();
#endif

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

  if (info.apid == 0 && info.ppid == 0)
    {
      if (info.upid)
        {
          pid = info.upid;
        }

      /* Otherwise use the default TCB.  There should always be a default
       * TCB. It will have the signal blocked, but can be used to get the
       * signal to a pending state.
       */

      else if (info.dpid)
        {
          pid = info.dpid;
        }
      else
        {
          ret = -ECHILD;
          goto errout;
        }

      /* Now deliver the signal to the selected group member */

      tcb = nxsched_get_tcb(pid);
      if (tcb)
        {
          ret = nxsig_tcbdispatch(tcb, siginfo, true, source, limit);
        }
      else
        {
          ret = -ESRCH;
        }

      nxsched_put_tcb(tcb);
    }

errout:
#ifdef CONFIG_SMP
  leave_critical_section(flags);
#else
  sched_unlock();
#endif
  return ret;

#else

  UNUSED(group);
  UNUSED(siginfo);
  UNUSED(source);
  UNUSED(limit);
  return -ENOSYS;

#endif
}
