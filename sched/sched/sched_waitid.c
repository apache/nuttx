/****************************************************************************
 * sched/sched/sched_waitid.c
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

#include <sys/wait.h>
#include <signal.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/sched.h>
#include <nuttx/signal.h>
#include <nuttx/cancelpt.h>

#include "sched/sched.h"
#include "group/group.h"

#if defined(CONFIG_SCHED_WAITPID) && defined(CONFIG_SCHED_HAVE_PARENT)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* The search state of child_running() */

struct child_search_s
{
  FAR struct task_group_s *group;   /* The parent task group */
  idtype_t idtype;                  /* P_ALL, P_PID or P_PGID */
  pid_t id;                         /* The process ID or process group ID */
  bool found;                       /* A child that idtype and id select */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: report_child
 *
 * Description:
 *   Give the event of a child (see group_child_event()) in info.  The
 *   event is then forgotten, and a child that exited is discarded, unless
 *   WNOWAIT keeps the child in a waitable state.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_CHILD_STATUS
static void report_child(FAR struct tcb_s *rtcb,
                         FAR struct child_status_s *child,
                         FAR siginfo_t *info, int options)
{
  uint8_t event = group_child_event(child, options);

  info->si_signo           = SIGCHLD;
  info->si_errno           = OK;
  info->si_value.sival_ptr = NULL;
  info->si_pid             = child->ch_pid;
#ifdef CONFIG_SCHED_USER_IDENTITY
  info->si_uid             = child->ch_uid;
#else
  info->si_uid             = 0;
#endif

  if (event == CHILD_FLAG_STOPPED)
    {
      info->si_code   = CLD_STOPPED;
      info->si_status = child->ch_stopsig;
    }
  else if (event == CHILD_FLAG_CONTINUED)
    {
      info->si_code   = CLD_CONTINUED;
      info->si_status = SIGCONT;
    }
  else
    {
      info->si_code   = CLD_EXITED;
      info->si_status = child->ch_status;
    }

  if ((options & WNOWAIT) == 0)
    {
      if (event == CHILD_FLAG_EXITED)
        {
          group_remove_child(rtcb->group, child->ch_pid);
          group_free_child(child);
        }
      else
        {
          child->ch_flags &= ~event;
        }
    }
}
#endif

/****************************************************************************
 * Name: child_handler
 *
 * Description:
 *   The nxsched_foreach() handler of child_running().
 *
 ****************************************************************************/

static void child_handler(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct child_search_s *search = arg;
  FAR struct task_group_s *group = tcb->group;

  if (group == NULL || group == search->group ||
      group->tg_ppid != search->group->tg_pid)
    {
      return;
    }

#ifdef CONFIG_SCHED_CHILD_STATUS
  /* A child that sent SIGCHLD can still be found during its exit */

  if ((tcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
    {
      return;
    }
#endif

  if (search->idtype == P_ALL ||
      (search->idtype == P_PID && group->tg_pid == search->id) ||
      (search->idtype == P_PGID &&
       group_getpgid(group) == search->id))
    {
      search->found = true;
    }
}

/****************************************************************************
 * Name: child_running
 *
 * Description:
 *   Return true if a child that idtype and id select still runs.  This
 *   also finds the children that have no child status structure.
 *
 ****************************************************************************/

static bool child_running(FAR struct tcb_s *rtcb, idtype_t idtype, pid_t id)
{
  struct child_search_s search;

#ifndef CONFIG_SCHED_CHILD_STATUS
  if (rtcb->group->tg_nchildren == 0)
    {
      return false;
    }

  if (idtype == P_ALL)
    {
      return true;
    }

  if (idtype == P_PID)
    {
      return nxsched_get_tcb(id) != NULL;
    }
#endif

  search.group  = rtcb->group;
  search.idtype = idtype;
  search.id     = id;
  search.found  = false;
  nxsched_foreach(child_handler, &search);
  return search.found;
}

/****************************************************************************
 * Name: child_selected
 *
 * Description:
 *   Without a child status: return true if the SIGCHLD in info is an event
 *   that options wait for, of a child that idtype and id select.  A child
 *   that is gone is taken to be in the process group.
 *
 ****************************************************************************/

static bool child_selected(FAR siginfo_t *info, idtype_t idtype, pid_t id,
                           int options)
{
  FAR struct task_group_s *chgrp;

  if (info->si_signo != SIGCHLD)
    {
      return false;
    }

  if (info->si_code == CLD_STOPPED)
    {
      if ((options & (WUNTRACED | WSTOPPED)) == 0)
        {
          return false;
        }
    }
  else if (info->si_code == CLD_CONTINUED)
    {
      if ((options & WCONTINUED) == 0)
        {
          return false;
        }
    }
  else
    {
#ifdef CONFIG_SCHED_CHILD_STATUS
      /* The exit status is not kept (XSI): wait for all children */

      return false;
#else
      if ((options & WEXITED) == 0)
        {
          return false;
        }
#endif
    }

  if (idtype == P_PGID)
    {
      chgrp = task_getgroup(info->si_pid);
      return chgrp == NULL || group_getpgid(chgrp) == id;
    }

  return idtype == P_ALL || info->si_pid == id;
}

/****************************************************************************
 * Name: check_children
 *
 * Description:
 *   Return -ECHILD if idtype and id select no child of the caller.
 *
 ****************************************************************************/

static int check_children(FAR struct tcb_s *rtcb, idtype_t idtype, pid_t id)
{
#ifdef CONFIG_SCHED_CHILD_STATUS
  bool found;

  /* Without the child status, the wait loop checks the running children */

  if ((rtcb->group->tg_flags & GROUP_FLAG_NOCLDWAIT) != 0)
    {
      return OK;
    }

  group_wait_child(rtcb->group, idtype, id, 0, &found);
  return found ? OK : -ECHILD;
#else
  FAR struct tcb_s *ctcb;

  if (idtype == P_PID)
    {
      ctcb = nxsched_get_tcb(id);
      if (ctcb == NULL || ctcb->group == NULL ||
          ctcb->group->tg_ppid != rtcb->group->tg_pid ||
          (ctcb->flags & TCB_FLAG_EXIT_PROCESSING) != 0)
        {
          return -ECHILD;
        }
    }

  return child_running(rtcb, idtype, id) ? OK : -ECHILD;
#endif
}

/****************************************************************************
 * Name: waittcb
 *
 * Description:
 *   Wait until a child that idtype and id select has an event that options
 *   wait for.
 *
 ****************************************************************************/

static int waittcb(FAR struct tcb_s *rtcb, idtype_t idtype, pid_t id,
                   FAR siginfo_t *info, int options)
{
#ifdef CONFIG_SCHED_CHILD_STATUS
  FAR struct child_status_s *child;
#endif
  bool retains = false;
  bool found = false;
  sigset_t set;
  int ret;

  /* Create a signal set that contains only SIGCHLD */

  sigemptyset(&set);
  nxsig_addset(&set, SIGCHLD);

  for (; ; )
    {
#ifdef CONFIG_SCHED_CHILD_STATUS
      /* Another thread can change the action for SIGCHLD while we wait */

      retains = (rtcb->group->tg_flags & GROUP_FLAG_NOCLDWAIT) == 0;
      found   = false;
      if (retains)
        {
          /* Signals are not queued in NuttX, so a SIGCHLD can be lost.
           * The child status keeps each event: look there first.
           */

          child = group_wait_child(rtcb->group, idtype, id, options,
                                   &found);
          if (child != NULL)
            {
              report_child(rtcb, child, info, options);
              return OK;
            }
        }
#endif

      /* Without a child status (XSI, or another thread took it), wait
       * while a child runs.  Without the child status option, the child
       * ended and the SIGCHLD was lost: ECHILD is better than a wrong
       * status.
       */

      if (!found && !child_running(rtcb, idtype, id))
        {
          return -ECHILD;
        }

      if ((options & WNOHANG) != 0)
        {
          /* SUSv4 says:
           *
           * "If waitid() returns because WNOHANG was specified and status
           * is not available for any process specified by idtype and id,
           * then the si_signo and si_pid members of the structure pointed
           * to by infop shall be set to zero and the values of other
           * members of the structure are unspecified."
           */

          info->si_signo = 0;
          info->si_pid   = 0;
          return OK;
        }

      /* Wait for the next SIGCHLD */

      ret = nxsig_timedwait(&set, info, NULL);
      if (ret < 0)
        {
          return ret;
        }

      /* Without a child status, the SIGCHLD tells the event */

      if (!retains && child_selected(info, idtype, id, options))
        {
          return OK;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxsched_waitid
 *
 * Description:
 *   The common part of waitid() and waitpid(): wait until a child that
 *   idtype and id select exited (WEXITED), stopped (WSTOPPED or WUNTRACED)
 *   or continued (WCONTINUED), and give the event in info as the SIGCHLD of
 *   the event.  P_PGID with an id of 0 selects the process group of the
 *   caller.
 *
 * Input Parameters:
 *   idtype  - P_PID, P_PGID or P_ALL
 *   id      - The process ID or the process group ID
 *   info    - The location for the event
 *   options - See waitid(); WUNTRACED is the same as WSTOPPED
 *
 * Returned Value:
 *   OK (0) on success; with WNOHANG and no event, si_pid in info is 0.
 *   A negated errno value on failure (ECHILD, EINTR).
 *
 ****************************************************************************/

int nxsched_waitid(idtype_t idtype, pid_t id, FAR siginfo_t *info,
                   int options)
{
  FAR struct tcb_s *rtcb = this_task();
  irqstate_t flags;
  int ret;

  if (idtype == P_PGID && id == 0)
    {
      id = group_getpgid(rtcb->group);
    }

  /* NOTE: sched_lock() is not enough for SMP
   * because the child task is running on another CPU
   */

  flags = enter_critical_section();

  ret = check_children(rtcb, idtype, id);
  if (ret == OK)
    {
      ret = waittcb(rtcb, idtype, id, info, options);
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: waitid
 *
 * Description:
 *   The waitid() function suspends the calling thread until one child of
 *   the process containing the calling thread changes state. It records the
 *   current state of a child in the structure pointed to by 'info'. If a
 *   child process changed state prior to the call to waitid(), waitid()
 *   returns immediately. If more than one thread is suspended in wait() or
 *   waitpid() waiting termination of the same process, exactly one thread
 *   will return the process status at the time of the target process
 *   termination
 *
 *   The idtype and id arguments are used to specify which children waitid()
 *   will wait for.
 *
 *     If idtype is P_PID, waitid() will wait for the child with a process
 *     ID equal to (pid_t)id.
 *
 *     If idtype is P_PGID, waitid() will wait for any child with a process
 *     group ID equal to (pid_t)id.
 *
 *     If idtype is P_ALL, waitid() will wait for any children and id is
 *     ignored.
 *
 *   The options argument is used to specify which state changes waitid()
 *   will will wait for. It is formed by OR-ing together one or more of the
 *   following flags:
 *
 *     WEXITED - Wait for processes that have exited.
 *     WSTOPPED - Status will be returned for any child that has stopped
 *       upon receipt of a signal.
 *     WCONTINUED - Status will be returned for any child that was stopped
 *       and has been continued.
 *     WNOHANG - Return immediately if there are no children to wait for.
 *     WNOWAIT - Keep the process whose status is returned in 'info' in a
 *       waitable state. This will not affect the state of the process; the
 *       process may be waited for again after this call completes.
 *
 *   The 'info' argument must point to a siginfo_t structure. If waitid()
 *   returns because a child process was found that satisfied the conditions
 *   indicated by the arguments idtype and options, then the structure
 *   pointed to by 'info' will be filled in by the system with the status of
 *   the process. The si_signo member will always be equal to SIGCHLD.
 *
 * Input Parameters:
 *   See description.
 *
 * Returned Value:
 *   If waitid() returns due to the change of state of one of its children,
 *   0 is returned. Otherwise, -1 is returned and errno is set to indicate
 *   the error.
 *
 *   The waitid() function will fail if:
 *
 *     ECHILD - The calling process has no existing unwaited-for child
 *       processes.
 *     EINTR - The waitid() function was interrupted by a signal.
 *     EINVAL - An invalid value was specified for options, or idtype and id
 *       specify an invalid set of processes.
 *
 ****************************************************************************/

int waitid(idtype_t idtype, id_t id, FAR siginfo_t *info, int options)
{
  int ret;

  /* waitid() is a cancellation point */

  enter_cancellation_point();

  if ((idtype != P_PID && idtype != P_PGID && idtype != P_ALL) ||
      (options & ~(WEXITED | WSTOPPED | WCONTINUED | WNOHANG |
                   WNOWAIT)) != 0 ||
      (options & (WEXITED | WSTOPPED | WCONTINUED)) == 0)
    {
      ret = -EINVAL;
    }
  else
    {
      ret = nxsched_waitid(idtype, id, info, options);
    }

  leave_cancellation_point();
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

#endif /* CONFIG_SCHED_WAITPID && CONFIG_SCHED_HAVE_PARENT */
