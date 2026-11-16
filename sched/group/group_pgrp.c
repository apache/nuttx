/****************************************************************************
 * sched/group/group_pgrp.c
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

#include <sys/types.h>
#include <errno.h>
#include <unistd.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>

#include "sched/sched.h"
#include "group/group.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct group_find_s
{
  pid_t pgid;   /* The process group ID */
  pid_t sid;    /* The session ID, or 0 for each session */
  bool  found;  /* A process of the process group is in the session */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_find_pgrp
 *
 * Description:
 *   The nxsched_foreach() function of group_has_pgrp().
 *
 ****************************************************************************/

static void group_find_pgrp(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct group_find_s *find = arg;

  if (group_in_pgrp(tcb, find->pgid) &&
      (find->sid == 0 || tcb->group->tg_sid == find->sid))
    {
      find->found = true;
    }
}

/****************************************************************************
 * Name: group_has_pgrp
 *
 * Description:
 *   Return true if a process of the session sid (0: of any session) is in
 *   the process group pgid.
 *
 ****************************************************************************/

static bool group_has_pgrp(pid_t pgid, pid_t sid)
{
  struct group_find_s find;

  find.pgid  = pgid;
  find.sid   = sid;
  find.found = false;
  nxsched_foreach(group_find_pgrp, &find);
  return find.found;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_in_pgrp
 *
 * Description:
 *   Return true if tcb is the main thread of a process in the process
 *   group pgid.  A process that exits is in no process group.
 *
 ****************************************************************************/

bool group_in_pgrp(FAR struct tcb_s *tcb, pid_t pgid)
{
  FAR struct task_group_s *group = tcb->group;

  return pgid > 0 && group != NULL && tcb->pid == group->tg_pid &&
         group->tg_pgid == pgid &&
         (tcb->flags & TCB_FLAG_EXIT_PROCESSING) == 0 &&
         (group->tg_flags & GROUP_FLAG_EXITING) == 0;
}

/****************************************************************************
 * Name: group_setpgid
 *
 * Description:
 *   The setpgid() of the kernel.  POSIX_SPAWN_SETPGROUP uses it before the
 *   new process runs.
 *
 * Returned Value:
 *   Zero (OK) on success, or a negated errno value as setpgid() gives.
 *
 ****************************************************************************/

int group_setpgid(pid_t pid, pid_t pgid)
{
  FAR struct task_group_s *rgroup = this_task()->group;
  FAR struct task_group_s *group;
  irqstate_t flags;
  int ret = OK;

  if (pgid < 0)
    {
      return -EINVAL;
    }

  if (pid < 0)
    {
      return -ESRCH;
    }

  flags = enter_critical_section();

  if (pid == 0)
    {
      pid = rgroup->tg_pid;
    }

  if (pgid == 0)
    {
      pgid = pid;
    }

  /* The caller, or a child of the caller in its session that did not
   * exec().
   */

  group = task_getgroup(pid);
  if (group == NULL || group->tg_pid != pid)
    {
      ret = -ESRCH;
    }
  else if (group != rgroup)
    {
      if (group->tg_ppid != rgroup->tg_pid)
        {
          ret = -ESRCH;
        }
      else if (group->tg_sid != rgroup->tg_sid)
        {
          ret = -EPERM;
        }
      else if ((group->tg_flags & GROUP_FLAG_EXECED) != 0)
        {
          ret = -EACCES;
        }
    }

  /* A session leader keeps its process group.  Another process group must
   * be in the session of the caller.
   */

  if (ret == OK)
    {
      if (group->tg_sid == group->tg_pid ||
          (pgid != pid && !group_has_pgrp(pgid, rgroup->tg_sid)))
        {
          ret = -EPERM;
        }
      else
        {
          group->tg_pgid = pgid;
        }
    }

  leave_critical_section(flags);
  return ret;
}

/****************************************************************************
 * Name: setpgid
 *
 * Description:
 *   Put the process pid into the process group pgid: a process group of
 *   the session of the caller, or a new process group with the ID pid.
 *
 * Input Parameters:
 *   pid  - The caller or a child of the caller.  Zero is the caller.
 *   pgid - The process group ID.  Zero is pid.
 *
 * Returned Value:
 *   Zero (OK) on success.  Otherwise -1 (ERROR) and errno is set:
 *
 *   EACCES - pid is a child of the caller after an exec().
 *   EINVAL - pgid is less than zero.
 *   EPERM  - pid is a session leader or a child in another session, or
 *            the session of the caller has no process group pgid.
 *   ESRCH  - pid is not the caller and not a child of the caller.
 *
 ****************************************************************************/

int setpgid(pid_t pid, pid_t pgid)
{
  int ret = group_setpgid(pid, pgid);

  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

/****************************************************************************
 * Name: getpgid
 *
 * Description:
 *   Return the process group ID of the process pid (the caller if pid is
 *   zero).  On failure, return -1 (ERROR) and set errno: EINVAL if pid is
 *   less than zero, ESRCH if no process has the ID pid.
 *
 ****************************************************************************/

pid_t getpgid(pid_t pid)
{
  FAR struct task_group_s *group;
  irqstate_t flags;
  pid_t ret;

  if (pid < 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (pid == 0)
    {
      return this_task()->group->tg_pgid;
    }

  flags = enter_critical_section();
  group = task_getgroup(pid);
  ret   = group != NULL ? group->tg_pgid : ERROR;
  leave_critical_section(flags);

  if (ret == ERROR)
    {
      set_errno(ESRCH);
    }

  return ret;
}

/****************************************************************************
 * Name: setsid
 *
 * Description:
 *   Make the caller the leader of a new session and of a new process
 *   group.  Their ID is the process ID of the caller.
 *
 * Returned Value:
 *   The new session ID.  Otherwise -1 (ERROR) and errno is set to EPERM:
 *   the caller is a process group leader, or the process group ID of
 *   another process is the process ID of the caller.
 *
 ****************************************************************************/

pid_t setsid(void)
{
  FAR struct task_group_s *group = this_task()->group;
  irqstate_t flags;
  pid_t ret;

  flags = enter_critical_section();

  if (group->tg_pgid == group->tg_pid || group_has_pgrp(group->tg_pid, 0))
    {
      ret = ERROR;
    }
  else
    {
      group->tg_pgid = group->tg_pid;
      group->tg_sid  = group->tg_pid;
      ret            = group->tg_pid;
    }

  leave_critical_section(flags);

  if (ret == ERROR)
    {
      set_errno(EPERM);
    }

  return ret;
}

/****************************************************************************
 * Name: getsid
 *
 * Description:
 *   Return the session ID of the process pid (the caller if pid is zero).
 *   On failure, return -1 (ERROR) and set errno: EINVAL if pid is less
 *   than zero, ESRCH if no process has the ID pid.
 *
 ****************************************************************************/

pid_t getsid(pid_t pid)
{
  FAR struct task_group_s *group;
  irqstate_t flags;
  pid_t ret;

  if (pid < 0)
    {
      set_errno(EINVAL);
      return ERROR;
    }

  if (pid == 0)
    {
      return this_task()->group->tg_sid;
    }

  flags = enter_critical_section();
  group = task_getgroup(pid);
  ret   = group != NULL ? group->tg_sid : ERROR;
  leave_critical_section(flags);

  if (ret == ERROR)
    {
      set_errno(ESRCH);
    }

  return ret;
}
