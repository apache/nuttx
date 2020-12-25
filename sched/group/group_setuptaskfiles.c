/****************************************************************************
 *  sched/group/group_setuptaskfiles.c
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
#include <errno.h>
#include <fcntl.h>

#include <nuttx/fs/fs.h>
#include <nuttx/net/net.h>

#include "sched/sched.h"
#include "group/group.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine how many file descriptors to clone.  If CONFIG_FDCLONE_DISABLE
 * is set, no file descriptors will be cloned.  If CONFIG_FDCLONE_STDIO is
 * set, only the first three descriptors (stdin, stdout, and stderr) will
 * be cloned.  Otherwise all file descriptors will be cloned.
 */

#if defined(CONFIG_FDCLONE_STDIO) && CONFIG_NFILE_DESCRIPTORS > 3
#  define NFDS_TOCLONE 3
#else
#  define NFDS_TOCLONE CONFIG_NFILE_DESCRIPTORS
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_dupfiles
 *
 * Description:
 *   Duplicate parent task's file descriptors.
 *
 * Input Parameters:
 *   tcb - tcb of the new task.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_FDCLONE_DISABLE
static inline void sched_dupfiles(FAR struct task_tcb_s *tcb)
{
  /* The parent task is the one at the head of the ready-to-run list */

  FAR struct tcb_s *rtcb = this_task();
  FAR struct file *parent;
  FAR struct file *child;
  int i;

  DEBUGASSERT(tcb && tcb->cmn.group && rtcb->group);

  /* Duplicate the file descriptors.  This will be either all of the
   * file descriptors or just the first three (stdin, stdout, and stderr)
   * if CONFIG_FDCLONE_STDIO is defined.  NFSDS_TOCLONE is set
   * accordingly above.
   */

  /* Get pointers to the parent and child task file lists */

  parent = rtcb->group->tg_filelist.fl_files;
  child  = tcb->cmn.group->tg_filelist.fl_files;

  /* Check each file in the parent file list */

  for (i = 0; i < NFDS_TOCLONE; i++)
    {
      /* Check if this file is opened by the parent.  We can tell if
       * if the file is open because it contain a reference to a non-NULL
       * i-node structure.
       */

      if (parent[i].f_inode &&
          (parent[i].f_oflags & O_CLOEXEC) == 0)
        {
          /* Yes... duplicate it for the child */

          file_dup2(&parent[i], &child[i]);
        }
    }
}
#else /* !CONFIG_FDCLONE_DISABLE */
#  define sched_dupfiles(tcb)
#endif /* !CONFIG_FDCLONE_DISABLE */

/****************************************************************************
 * Name: sched_dupsockets
 *
 * Description:
 *   Duplicate the parent task's socket descriptors.
 *
 * Input Parameters:
 *   tcb - tcb of the new task.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#if defined(CONFIG_NET) && !defined(CONFIG_SDCLONE_DISABLE)
static inline void sched_dupsockets(FAR struct task_tcb_s *tcb)
{
  /* The parent task is the one at the head of the ready-to-run list */

  FAR struct tcb_s *rtcb = this_task();
  FAR struct socket *parent;
  FAR struct socket *child;
  int i;

  /* Duplicate the socket descriptors of all sockets opened by the parent
   * task.
   */

  DEBUGASSERT(tcb && tcb->cmn.group && rtcb->group);

  /* Get pointers to the parent and child task socket lists */

  parent = rtcb->group->tg_socketlist.sl_sockets;
  child  = tcb->cmn.group->tg_socketlist.sl_sockets;

  /* Check each socket in the parent socket list */

  for (i = 0; i < CONFIG_NSOCKET_DESCRIPTORS; i++)
    {
      /* Check if this parent socket is valid.  Valid means both (1)
       * allocated and (2) successfully initialized.  A complexity in SMP
       * mode is that a socket my be allocated, but not yet initialized when
       * the socket is cloned by another pthread.
       *
       * Sockets with the close-on-exec flag set should not be cloned either.
       */

      if (_PS_VALID(&parent[i]) && !_SS_ISCLOEXEC(parent[i].s_flags))
        {
          /* Yes... duplicate it for the child */

          psock_dup2(&parent[i], &child[i]);
        }
    }
}
#else /* CONFIG_NET && !CONFIG_SDCLONE_DISABLE */
#  define sched_dupsockets(tcb)
#endif /* CONFIG_NET && !CONFIG_SDCLONE_DISABLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: group_setuptaskfiles
 *
 * Description:
 *   Configure a newly allocated TCB so that it will inherit
 *   file descriptors and streams from the parent task.
 *
 * Input Parameters:
 *   tcb - tcb of the new task.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   failure.
 *
 * Assumptions:
 *
 ****************************************************************************/

int group_setuptaskfiles(FAR struct task_tcb_s *tcb)
{
  FAR struct task_group_s *group = tcb->cmn.group;

  DEBUGASSERT(group);
#ifndef CONFIG_DISABLE_PTHREAD
  DEBUGASSERT((tcb->cmn.flags & TCB_FLAG_TTYPE_MASK) !=
              TCB_FLAG_TTYPE_PTHREAD);
#endif

  /* Initialize file descriptors for the TCB */

  files_initlist(&group->tg_filelist);

#ifdef CONFIG_NET
  /* Allocate socket descriptors for the TCB */

  net_initlist(&group->tg_socketlist);
#endif

  /* Duplicate the parent task's file descriptors */

  sched_dupfiles(tcb);

  /* Duplicate the parent task's socket descriptors */

  sched_dupsockets(tcb);

  /* Allocate file/socket streams for the new TCB */

#ifdef CONFIG_FILE_STREAM
  return group_setupstreams(tcb);
#else
  return OK;
#endif
}
