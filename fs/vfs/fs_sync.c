/****************************************************************************
 * fs/vfs/fs_sync.c
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

#include <limits.h>
#include <string.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/sched.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The number of task IDs that one walk of the task list collects */

#define SYNC_NPIDS 16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sync_pids_s
{
  pid_t last;                /* Collect only the IDs above this one */
  int   npids;               /* The number of IDs in pids[] */
  pid_t pids[SYNC_NPIDS];    /* The smallest IDs above last, in order */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sync_collect
 *
 * Description:
 *   nxsched_foreach() callback that keeps the SYNC_NPIDS smallest task IDs
 *   above sp->last, in order.  It runs in the critical section, so it only
 *   reads the TCB.
 *
 ****************************************************************************/

static void sync_collect(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct sync_pids_s *sp = arg;
  pid_t pid = tcb->pid;
  int n;
  int i;

  if (pid <= sp->last ||
      (sp->npids == SYNC_NPIDS && pid > sp->pids[SYNC_NPIDS - 1]))
    {
      return;
    }

  /* Insert the ID in order.  If the array is full, the greatest ID drops
   * out, and a later walk collects it again.
   */

  n = sp->npids < SYNC_NPIDS ? sp->npids++ : SYNC_NPIDS - 1;
  i = 0;
  while (i < n && sp->pids[i] < pid)
    {
      i++;
    }

  memmove(&sp->pids[i + 1], &sp->pids[i], (n - i) * sizeof(sp->pids[0]));
  sp->pids[i] = pid;
}

/****************************************************************************
 * Name: task_fssync
 *
 * Description:
 *   Fsync every open file of one task.  This runs outside the critical
 *   section.
 *
 ****************************************************************************/

static void task_fssync(pid_t pid)
{
  FAR struct tcb_s *tcb;
  FAR struct file *filep;
  irqstate_t flags;
  int ret;
  int fd;

  for (fd = 0; ; fd++)
    {
      /* The TCB and the descriptor list are valid only in the critical
       * section that finds the task.  The file reference keeps the file.
       */

      flags = enter_critical_section();
      tcb = nxsched_get_tcb(pid);
      if (tcb == NULL || tcb->group == NULL ||
          fd >= fdlist_count(&tcb->group->tg_fdlist))
        {
          leave_critical_section(flags);
          break;
        }

      ret = fdlist_get(&tcb->group->tg_fdlist, fd, &filep);
      leave_critical_section(flags);

      if (ret >= 0)
        {
          file_fsync(filep);
          file_put(filep);
        }
    }
}

#ifndef CONFIG_DISABLE_MOUNTPOINT
/****************************************************************************
 * Name: mountpt_syncfs
 *
 * Description:
 *   foreach_inode() callback that flushes one mounted filesystem.  syncfs()
 *   also reaches state that no open descriptor of any task refers to.
 *
 ****************************************************************************/

static int mountpt_syncfs(FAR struct inode *inode,
                          FAR char dirpath[PATH_MAX], FAR void *arg)
{
  if (INODE_IS_MOUNTPT(inode) && inode->u.i_mops != NULL &&
      inode->u.i_mops->syncfs != NULL)
    {
      inode->u.i_mops->syncfs(inode);
    }

  return 0;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sync
 *
 * Description:
 *   sync() causes all pending modifications to filesystem metadata and
 *   cached file data to be written to the underlying filesystems.
 *
 ****************************************************************************/

void sync(void)
{
  struct sync_pids_s sp;
  int i;

  /* A file method can take a driver spinlock whose holder enters the
   * critical section (a pipe posts a semaphore with d_bflock held).  So
   * each walk only collects task IDs, and the fsyncs come after the walk.
   */

  sp.last = -1;
  do
    {
      sp.npids = 0;
      nxsched_foreach(sync_collect, &sp);

      for (i = 0; i < sp.npids; i++)
        {
          task_fssync(sp.pids[i]);
        }

      if (sp.npids > 0)
        {
          sp.last = sp.pids[sp.npids - 1];
        }
    }
  while (sp.npids == SYNC_NPIDS);

#ifndef CONFIG_DISABLE_MOUNTPOINT
  foreach_inode(mountpt_syncfs, NULL, true);
#endif
}
