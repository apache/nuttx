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
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sched.h>

#include "inode/inode.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: task_fssync
 *
 * Description:
 *   nxsched_foreach() callback that fsyncs every open file of one task.
 *
 ****************************************************************************/

static void task_fssync(FAR struct tcb_s *tcb, FAR void *arg)
{
  FAR struct tcb_s *ctcb;
  int pid = tcb->pid;
  int count;
  int fd;

  if (tcb->group == NULL)
    {
      return;
    }

  count = fdlist_count(&tcb->group->tg_fdlist);

  for (fd = 0; fd < count; fd++)
    {
      FAR struct file *filep;

      /* file_fsync() may block and let the task exit meanwhile */

      ctcb = nxsched_get_tcb(pid);
      if (ctcb == NULL || ctcb->group == NULL || ctcb != tcb)
        {
          return;
        }

      if (fdlist_get(&ctcb->group->tg_fdlist, fd, &filep) >= 0)
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
  nxsched_foreach(task_fssync, NULL);

#ifndef CONFIG_DISABLE_MOUNTPOINT
  foreach_inode(mountpt_syncfs, NULL, true);
#endif
}
