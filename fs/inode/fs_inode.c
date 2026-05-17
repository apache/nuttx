/****************************************************************************
 * fs/inode/fs_inode.c
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

#include <assert.h>
#include <errno.h>
#include <fcntl.h>

#include <nuttx/fs/fs.h>
#include <nuttx/rwsem.h>
#include <nuttx/sched.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static rw_semaphore_t g_inode_lock = RWSEM_INITIALIZER;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: inode_initialize
 *
 * Description:
 *   This is called from the OS initialization logic to configure the file
 *   system.
 *
 ****************************************************************************/

void inode_initialize(void)
{
  /* Reserve the root node */

  inode_root_reserve();
}

/****************************************************************************
 * Name: inode_lock
 *
 * Description:
 *   Get writeable exclusive access to the in-memory inode tree.
 *
 ****************************************************************************/

void inode_lock(void)
{
  down_write(&g_inode_lock);
}

/****************************************************************************
 * Name: inode_rlock
 *
 * Description:
 *   Get readable exclusive access to the in-memory inode tree.
 *
 ****************************************************************************/

void inode_rlock(void)
{
  down_read(&g_inode_lock);
}

/****************************************************************************
 * Name: inode_unlock
 *
 * Description:
 *   Relinquish writeable exclusive access to the in-memory inode tree.
 *
 ****************************************************************************/

void inode_unlock(void)
{
  up_write(&g_inode_lock);
}

/****************************************************************************
 * Name: inode_runlock
 *
 * Description:
 *   Relinquish read exclusive access to the in-memory inode tree.
 *
 ****************************************************************************/

void inode_runlock(void)
{
  up_read(&g_inode_lock);
}

/****************************************************************************
 * Name: inode_checkperm
 *
 * Description:
 *   Validate that 'inode' can be opened with the access described by
 *   'oflags'.  Two sequential checks are performed:
 *
 *   1. Operation-support check (all inode types, unconditional):
 *      Verifies the driver exposes the read/write entry points required by
 *      'oflags'.  Returns -ENXIO when ops are NULL and -EACCES when the
 *      required entry point is absent.  Pseudo-directory inodes
 *      (INODE_IS_PSEUDODIR) are exempted from this step.
 *
 *   2. UNIX permission check (pseudo-filesystem inodes only):
 *      Compares effective uid/gid against i_mode owner/group/other bits.
 *      Mountpoint inodes and kernel threads are unconditionally exempted.
 *      Requires CONFIG_PSEUDOFS_ATTRIBUTES and CONFIG_SCHED_USER_IDENTITY;
 *      when either option is disabled this step is a no-op.
 *
 * Input Parameters:
 *   inode  - The inode to check
 *   oflags - Open flags (O_RDONLY / O_WRONLY / O_RDWR)
 *
 * Returned Value:
 *   Zero (OK) on success.  Negated errno on failure:
 *     -ENXIO   ops pointer is NULL
 *     -EACCES  required operation not supported, or permission denied
 *
 ****************************************************************************/

int inode_checkperm(FAR struct inode *inode, int oflags)
{
#if defined(CONFIG_PSEUDOFS_ATTRIBUTES) && defined(CONFIG_SCHED_USER_IDENTITY)
  FAR struct tcb_s *rtcb;
  mode_t perm;
  uid_t uid;
  gid_t gid;
#endif
  FAR const struct file_operations *ops;

  /* === Step 1: operation-support check === */

  /* Pseudo-directories carry no ops and are always accessible */

  if (INODE_IS_PSEUDODIR(inode))
    {
      return OK;
    }

  ops = inode->u.i_ops;
  if (ops == NULL)
    {
      return -ENXIO;
    }

  if (((oflags & O_RDOK) != 0 &&
       !ops->readv && !ops->read && !ops->ioctl) ||
      ((oflags & O_WROK) != 0 &&
       !ops->writev && !ops->write && !ops->ioctl))
    {
      return -EACCES;
    }

#if defined(CONFIG_PSEUDOFS_ATTRIBUTES) && defined(CONFIG_SCHED_USER_IDENTITY)

  /* === Step 2: UNIX permission check (pseudo-filesystem inodes only) === */

  /* Mountpoints delegate permission enforcement to the underlying
   * filesystem
   */

  if (INODE_IS_MOUNTPT(inode))
    {
      return OK;
    }

  /* Kernel threads are always granted access */

  rtcb = nxsched_self();
  if ((rtcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      return OK;
    }

  /* Use effective credentials */

  DEBUGASSERT(rtcb->group != NULL);
  uid = rtcb->group->tg_euid;
  gid = rtcb->group->tg_egid;

  /* Select the applicable permission-bit triplet */

  if (uid == inode->i_owner)
    {
      perm = (inode->i_mode >> 6) & 7;
    }
  else if (gid == inode->i_group)
    {
      perm = (inode->i_mode >> 3) & 7;
    }
  else
    {
      perm = inode->i_mode & 7;
    }

  /* Bit 2 (value 4) = read permission */

  if (((oflags & O_RDOK) != 0) && ((perm & 4) == 0))
    {
      return -EACCES;
    }

  /* Bit 1 (value 2) = write permission */

  if (((oflags & O_WROK) != 0) && ((perm & 2) == 0))
    {
      return -EACCES;
    }

#endif /* CONFIG_PSEUDOFS_ATTRIBUTES && CONFIG_SCHED_USER_IDENTITY */

  return OK;
}
