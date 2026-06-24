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
#include <unistd.h>

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
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_FS_PERMISSION
/****************************************************************************
 * Name: fs_checkmode
 *
 * Description:
 *   Test the calling task's effective credentials against the owner, group,
 *   and mode of a file or directory.  Kernel threads always pass.
 *
 ****************************************************************************/

int fs_checkmode(uid_t owner, gid_t group, mode_t mode, int amode)
{
  FAR struct tcb_s *rtcb;
  mode_t perm;
  uid_t uid;
  gid_t gid;

  rtcb = nxsched_self();
  if ((rtcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL)
    {
      return OK;
    }

  DEBUGASSERT(rtcb->group != NULL);
  uid = rtcb->group->tg_euid;
  gid = rtcb->group->tg_egid;

  if (uid == owner)
    {
      perm = (mode >> 6) & 7;
    }
  else if (gid == group)
    {
      perm = (mode >> 3) & 7;
    }
  else
    {
      perm = mode & 7;
    }

  if ((amode & perm) != amode)
    {
      return -EACCES;
    }

  return OK;
}

/****************************************************************************
 * Name: fs_open_amode
 *
 * Description:
 *   Map open flags to a permission access mode bitmask.
 *
 ****************************************************************************/

int fs_open_amode(int oflags)
{
  switch (oflags & O_ACCMODE)
    {
      case O_RDONLY:
        return R_OK;

      case O_WRONLY:
        return W_OK;

      case O_RDWR:
        return R_OK | W_OK;

      default:
        return 0;
    }
}

/****************************************************************************
 * Name: fs_checkopenperm
 *
 * Description:
 *   Test open access for the calling task against owner, group, and mode.
 *
 ****************************************************************************/

int fs_checkopenperm(uid_t owner, gid_t group, mode_t mode, int oflags)
{
  return fs_checkmode(owner, group, mode, fs_open_amode(oflags));
}
#endif /* CONFIG_FS_PERMISSION */

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
 *   Check 'inode' for 'amode' access on pseudo-filesystem inodes.
 *   NULL 'inode' (root) and mountpoints are exempt.
 *
 * Input Parameters:
 *   inode - Inode to check, or NULL for a root-level path
 *   amode - Access mode bitmask (R_OK / W_OK / X_OK)
 *
 * Returned Value:
 *   Zero (OK) on success, or -EACCES if permission is denied.
 *
 ****************************************************************************/

int inode_checkperm(FAR struct inode *inode, int amode)
{
#ifdef CONFIG_FS_PERMISSION

  if (inode == NULL)
    {
      return OK;
    }

  if (INODE_IS_MOUNTPT(inode))
    {
      return OK;
    }

  return fs_checkmode(inode->i_owner, inode->i_group, inode->i_mode, amode);

#else
  return OK;
#endif /* CONFIG_FS_PERMISSION */
}

/****************************************************************************
 * Name: inode_checkopenperm
 *
 * Description:
 *   Validate open access to 'inode' for 'oflags'.  Checks driver operation
 *   support, then pseudo-filesystem mode bits when enabled.  Mountpoints
 *   are exempt from mode checks.
 *
 * Input Parameters:
 *   inode  - The inode to check
 *   oflags - Open flags (O_RDONLY / O_WRONLY / O_RDWR)
 *
 * Returned Value:
 *   Zero (OK) on success, or a negated errno on failure.
 *
 ****************************************************************************/

int inode_checkopenperm(FAR struct inode *inode, int oflags)
{
  FAR const struct file_operations *ops;

  if (INODE_IS_PSEUDODIR(inode))
    {
#ifdef CONFIG_FS_PERMISSION
      return inode_checkperm(inode, fs_open_amode(oflags));
#else
      return OK;
#endif
    }

  ops = inode->u.i_ops;
  if (ops == NULL)
    {
      return -ENXIO;
    }

  if (((oflags & O_ACCMODE) != O_WRONLY &&
       !ops->readv && !ops->read && !ops->ioctl) ||
      ((oflags & O_ACCMODE) != O_RDONLY &&
       !ops->writev && !ops->write && !ops->ioctl))
    {
      return -EACCES;
    }

#ifdef CONFIG_FS_PERMISSION

  return inode_checkperm(inode, fs_open_amode(oflags));

#else
  return OK;
#endif /* CONFIG_FS_PERMISSION */
}
