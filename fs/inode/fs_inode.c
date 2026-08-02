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

#ifdef CONFIG_FS_PERMISSION
/****************************************************************************
 * Name: inode_checkperm
 *
 * Description:
 *   Check 'inode' for 'amode' access against stored owner/group/mode.
 *   Applies to pseudoFS nodes and to mountpoint inodes (whose i_mode gates
 *   traversal into the mounted filesystem).  Optional mountpt_operations
 *   .permission may expose the same policy for in-volume paths; the VFS
 *   does not call that hook for mount-crossing.
 *
 ****************************************************************************/

int inode_checkperm(FAR struct inode *inode, int amode)
{
  if (inode == NULL)
    {
      return OK;
    }

  return fs_checkmode(inode->i_owner, inode->i_group, inode->i_mode, amode);
}

/****************************************************************************
 * Name: inode_checkpathperm
 *
 * Description:
 *   Require X_OK on every ancestor of 'inode', and on 'inode' itself when
 *   it is a directory or mountpoint that must be traversed.  If 'amode' is
 *   non-zero, also require that access on 'inode'.  Takes the inode tree
 *   read lock unless INODE_CHECK_LOCKED is set in 'flags'.
 *
 ****************************************************************************/

int inode_checkpathperm(FAR struct inode *inode, int amode, int flags)
{
  FAR struct inode *node;
  int locked = (flags & INODE_CHECK_LOCKED) != 0;
  int ret;

  if (inode == NULL)
    {
      return OK;
    }

  if (!locked)
    {
      inode_rlock();
    }

  if (INODE_IS_PSEUDODIR(inode)
#ifndef CONFIG_DISABLE_MOUNTPOINT
      || INODE_IS_MOUNTPT(inode)
#endif
     )
    {
      ret = inode_checkperm(inode, X_OK);
      if (ret < 0)
        {
          goto errout;
        }
    }

  for (node = inode->i_parent; node != NULL; node = node->i_parent)
    {
      ret = inode_checkperm(node, X_OK);
      if (ret < 0)
        {
          goto errout;
        }
    }

  if (amode != 0)
    {
      ret = inode_checkperm(inode, amode);
      if (ret < 0)
        {
          goto errout;
        }
    }

  ret = OK;

errout:
  if (!locked)
    {
      inode_runlock();
    }

  return ret;
}
#endif /* CONFIG_FS_PERMISSION */

/****************************************************************************
 * Name: inode_checkopenperm
 *
 * Description:
 *   Validate open access to 'inode' for 'oflags'.  Checks driver operation
 *   support, then mode bits for non-mountpoint inodes.  Mountpoints are not
 *   mode-checked here for R/W (that would confuse directory bits with file
 *   open modes); callers use inode_checkpathperm() for traversal.
 *
 ****************************************************************************/

int inode_checkopenperm(FAR struct inode *inode, int oflags)
{
  FAR const struct file_operations *ops;

#ifndef CONFIG_DISABLE_MOUNTPOINT
  /* Mountpoints: only verify that open exists.  Path search / DAC for the
   * mount directory is handled by inode_checkpathperm(); per-file DAC is
   * the filesystem's responsibility.
   */

  if (INODE_IS_MOUNTPT(inode))
    {
      if (inode->u.i_mops == NULL || inode->u.i_mops->open == NULL)
        {
          return -ENXIO;
        }

      return OK;
    }
#endif

  if (INODE_IS_NAMEDSEM(inode))
    {
      return inode_checkperm(inode, R_OK | W_OK);
    }

  if (INODE_IS_MQUEUE(inode) || INODE_IS_PSEUDODIR(inode))
    {
      return inode_checkperm(inode, fs_open_amode(oflags));
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

  return inode_checkperm(inode, fs_open_amode(oflags));
}
