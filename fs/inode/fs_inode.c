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

/****************************************************************************
 * Name: _inode_checkmode
 *
 * Description:
 *   Test effective credentials against 'inode' for 'amode' access.
 *   Kernel threads always pass.
 *
 * Returned Value:
 *   Zero (OK) or -EACCES.
 *
 ****************************************************************************/

#if defined(CONFIG_PSEUDOFS_ATTRIBUTES) && defined(CONFIG_SCHED_USER_IDENTITY)
static int _inode_checkmode(FAR struct inode *inode, int amode)
{
  FAR struct tcb_s *rtcb;
  mode_t perm;
  uid_t uid;
  gid_t gid;

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

  /* Every requested bit must be present in the selected triplet */

  if ((amode & perm) != amode)
    {
      return -EACCES;
    }

  return OK;
}
#endif /* CONFIG_PSEUDOFS_ATTRIBUTES && CONFIG_SCHED_USER_IDENTITY */

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

int inode_checkperm(FAR struct inode *inode, int oflags)
{
#if defined(CONFIG_PSEUDOFS_ATTRIBUTES) && defined(CONFIG_SCHED_USER_IDENTITY)
  int amode = 0;
#endif
  FAR const struct file_operations *ops;

#if defined(CONFIG_PSEUDOFS_ATTRIBUTES) && defined(CONFIG_SCHED_USER_IDENTITY)
  if ((oflags & O_RDOK) != 0)
    {
      amode |= R_OK;
    }

  if ((oflags & O_WROK) != 0)
    {
      amode |= W_OK;
    }
#endif

  if (INODE_IS_PSEUDODIR(inode))
    {
#if defined(CONFIG_PSEUDOFS_ATTRIBUTES) && defined(CONFIG_SCHED_USER_IDENTITY)
      return _inode_checkmode(inode, amode);
#else
      return OK;
#endif
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

  if (INODE_IS_MOUNTPT(inode))
    {
      return OK;
    }

  return _inode_checkmode(inode, amode);

#endif /* CONFIG_PSEUDOFS_ATTRIBUTES && CONFIG_SCHED_USER_IDENTITY */

  return OK;
}

/****************************************************************************
 * Name: inode_checkdirperm
 *
 * Description:
 *   Check parent directory 'dir' for 'amode' access on pseudo-filesystem
 *   inodes.  NULL 'dir' (root) and mountpoints are exempt.
 *
 * Input Parameters:
 *   dir   - Parent directory inode, or NULL for a root-level path
 *   amode - Access mode bitmask (R_OK / W_OK / X_OK)
 *
 * Returned Value:
 *   Zero (OK) on success, or -EACCES if permission is denied.
 *
 ****************************************************************************/

int inode_checkdirperm(FAR struct inode *dir, int amode)
{
#if defined(CONFIG_PSEUDOFS_ATTRIBUTES) && defined(CONFIG_SCHED_USER_IDENTITY)

  if (dir == NULL)
    {
      return OK;
    }

  if (INODE_IS_MOUNTPT(dir))
    {
      return OK;
    }

  return _inode_checkmode(dir, amode);

#else
  return OK;
#endif /* CONFIG_PSEUDOFS_ATTRIBUTES && CONFIG_SCHED_USER_IDENTITY */
}
