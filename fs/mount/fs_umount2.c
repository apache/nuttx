/****************************************************************************
 * fs/mount/fs_umount2.c
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

#include <sys/mount.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_umount2
 *
 * Description:
 *   nx_umount2() is similar to the standard 'umount2' interface except that
 *   is not a cancellation point and it does not modify the errno variable.
 *
 *   nx_umount2() is an internal NuttX interface and should not be called
 *   from applications.
 *
 * Returned Value:
 *   Zero is returned on success; a negated value is returned on any failure.
 *
 ****************************************************************************/

int nx_umount2(FAR const char *target, unsigned int flags)
{
  FAR struct inode *mountpt_inode;
  FAR struct inode *blkdrvr_inode = NULL;
  struct inode_search_s desc;
  int ret;

  /* Verify required pointer arguments */

  if (!target)
    {
      ret = -EFAULT;
      goto errout;
    }

  /* Find the mountpt */

  SETUP_SEARCH(&desc, target, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      goto errout_with_search;
    }

  /* Get the search results */

  mountpt_inode = desc.node;
  DEBUGASSERT(mountpt_inode != NULL);

  /* Verify that the inode is a mountpoint */

  if (!INODE_IS_MOUNTPT(mountpt_inode))
    {
      ret = -EINVAL;
      goto errout_with_mountpt;
    }

  /* Unbind the block driver from the file system (destroying any fs
   * private data.
   */

  if (!mountpt_inode->u.i_mops->unbind)
    {
      /* The filesystem does not support the unbind operation ??? */

      ret = -EINVAL;
      goto errout_with_mountpt;
    }

  /* The unbind method returns the number of references to the
   * filesystem (i.e., open files), zero if the unbind was
   * performed, or a negated error code on a failure.
   */

  /* Hold the semaphore through the unbind logic */

  ret = inode_semtake();
  if (ret < 0)
    {
      goto errout_with_mountpt;
    }

  ret = mountpt_inode->u.i_mops->unbind(mountpt_inode->i_private,
                                       &blkdrvr_inode, flags);
  if (ret < 0)
    {
      /* The inode is unhappy with the blkdrvr for some reason */

      goto errout_with_semaphore;
    }
  else if (ret > 0)
    {
      ret = -EBUSY;
      goto errout_with_semaphore;
    }

  /* Successfully unbound.  Convert the mountpoint inode to regular
   * pseudo-file inode.
   */

  mountpt_inode->i_flags  &= ~FSNODEFLAG_TYPE_MASK;
  mountpt_inode->i_private = NULL;
  mountpt_inode->u.i_mops  = NULL;

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* If the node has children, then do not delete it. */

  if (mountpt_inode->i_child != NULL)
    {
      /* Just decrement the reference count (without deleting it) */

      DEBUGASSERT(mountpt_inode->i_crefs > 0);
      mountpt_inode->i_crefs--;
      inode_semgive();
    }
  else
#endif
    {
      /* Remove the mountpoint inode from the inode tree.  The inode will
       * not be deleted yet because there is still at least reference on
       * it (from the mount)
       */

      ret = inode_remove(target);
      inode_semgive();

      /* The return value of -EBUSY is normal (in fact, it should
       * not be OK)
       */

      if (ret != OK && ret != -EBUSY)
        {
          goto errout_with_mountpt;
        }

      /* Release the mountpoint inode and any block driver inode
       * returned by the file system unbind above.  This should cause
       * the inode to be deleted (unless there are other references)
       */

      inode_release(mountpt_inode);
    }

  /* Did the unbind method return a contained block driver */

  if (blkdrvr_inode)
    {
      inode_release(blkdrvr_inode);
    }

  RELEASE_SEARCH(&desc);
  return OK;

  /* A lot of goto's!  But they make the error handling much simpler */

errout_with_semaphore:
  inode_semgive();

errout_with_mountpt:
  inode_release(mountpt_inode);
  if (blkdrvr_inode)
    {
      inode_release(blkdrvr_inode);
    }

errout_with_search:
  RELEASE_SEARCH(&desc);

errout:
  return ret;
}

/****************************************************************************
 * Name: umount2
 *
 * Description:
 *   umount() detaches the filesystem mounted at the path specified by
 *  'target.'
 *
 * Returned Value:
 *   Zero is returned on success; -1 is returned on an error and errno is
 *   set appropriately:
 *
 *   EACCES A component of a path was not searchable or mounting a read-only
 *      filesystem was attempted without giving the MS_RDONLY flag.
 *   EBUSY The target could not be unmounted because it is busy.
 *   EFAULT The pointer argument points outside the user address space.
 *
 ****************************************************************************/

int umount2(FAR const char *target, unsigned int flags)
{
  int ret;

  ret = nx_umount2(target, flags);
  if (ret < 0)
    {
      set_errno(-ret);
      ret = ERROR;
    }

  return ret;
}
