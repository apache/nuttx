/****************************************************************************
 * fs/vfs/fs_unlink.c
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

#include <stdbool.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef FS_HAVE_UNLINK
#if !defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_DISABLE_PSEUDOFS_OPERATIONS)
#  define FS_HAVE_UNLINK 1
#endif

#ifdef FS_HAVE_UNLINK

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nx_unlink
 *
 * Description:  Remove a file managed a mountpoint
 *
 ****************************************************************************/

int nx_unlink(FAR const char *pathname)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
  int ret;

  /* Get an inode for this file (without deference the final node in the path
   * which may be a symbolic link)
   */

  SETUP_SEARCH(&desc, pathname, true);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* There is no inode that includes in this path */

      goto errout_with_search;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

#ifndef CONFIG_DISABLE_MOUNTPOINT
  /* Check if the inode is a valid mountpoint. */

  if (INODE_IS_MOUNTPT(inode) && inode->u.i_mops)
    {
      /* Perform the unlink operation using the relative path at the
       * mountpoint.
       */

      if (inode->u.i_mops->unlink)
        {
          ret = inode->u.i_mops->unlink(inode, desc.relpath);
          if (ret < 0)
            {
              goto errout_with_inode;
            }
        }
      else
        {
          ret = -ENOSYS;
          goto errout_with_inode;
        }
    }
  else
#endif

    {
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
      /* Refuse to unlink the inode if it has children.  I.e., if it is
       * functioning as a directory and the directory is not empty.
       */

      if (inode->i_child != NULL)
        {
          ret = -ENOTEMPTY;
          goto errout_with_inode;
        }

      /* Notify the driver that it has been unlinked.  If there are no
       * open references to the driver instance, then the driver should
       * release all resources because it is no longer accessible.
       */

      if (INODE_IS_DRIVER(inode) && inode->u.i_ops->unlink)
        {
          /* Notify the character driver that it has been unlinked */

          ret = inode->u.i_ops->unlink(inode);
          if (ret < 0)
            {
              goto errout_with_inode;
            }
        }
#ifndef CONFIG_DISABLE_MOUNTPOINT
      else if (INODE_IS_BLOCK(inode) && inode->u.i_bops->unlink)
        {
          /* Notify the block driver that it has been unlinked */

          ret = inode->u.i_bops->unlink(inode);
          if (ret < 0)
            {
              goto errout_with_inode;
            }
        }
#endif
#ifdef CONFIG_PSEUDOFS_SOFTLINKS
      else if (INODE_IS_PSEUDODIR(inode) || INODE_IS_SOFTLINK(inode))
#else
      else if (INODE_IS_PSEUDODIR(inode))
#endif
        {
          /* If this is a "dangling" pseudo-file node
           * (i.e., it has no operations) or a soft link,
           * then rm should remove the node.
           */
        }
      else
        {
          ret = -ENXIO;
          goto errout_with_inode;
        }

      /* Remove the old inode.  Because we hold a reference count on the
       * inode, it will not be deleted now.  It will be deleted when all
       * of the references to the inode have been released (perhaps
       * when inode_release() is called below).  inode_remove() will
       * return -EBUSY to indicate that the inode was not deleted now.
       */

      ret = inode_lock();
      if (ret < 0)
        {
          goto errout_with_inode;
        }

      ret = inode_remove(pathname);
      inode_unlock();

      if (ret < 0 && ret != -EBUSY)
        {
          goto errout_with_inode;
        }
#endif
    }

  /* Successfully unlinked */

  inode_release(inode);
  RELEASE_SEARCH(&desc);
  return OK;

#if !defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_DISABLE_PSEUDOFS_OPERATIONS)
errout_with_inode:
  inode_release(inode);
#endif

errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}

/****************************************************************************
 * Name: unlink
 *
 * Description:  Remove a file managed a mountpoint
 *
 ****************************************************************************/

int unlink(FAR const char *pathname)
{
  int ret;

  ret = nx_unlink(pathname);
  if (ret < 0)
    {
      set_errno(-ret);
      return ERROR;
    }

  return OK;
}

#endif /* FS_HAVE_UNLINK */
