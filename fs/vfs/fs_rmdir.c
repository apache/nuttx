/****************************************************************************
 * fs/vfs/fs_rmdir.c
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

#undef FS_HAVE_RMDIR
#if !defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_DISABLE_PSEUDOFS_OPERATIONS)
#  define FS_HAVE_RMDIR 1
#endif

#ifdef FS_HAVE_RMDIR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rmdir
 *
 * Description:  Remove a file managed a mountpoint
 *
 ****************************************************************************/

int rmdir(FAR const char *pathname)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
  int errcode;
  int ret;

  /* Get an inode for the directory (or for the mountpoint containing the
   * directory).  inode_find() automatically increments the reference count
   * on the inode if one is found.
   */

  SETUP_SEARCH(&desc, pathname, true);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* There is no inode that includes in this path */

      errcode = -ret;
      goto errout_with_search;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

#ifndef CONFIG_DISABLE_MOUNTPOINT
  /* Check if the inode is a valid mountpoint. */

  if (INODE_IS_MOUNTPT(inode) && inode->u.i_mops)
    {
      /* Perform the rmdir operation using the relative path
       * from the mountpoint.
       */

      if (inode->u.i_mops->rmdir)
        {
          ret = inode->u.i_mops->rmdir(inode, desc.relpath);
          if (ret < 0)
            {
              errcode = -ret;
              goto errout_with_inode;
            }
        }
      else
        {
          errcode = ENOSYS;
          goto errout_with_inode;
        }
    }
  else
#endif

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* If this is a "dangling" pseudo-directory node (i.e., it has no
   * operations) then rmdir should remove the node.
   */

  if (!inode->u.i_ops)
    {
      /* If the directory inode has children, however, then it cannot be
       * removed.
       */

      if (inode->i_child)
        {
          errcode = ENOTEMPTY;
          goto errout_with_inode;
        }

      /* Remove the inode.  NOTE: Because we hold a reference count on the
       * inode, it will not be deleted now.  But probably when
       * inode_release() is called below.  inode_remove should return
       * -EBUSY to indicate that the inode was not deleted now.
       */

      ret = inode_semtake();
      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_inode;
        }

      ret = inode_remove(pathname);
      inode_semgive();

      if (ret < 0 && ret != -EBUSY)
        {
          errcode = -ret;
          goto errout_with_inode;
        }
    }
  else
    {
      errcode = ENOTDIR;
      goto errout_with_inode;
    }
#else
    {
      errcode = ENXIO;
      goto errout_with_inode;
    }
#endif

  /* Successfully removed the directory */

  inode_release(inode);
  RELEASE_SEARCH(&desc);
  return OK;

errout_with_inode:
  inode_release(inode);
errout_with_search:
  RELEASE_SEARCH(&desc);
  set_errno(errcode);
  return ERROR;
}

#endif /* FS_HAVE_RMDIR */
