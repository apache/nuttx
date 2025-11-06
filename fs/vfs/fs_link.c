/****************************************************************************
 * fs/vfs/fs_link.c
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

#include <sys/types.h>
#include <sys/stat.h>
#include <stdbool.h>
#include <unistd.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/lib/lib.h>
#include <nuttx/fs/fs.h>
#include <nuttx/atomic.h>

#include "inode/inode.h"
#include "vfs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_FS_LINKS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: link
 *
 * Description:
 *  The link() function creates a new hard link for the existing file
 *  specified by 'path1'. The new link is created at the location specified
 *  by 'path2'.
 *
 *  This implementation currently only supports creating hard links within
 *  the NuttX pseudo-filesystem (pseudofs). Hard links for files within
 *  mounted filesystems are not yet supported. In the future, support for
 *  hard links in mounted filesystems may be added.
 *
 * Input Parameters:
 *   path1 - Points to a pathname naming an existing file.
 *   path2 - Points to a pathname naming the new directory entry to be
 *           created.
 *
 * Returned Value:
 *   On success, zero (OK) is returned.  Otherwise, -1 (ERROR) is returned
 *   the errno variable is set appropriately.
 *
 ****************************************************************************/

int link(FAR const char *path1, FAR const char *path2)
{
  struct inode_search_s desc_path1;
  struct inode_search_s desc_path2;
  FAR struct inode *target = NULL;
  FAR struct inode *newinode = NULL;
  int errcode;
  int ret;

  if (path1 == NULL || path2 == NULL)
    {
      errcode = EINVAL;
      goto errout;
    }

  if (*path1 == '\0' || *path2 == '\0')
    {
      errcode = ENOENT;
      goto errout;
    }

  SETUP_SEARCH(&desc_path1, path1, false);
  ret = inode_find(&desc_path1);
  if (ret < 0)
    {
      errcode = -ret;
      goto errout_with_search_path1;
    }

  target = desc_path1.node;

  if (INODE_GET_NLINK(target) >= _POSIX_LINK_MAX)
    {
      /* Too many links to the target inode */

      errcode = EMLINK;
      goto errout_with_target;
    }

  /* Check that no inode exists at the 'path2' and that the path up to
   * 'path2' does not lie on a mounted volume.
   */

  SETUP_SEARCH(&desc_path2, path2, true);

  ret = inode_find(&desc_path2);
  if (ret >= 0)
    {
      newinode = desc_path2.node;

      /* Something exists at the path2 where we are trying to create the
       * link.
       */

#ifndef CONFIG_DISABLE_MOUNTPOINT
      /* Check if the inode is a mountpoint. */

      DEBUGASSERT(newinode != NULL);
      if (INODE_IS_MOUNTPT(newinode))
        {
          /* Check if path1 and path2 are on the same mountpoint */

          if (newinode != target)
            {
              errcode = EXDEV;
              goto errout_with_newinode;
            }

          if (target->u.i_mops && target->u.i_mops->link)
            {
              /* Perform the link operation using the relative path at the
               * mountpoint.
               */

              ret = target->u.i_mops->link(target, desc_path1.relpath,
                                           desc_path2.relpath);
              if (ret < 0)
                {
                  errcode = -ret;
                  goto errout_with_newinode;
                }
            }
          else
            {
              /* Hard links within this type of fs are not supported */

              errcode = ENOSYS;
              goto errout_with_newinode;
            }
        }
      else
#endif
        {
          /* A node already exists in the pseudofs at 'path2' */

          errcode = EEXIST;
          goto errout_with_newinode;
        }
    }

  /* No inode exists that contains this path.  Create a new inode in the
   * pseudo-filesystem at this location.
   */

  else
    {
      /* Cannot link between pseudofs and other mountpoints */

      if (INODE_IS_MOUNTPT(target))
        {
          errcode = EXDEV;
          goto errout_with_target;
        }

      /* Create an inode in the pseudo-filesystem at this path. */

      inode_lock();
      ret = inode_reserve(path2, 0777, &newinode);

      if (ret >= 0)
        {
          /* Initialize the inode */

          INODE_SET_HARDLINK(newinode);
          newinode->i_private = target;
          atomic_add(&target->i_crefs, INODE_NLINK_INC);
        }

      inode_unlock();
      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_newinode;
        }
    }

  /* Hard link successfully created */

  RELEASE_SEARCH(&desc_path1);
  RELEASE_SEARCH(&desc_path2);
  inode_release(target);

#ifdef CONFIG_FS_NOTIFY
  notify_create(path2);
#endif
  return OK;

errout_with_newinode:
  inode_release(newinode);
  RELEASE_SEARCH(&desc_path2);
errout_with_target:
  inode_release(target);
errout_with_search_path1:
  RELEASE_SEARCH(&desc_path1);

errout:
  set_errno(errcode);
  return ERROR;
}

#else /* CONFIG_FS_LINKS */

/****************************************************************************
 * Name: link
 *
 * Description:
 *   When CONFIG_FS_LINKS is disabled, link() is not supported.  The symbol
 *   is still provided so that applications referencing link() continue to
 *   link, but the call always fails with ENOSYS.
 *
 ****************************************************************************/

int link(FAR const char *path1, FAR const char *path2)
{
  UNUSED(path1);
  UNUSED(path2);

  set_errno(ENOSYS);
  return ERROR;
}

#endif /* CONFIG_FS_LINKS */
