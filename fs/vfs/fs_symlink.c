/****************************************************************************
 * fs/vfs/fs_symlink.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PSEUDOFS_SOFTLINKS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: symlink
 *
 * Description:
 *   The symlink() function will create a new link (directory entry) for the
 *   existing file, path2.  This implementation is simplied for use with
 *   NuttX in these ways:
 *
 *   - Links may be created only within the NuttX top-level, pseudo file
 *     system.  No file system currently supported by NuttX provides
 *     symbolic links.
 *   - For the same reason, only soft links are implemented.
 *   - File privileges are ignored.
 *   - c_time is not updated.
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

int symlink(FAR const char *path1, FAR const char *path2)
{
  struct inode_search_s desc;
  FAR struct inode *inode = NULL;
  int errcode;
  int ret;

  if (path1 == NULL)
    {
      errcode = EINVAL;
      goto errout;
    }

  /* Check that no inode exists at the 'path2' and that the path up to
   * 'path2' does not lie on a mounted volume.
   */

  SETUP_SEARCH(&desc, path2, false);

  ret = inode_find(&desc);
  if (ret >= 0)
    {
      /* Something exists at the path2 where we are trying to create the
       * link.
       */

#ifndef CONFIG_DISABLE_MOUNTPOINT
      /* Check if the inode is a mountpoint. */

      DEBUGASSERT(desc.node != NULL);
      if (INODE_IS_MOUNTPT(desc.node))
        {
          /* Symbolic links within the mounted volume are not supported */

          errcode = ENOSYS;
        }
      else
#endif
        {
          /* A node already exists in the pseudofs at 'path1' */

          errcode = EEXIST;
        }

      goto errout_with_inode;
    }

  /* No inode exists that contains this path.  Create a new inode in the
   * pseudo-filesystem at this location.
   */

  else
    {
      /* Copy path1 */

      FAR char *newpath2 = strdup(path1);
      if (newpath2 == NULL)
        {
          errcode = ENOMEM;
          goto errout_with_search;
        }

      /* Create an inode in the pseudo-filesystem at this path.
       * NOTE that the new inode will be created with a reference
       * count of zero.
       */

      ret = inode_lock();
      if (ret < 0)
        {
          kmm_free(newpath2);
          errcode = -ret;
          goto errout_with_search;
        }

      ret = inode_reserve(path2, 0777, &inode);
      inode_unlock();

      if (ret < 0)
        {
          kmm_free(newpath2);
          errcode = -ret;
          goto errout_with_search;
        }

      /* Initialize the inode */

      INODE_SET_SOFTLINK(inode);
      inode->u.i_link = newpath2;
    }

  /* Symbolic link successfully created */

  RELEASE_SEARCH(&desc);
  return OK;

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);

errout:
  set_errno(errcode);
  return ERROR;
}

#endif /* CONFIG_PSEUDOFS_SOFTLINKS */
