/****************************************************************************
 * fs/vfs/fs_mkdir.c
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
#include <assert.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#undef FS_HAVE_MKDIR
#if !defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_DISABLE_PSEUDOFS_OPERATIONS)
#  define FS_HAVE_MKDIR 1
#endif

#ifdef FS_HAVE_MKDIR

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mkdir
 *
 * Description:  Create a directory
 *
 ****************************************************************************/

int mkdir(const char *pathname, mode_t mode)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
  int errcode;
  int ret;

  mode &= ~getumask();

  /* Find the inode that includes this path */

  SETUP_SEARCH(&desc, pathname, false);

  ret = inode_find(&desc);
  if (ret >= 0)
    {
      /* An inode was found that includes this path and possibly refers to a
       * mountpoint.
       */

      inode = desc.node;
      DEBUGASSERT(inode != NULL);

#ifndef CONFIG_DISABLE_MOUNTPOINT
      /* Check if the inode is a valid mountpoint. */

      if (!INODE_IS_MOUNTPT(inode) || !inode->u.i_mops)
        {
          /* The inode is not a mountpoint */

          errcode = ENXIO;
          goto errout_with_inode;
        }

      /* Perform the mkdir operation using the relative path
       * at the mountpoint.
       */

      if (inode->u.i_mops->mkdir)
        {
          ret = inode->u.i_mops->mkdir(inode, desc.relpath, mode);
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

      /* Release our reference on the inode */

      inode_release(inode);
#else
      /* But mountpoints are not supported in this configuration */

      errcode = EEXIST;
      goto errout_with_inode;
#endif
    }

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  /* No inode exists that contains this path.  Create a new inode in the
   * pseudo-filesystem at this location.
   */

  else
    {
      /* Create an inode in the pseudo-filesystem at this path.
       * NOTE that the new inode will be created with a reference
       * count of zero.
       */

      ret = inode_semtake();
      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_search;
        }

      ret = inode_reserve(pathname, mode, &inode);
      inode_semgive();

      if (ret < 0)
        {
          errcode = -ret;
          goto errout_with_search;
        }
    }
#else
  else
    {
      errcode = ENXIO;
      goto errout_with_search;
    }
#endif

  /* Directory successfully created */

  RELEASE_SEARCH(&desc);
  return OK;

errout_with_inode:
  inode_release(inode);

errout_with_search:
  RELEASE_SEARCH(&desc);
  set_errno(errcode);
  return ERROR;
}

#endif /* FS_HAVE_MKDIR */
