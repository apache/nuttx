/****************************************************************************
 * fs/driver/fs_findblockdriver.c
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
#include <sys/mount.h>
#include <stdbool.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "driver/driver.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: find_blockdriver
 *
 * Description:
 *   Return the inode of the block driver specified by 'pathname'
 *
 * Input Parameters:
 *   pathname   - The full path to the block driver to be located
 *   mountflags - If MS_RDONLY is not set, then driver must support write
 *                operations (see include/sys/mount.h)
 *   ppinode    - Address of the location to return the inode reference
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   ENOENT  - No block driver of this name is registered
 *   ENOTBLK - The inode associated with the pathname is not a block driver
 *   EACCESS - The MS_RDONLY option was not set but this driver does not
 *             support write access
 *
 ****************************************************************************/

int find_blockdriver(FAR const char *pathname, int mountflags,
                     FAR struct inode **ppinode)
{
  struct inode_search_s desc;
  FAR struct inode *inode;
  int ret = 0; /* Assume success */

  DEBUGASSERT(pathname != NULL || ppinode != NULL);

  finfo("pathname=\"%s\"\n", pathname);

  /* Find the inode registered with this pathname */

  SETUP_SEARCH(&desc, pathname, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      ferr("ERROR: Failed to find %s\n", pathname);
      ret = -ENOENT;
      goto errout_with_search;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* Verify that the inode is a block driver. */

  if (!INODE_IS_BLOCK(inode))
    {
#ifdef CONFIG_MTD
      if (INODE_IS_MTD(inode))
        {
          finfo("%s is a MTD\n", pathname);
        }
      else
#endif
        {
          ferr("ERROR: %s is not a block driver\n", pathname);
        }

      ret = -ENOTBLK;
      goto errout_with_inode;
    }

  /* Make sure that the inode supports the requested access */

  if (!inode->u.i_bops || !inode->u.i_bops->read ||
      (!inode->u.i_bops->write && (mountflags & MS_RDONLY) == 0))
    {
      ferr("ERROR: %s does not support requested access\n", pathname);
      ret = -EACCES;
      goto errout_with_inode;
    }

  *ppinode = inode;
  RELEASE_SEARCH(&desc);
  return OK;

errout_with_inode:
  inode_release(inode);
errout_with_search:
  RELEASE_SEARCH(&desc);
  return ret;
}
