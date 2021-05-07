/****************************************************************************
 * fs/driver/fs_openblockdriver.c
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

#include <debug.h>
#include <errno.h>
#include <nuttx/fs/fs.h>

#include "inode/inode.h"
#include "driver/driver.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: open_blockdriver
 *
 * Description:
 *   Return the inode of the block driver specified by 'pathname'
 *
 * Input Parameters:
 *   pathname - the full path to the block driver to be opened
 *   mountflags - if MS_RDONLY is not set, then driver must support write
 *     operations (see include/sys/mount.h)
 *   ppinode - address of the location to return the inode reference
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   EINVAL  - pathname or pinode is NULL
 *   ENOENT  - No block driver of this name is registered
 *   ENOTBLK - The inode associated with the pathname is not a block driver
 *   EACCESS - The MS_RDONLY option was not set but this driver does not
 *     support write access
 *
 ****************************************************************************/

int open_blockdriver(FAR const char *pathname, int mountflags,
                     FAR struct inode **ppinode)
{
  FAR struct inode *inode;
  int ret;

  /* Minimal sanity checks */

#ifdef CONFIG_DEBUG_FEATURES
  if (!ppinode)
    {
      return -EINVAL;
    }
#endif

  /* Find the inode associated with this block driver name.  find_blockdriver
   * will perform all additional error checking.
   */

  ret = find_blockdriver(pathname, mountflags, &inode);
  if (ret < 0)
    {
#ifdef CONFIG_MTD
      /* Not block device, mtd device? let's try it. */

      return mtd_proxy(pathname, mountflags, ppinode);
#else
      ferr("ERROR: Failed to file %s block driver\n", pathname);
      return ret;
#endif
    }

  /* Open the block driver.  Note that no mutually exclusive access
   * to the driver is enforced here.  That must be done in the driver
   * if needed.
   */

  if (inode->u.i_bops->open)
    {
      ret = inode->u.i_bops->open(inode);
      if (ret < 0)
        {
          ferr("ERROR: %s driver open failed\n", pathname);
          inode_release(inode);
          return ret;
        }
    }

  *ppinode = inode;
  return OK;
}
