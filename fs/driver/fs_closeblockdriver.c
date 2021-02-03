/****************************************************************************
 * fs/driver/fs_closeblockdriver.c
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: close_blockdriver
 *
 * Description:
 *   Call the close method and release the inode
 *
 * Input Parameters:
 *   inode - reference to the inode of a block driver opened by
 *           open_blockdriver
 *
 * Returned Value:
 *   Returns zero on success or a negated errno on failure:
 *
 *   EINVAL  - inode is NULL
 *   ENOTBLK - The inode is not a block driver
 *
 ****************************************************************************/

int close_blockdriver(FAR struct inode *inode)
{
  int ret = 0; /* Assume success */

  /* Sanity checks */

#ifdef CONFIG_DEBUG_FEATURES
  if (!inode || !inode->u.i_bops)
    {
      ret = -EINVAL;
      goto errout;
    }
#endif

  /* Verify that the inode is a block driver. */

  if (!INODE_IS_BLOCK(inode))
    {
      ferr("ERROR: inode is not a block driver\n");
      ret = -ENOTBLK;
      goto errout;
    }

  /* Close the block driver.  Not that no mutually exclusive access
   * to the driver is enforced here.  That must be done in the driver
   * if needed.
   */

  if (inode->u.i_bops->close)
    {
      ret = inode->u.i_bops->close(inode);
    }

  /* Then release the reference on the inode */

  inode_release(inode);

errout:
  return ret;
}
