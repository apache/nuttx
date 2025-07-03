/****************************************************************************
 * fs/driver/fs_mtdproxy.c
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
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <fcntl.h>
#include <nuttx/lib/lib.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mutex.h>
#include "driver/driver.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtd_block_proxy
 *
 * Description:
 *   Create a temporary block driver using drivers/mtd/ftl to mediate block
 *   oriented accessed to the mtd driver.
 *
 * Input Parameters:
 *   mtddev  - The path to the mtd driver
 *   mountflags - if MS_RDONLY is not set, then driver must support write
 *     operations (see include/sys/mount.h)
 *   ppinode - address of the location to return the inode reference
 *
 * Returned Value:
 *   If zero, non-zero inode pointer is returned on success.  This
 *   is the inode pointer of the nameless block driver that mediates
 *   accesses to the mtd driver. A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

int mtd_block_proxy(FAR const char *mtddev, int mountflags,
                    FAR struct inode **ppinode)
{
  char blkdev[16];
  FAR struct inode *mtd;
  int ret;

  /* Create a unique temporary file name for the block device */

  ret = unique_dev("tmpb", blkdev, sizeof(blkdev));
  if (ret != OK)
    {
      ferr("ERROR: Failed to create temporary device name\n");
      return ret;
    }

  /* Wrap the mtd driver with an instance of the ftl driver */

  ret = find_mtddriver(mtddev, &mtd);
  if (ret < 0)
    {
      ferr("ERROR: Failed to find %s mtd driver\n", mtddev);
      return ret;
    }

  ret = ftl_initialize_to_block(blkdev, mtd->u.i_mtd);
  inode_release(mtd);
  if (ret < 0)
    {
      ferr("ERROR: ftl_initialize_to_block(%s, %s) failed: %d\n",
           mtddev, blkdev, ret);
      return ret;
    }

  /* Open the newly created block driver */

  ret = open_blockdriver(blkdev, mountflags, ppinode);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open %s: %d\n", blkdev, ret);
      nx_unlink(blkdev);
      return ret;
    }

  /* Unlink and free the block device name.  The driver instance will
   * persist, provided that CONFIG_DISABLE_PSEUDOFS_OPERATIONS=y (otherwise,
   * we have a problem here!)
   */

  ret = nx_unlink(blkdev);
  if (ret < 0)
    {
      ferr("ERROR: Failed to unlink %s: %d\n", blkdev, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mtd_char_proxy
 *
 * Description:
 *   Create a temporary char driver using drivers/mtd/ftl to mediate
 *   character oriented accessed to the mtd driver.
 *
 * Input Parameters:
 *   mtddev - The path to the mtd driver
 *   oflags - Character driver open flags
 *   filep  - The caller provided location in which to return the 'struct
 *            file' instance.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  On failure, a negated errno value is
 *   returned.
 *
 ****************************************************************************/

int mtd_char_proxy(FAR const char *mtddev, int oflags,
                   FAR struct file *filep)
{
  char chardev[32];
  FAR struct inode *mtd;
  int ret;

  DEBUGASSERT(mtddev);

  /* Create a unique temporary file name for the character device */

  ret = unique_dev("tmpmtdc", chardev, sizeof(chardev));
  if (ret != OK)
    {
      ferr("ERROR: Failed to create temporary mtd char name\n");
      return ret;
    }

  ret = find_mtddriver(mtddev, &mtd);
  if (ret < 0)
    {
      ferr("ERROR: Failed to find %s mtd driver\n", mtddev);
      return ret;
    }

  /* Wrap the mtd driver with an instance of the ftl driver */

  ret = ftl_initialize_by_path(chardev, mtd->u.i_mtd);
  inode_release(mtd);
  if (ret < 0)
    {
      ferr("ERROR: ftl_initialize_by_path (%s, %s) failed: %d\n",
           mtddev, chardev, ret);
      return ret;
    }

  /* Open the newly created driver */

  oflags &= ~(O_CREAT | O_EXCL | O_APPEND | O_TRUNC);
  ret = file_open(filep, chardev, oflags);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open %s: %d\n", chardev, ret);
      nx_unlink(chardev);
      return ret;
    }

  /* Unlink the device name.  The driver instance will persist,
   * provided that CONFIG_DISABLE_PSEUDOFS_OPERATIONS=y (otherwise, we have
   * a problem here!)
   */

  ret = nx_unlink(chardev);
  if (ret < 0)
    {
      ferr("ERROR: Failed to unlink temp mtd char %s: %d\n", chardev, ret);
      file_close(filep);
    }

  return ret;
}
