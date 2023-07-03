/****************************************************************************
 * fs/driver/fs_mtdpartition.c
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

#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "driver/driver.h"
#include "inode/inode.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: register_mtdpartition/register_partition_with_mtd
 *
 * Description:
 *   Register a mtd partition driver inode the pseudo file system.
 *
 * Input Parameters:
 *   partition  - The path to the partition inode
 *   parent     - The parent path or mtd instance
 *   firstblock - The offset in block to the partition
 *   nblocks    - The number of block in the partition
 *
 * Returned Value:
 *   Zero on success (with the inode point in 'inode'); A negated errno
 *   value is returned on a failure (all error values returned by
 *   inode_reserve):
 *
 *   EINVAL - 'path' is invalid for this operation
 *   EEXIST - An inode already exists at 'path'
 *   ENOMEM - Failed to allocate in-memory resources for the operation
 *
 ****************************************************************************/

int register_partition_with_mtd(FAR const char *partition,
                                mode_t mode, FAR struct mtd_dev_s *parent,
                                off_t firstblock, off_t nblocks)
{
  FAR struct mtd_dev_s *part;

  /* Create the mtd partition */

  part = mtd_partition(parent, firstblock, nblocks);
  if (part == NULL)
    {
      return -EINVAL;
    }

#ifdef CONFIG_MTD_PARTITION_NAMES
  mtd_setpartitionname(part, partition);
#endif

  /* Register the mtd partition */

  return register_mtddriver(partition, part, mode, part);
}

int register_mtdpartition(FAR const char *partition,
                          mode_t mode, FAR const char *parent,
                          off_t firstblock, off_t nblocks)
{
  FAR struct inode *mtd;
  int ret;

  /* Find the mtd driver */

  ret = find_mtddriver(parent, &mtd);
  if (ret < 0)
    {
      return ret;
    }

  /* Register the mtd partition */

  ret = register_partition_with_mtd(partition, mode,
                                    mtd->u.i_mtd, firstblock, nblocks);
  inode_release(mtd);

  return ret;
}
