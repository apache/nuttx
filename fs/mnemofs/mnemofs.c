/****************************************************************************
 * fs/mnemofs/mnemofs.c
 * mnemofs: Filesystem for NAND Flash storage devices.
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

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <math.h>
#include <stdio.h>

#include "mnemofs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mnemofs_bind(FAR struct inode *driver, FAR const void *data,
                        FAR void** handle);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Name: g_mnemofs_operations
 *
 * Description:
 *  The global list of VFS methods implemented by mnemofs
 *
 ****************************************************************************/

const struct mountpt_operations g_mnemofs_operations =
{
  NULL,         /* open */
  NULL,         /* close */
  NULL,         /* read */
  NULL,         /* write */
  NULL,         /* seek */
  NULL,         /* ioctl */
  NULL,         /* mmap */
  NULL,         /* truncate */
  NULL,         /* poll */

  NULL,         /* sync */
  NULL,         /* dup */
  NULL,         /* fstat */
  NULL,         /* fchstat */

  NULL,         /* opendir */
  NULL,         /* closedir */
  NULL,         /* readdir */
  NULL,         /* rewinddir */

  mnemofs_bind, /* bind */
  NULL,         /* unbind */
  NULL,         /* statfs */

  NULL,         /* unlink */
  NULL,         /* mkdir */
  NULL,         /* rmdir */
  NULL,         /* rename */
  NULL,         /* stat */
  NULL          /* chstat */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mnemofs_bind
 *
 * Description:
 *   Mounts the file system.
 *
 * Input Parameters:
 *   driver - MTD driver
 *   data   - Mount options
 *   handle - To be updated with file system information.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

static int mnemofs_bind(FAR struct inode *driver, FAR const void *data,
                        FAR void** handle)
{
  int ret = OK;
  struct mfs_sb_info *sb = NULL;

  sb = kmm_zalloc(sizeof(*sb));
  if (!sb)
    {
      ret = -ENOMEM;
      goto errout;
    }

  if (driver && INODE_IS_MTD(driver))
    {
      ret = MTD_IOCTL(driver->u.i_mtd, MTDIOC_GEOMETRY,
                      (unsigned long) &sb->geo);
    }
  else
    {
      ret = -ENODEV;
      goto errout_with_sb;
    }

  nxmutex_init(&sb->fs_lock);

  sb->drv           = driver;
  sb->pg_sz         = sb->geo.blocksize;
  sb->blk_sz        = sb->geo.erasesize;
  sb->n_blks        = sb->geo.neraseblocks;
  sb->pg_in_blk     = sb->blk_sz / sb->pg_sz;
  sb->j_nblks       = CONFIG_MNEMOFS_JOURNAL_NBLKS;
  sb->log_blk_sz    = log2(sb->blk_sz);
  sb->log_pg_sz     = log2(sb->pg_sz);
  sb->log_pg_in_blk = log2(sb->pg_in_blk);
  sb->master_node   = 0; /* Placeholder code for testing. */

  ret = mfs_jrnl_fmt(sb);
  if (ret != OK)
    {
      goto errout_with_sb;
    }

  handle = (FAR void *)sb;

  return ret;

errout_with_sb:
  kmm_free(sb);

errout:
  return ret;
}
