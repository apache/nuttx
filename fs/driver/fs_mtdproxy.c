/****************************************************************************
 * fs/driver/fs_mtdproxy.c
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

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/semaphore.h>

#include "driver/driver.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint32_t g_devno;
static sem_t g_devno_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: unique_blkdev
 *
 * Description:
 *   Create a unique temporary device name in the /dev/ directory of the
 *   pseudo-file system.  We cannot use mktemp for this because it will
 *   attempt to open() the file.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The allocated path to the device.  This must be released by the caller
 *   to prevent memory links.  NULL will be returned only the case where
 *   we fail to allocate memory.
 *
 ****************************************************************************/

static FAR char *unique_blkdev(void)
{
  struct stat statbuf;
  char devbuf[16];
  uint32_t devno;
  int ret;

  /* Loop until we get a unique device name */

  for (; ; )
    {
      /* Get the semaphore protecting the path number */

      ret = nxsem_wait_uninterruptible(&g_devno_sem);
      if (ret < 0)
        {
          ferr("ERROR: nxsem_wait_uninterruptible failed: %d\n", ret);
          return NULL;
        }

      /* Get the next device number and release the semaphore */

      devno = ++g_devno;
      nxsem_post(&g_devno_sem);

      /* Construct the full device number */

      devno &= 0xffffff;
      snprintf(devbuf, 16, "/dev/tmpb%06lx", (unsigned long)devno);

      /* Make sure that file name is not in use */

      ret = stat(devbuf, &statbuf);
      if (ret < 0)
        {
          DEBUGASSERT(errno == ENOENT);
          return strdup(devbuf);
        }

      /* It is in use, try again */
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtd_proxy
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
 *   accesses to the mtd driver.
 *
 *   Errors that may be returned:
 *
 *     ENOMEM - Failed to create a temporary path name.
 *
 *   Plus:
 *
 *     - Errors reported from ftl_initialize()
 *     - Errors reported from open() or unlink()
 *
 ****************************************************************************/

int mtd_proxy(FAR const char *mtddev, int mountflags,
              FAR struct inode **ppinode)
{
  FAR struct inode *mtd;
  FAR char *blkdev;
  int ret;

  /* Create a unique temporary file name for the block device */

  blkdev = unique_blkdev();
  if (blkdev == NULL)
    {
      ferr("ERROR: Failed to create temporary device name\n");
      return -ENOMEM;
    }

  /* Wrap the mtd driver with an instance of the ftl driver */

  ret = find_mtddriver(mtddev, &mtd);
  if (ret < 0)
    {
      ferr("ERROR: Failed to find %s mtd driver\n", mtddev);
      goto out_with_blkdev;
    }

  ret = ftl_initialize_by_path(blkdev, mtd->u.i_mtd);
  inode_release(mtd);
  if (ret < 0)
    {
      ferr("ERROR: ftl_initialize_by_path(%s, %s) failed: %d\n",
           mtddev, blkdev, ret);
      goto out_with_blkdev;
    }

  /* Open the newly created block driver */

  ret = open_blockdriver(blkdev, mountflags, ppinode);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open %s: %d\n", blkdev, ret);
      goto out_with_fltdev;
    }

  /* Unlink and free the block device name.  The driver instance will
   * persist, provided that CONFIG_DISABLE_PSEUDOFS_OPERATIONS=y (otherwise,
   * we have a problem here!)
   */

out_with_fltdev:
  unlink(blkdev);
out_with_blkdev:
  kmm_free(blkdev);
  return ret;
}
