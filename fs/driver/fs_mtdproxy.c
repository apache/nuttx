/****************************************************************************
 * fs/driver/fs_mtdproxy.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>

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

      do
        {
          ret = nxsem_wait(&g_devno_sem);

          /* The only case that an error should occur here is if the wait
           * was awakened by a signal.
           */

          DEBUGASSERT(ret == OK || ret == -EINTR);
        }
      while (ret == -EINTR);

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
 *     ENOMEM - Failed to create a temporay path name.
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

  /* Unlink and free the block device name.  The driver instance will persist,
   * provided that CONFIG_DISABLE_PSEUDOFS_OPERATIONS=y (otherwise, we have
   * a problem here!)
   */

out_with_fltdev:
  unlink(blkdev);
out_with_blkdev:
  kmm_free(blkdev);
  return ret;
}

