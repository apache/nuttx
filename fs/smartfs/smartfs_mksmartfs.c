/****************************************************************************
 * fs/smartfs/smartfs_mksmartfs.c
 *
 *   Copyright (C) 2013 Ken Pettit. All rights reserved.
 *   Author: Ken Pettit <pettitkd@gmail.com>
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/mksmartfs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/smart.h>

#include "smartfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mksmartfs
 *
 * Description:
 *   Make a SMART Flash file system image on the specified block device
 *
 * Inputs:
 *   pathname - the full path to a registered block driver
 *   nrootdirs - Number of root directory entries to create.
 *
 * Return:
 *   Zero (OK) on success; -1 (ERROR) on failure with errno set appropriately:
 *
 *   EINVAL  - NULL block driver string, bad number of FATS in 'fmt', bad FAT
 *     size in 'fmt', bad cluster size in 'fmt'
 *   ENOENT  - 'pathname' does not refer to anything in the filesystem.
 *   ENOTBLK - 'pathname' does not refer to a block driver
 *   EACCES  - block driver does not support wrie or geometry methods
 *
 * Assumptions:
 *   - The caller must assure that the block driver is not mounted and not in
 *     use when this function is called.  The result of formatting a mounted
 *     device is indeterminate (but likely not good).
 *
 ****************************************************************************/

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
int mksmartfs(FAR const char *pathname, uint8_t nrootdirs)
#else
int mksmartfs(FAR const char *pathname)
#endif
{
  struct inode* inode;
  struct smart_format_s fmt;
  int ret;
  int x;
  uint8_t type;
  struct smart_read_write_s request;

  /* Find the inode of the block driver indentified by 'source' */

  ret = open_blockdriver(pathname, 0, &inode);
  if (ret < 0)
    {
      fdbg("Failed to open %s\n", pathname);
      goto errout;
    }

  /* Make sure that the inode supports the write and geometry methods at a minimum */

  if (!inode->u.i_bops->write || !inode->u.i_bops->geometry)
    {
      fdbg("%s does not support write or geometry methods\n", pathname);
      ret = -EACCES;
      goto errout_with_driver;
    }

  /* Validate the block device is a SMART device */

  /* Perform a low-level SMART format */

#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  ret = inode->u.i_bops->ioctl(inode, BIOC_LLFORMAT, nrootdirs);
#else
  ret = inode->u.i_bops->ioctl(inode, BIOC_LLFORMAT, 0);
#endif
  if (ret != OK)
    {
      fdbg("Error creating low-level format: %d\n", ret);
      goto errout_with_driver;
    }

  /* Get the format information so we know how big the sectors are */

  ret = inode->u.i_bops->ioctl(inode, BIOC_GETFORMAT, (unsigned long) &fmt);

  /* Now Write the filesystem to media.  Loop for each root dir entry and
   * allocate the reserved Root Dir Enty, then write a blank root dir for it.
   */

  type = SMARTFS_SECTOR_TYPE_DIR;
  request.offset = 0;
  request.count = 1;
  request.buffer = &type;
  x = 0;
#ifdef CONFIG_SMARTFS_MULTI_ROOT_DIRS
  for (; x < nrootdirs; x++)
#endif
    {
      ret = inode->u.i_bops->ioctl(inode, BIOC_ALLOCSECT, SMARTFS_ROOT_DIR_SECTOR + x);
      if (ret != SMARTFS_ROOT_DIR_SECTOR + x)
        {
          ret = -EIO;
          goto errout_with_driver;
        }

      /* Mark this block as a directory entry */

      request.logsector = SMARTFS_ROOT_DIR_SECTOR + x;

      /* Issue a write to the sector, single byte */

      ret = inode->u.i_bops->ioctl(inode, BIOC_WRITESECT, (unsigned long) &request);
      if (ret != 0)
        {
          ret = -EIO;
          goto errout_with_driver;
        }
    }

errout_with_driver:
  /* Close the driver */

  (void)close_blockdriver(inode);

errout:
  /* Release all allocated memory */

  /* Return any reported errors */

  if (ret < 0)
    {
      errno = -ret;
      return ERROR;
    }

  return OK;
}
