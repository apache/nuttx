/****************************************************************************
 * fs/fat/fs_fat32attrib.c
 *
 *   Copyright (C) 2007-2009, 2011, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <stdbool.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/fat.h>

#include "inode/inode.h"
#include "fs_fat32.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fat_attrib
 ****************************************************************************/

static int fat_attrib(const char *path, fat_attrib_t *retattrib,
                      fat_attrib_t setbits, fat_attrib_t clearbits)
{
  struct fat_mountpt_s *fs;
  struct fat_dirinfo_s dirinfo;
  struct inode_search_s desc;
  FAR struct inode *inode;
  uint8_t *direntry;
  uint8_t oldattributes;
  uint8_t newattributes;
  int status;
  int ret;

  /* Find the inode for this file */

  SETUP_SEARCH(&desc, path, false);

  status = inode_find(&desc);
  if (status < 0)
    {
      /* There is no mountpoint that includes in this path */

      ret = -status;
      goto errout;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* Verify that the inode is a valid mountpoint. */

  if (!INODE_IS_MOUNTPT(inode) || !inode->u.i_mops || !inode->i_private)
    {
      ret = ENXIO;
      goto errout_with_inode;
    }

  /* Get the mountpoint private data from the inode structure */

  fs = inode->i_private;

  /* Check if the mount is still healthy */

  ret = fat_semtake(fs);
  if (ret < 0)
    {
      goto errout_with_inode;
    }

  ret = fat_checkmount(fs);
  if (ret != OK)
    {
      goto errout_with_semaphore;
    }

  /* Find the file/directory entry for the relpath */

  ret = fat_finddirentry(fs, &dirinfo, desc.relpath);
  if (ret != OK)
    {
      /* Some error occurred -- probably -ENOENT */

      goto errout_with_semaphore;
    }

  /* Make sure that we found some valid file or directory */

  if (dirinfo.fd_root)
    {
      /* Ooops.. we found the root directory */

      ret = EACCES;
      goto errout_with_semaphore;
    }

  /* Get the current attributes */

  direntry      = &fs->fs_buffer[dirinfo.fd_seq.ds_offset];
  oldattributes = DIR_GETATTRIBUTES(direntry);
  newattributes = oldattributes;

  /* Set or clear any bits as requested */

  newattributes &= ~(clearbits & (FATATTR_READONLY | FATATTR_HIDDEN |
                                  FATATTR_SYSTEM | FATATTR_ARCHIVE));
  newattributes |=  (setbits   & (FATATTR_READONLY | FATATTR_HIDDEN |
                                  FATATTR_SYSTEM | FATATTR_ARCHIVE));

  /* Did any thingchange? */

  if (newattributes != oldattributes)
    {
      DIR_PUTATTRIBUTES(direntry, newattributes);
      fs->fs_dirty = true;
      ret = fat_updatefsinfo(fs);
      if (ret != OK)
        {
          ret = -ret;
          goto errout_with_semaphore;
        }
    }

  /* Success */

  if (retattrib)
    {
      *retattrib = newattributes;
    }

  fat_semgive(fs);
  inode_release(inode);
  RELEASE_SEARCH(&desc);
  return OK;

errout_with_semaphore:
  fat_semgive(fs);

errout_with_inode:
  inode_release(inode);

errout:
  RELEASE_SEARCH(&desc);
  set_errno(ret);
  return ERROR;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: fat_getattrib
 ****************************************************************************/

int fat_getattrib(const char *path, fat_attrib_t *attrib)
{
  return fat_attrib(path, attrib, 0, 0);
}

/****************************************************************************
 * Name: fat_setattrib
 ****************************************************************************/

int fat_setattrib(const char *path, fat_attrib_t setbits,
                  fat_attrib_t clearbits)
{
  return fat_attrib(path, NULL, setbits, clearbits);
}
