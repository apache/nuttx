/****************************************************************************
 * fs/fat/fs_fat32attrib.c
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

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
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
  int ret;

  /* Find the inode for this file */

  SETUP_SEARCH(&desc, path, false);

  ret = inode_find(&desc);
  if (ret < 0)
    {
      /* There is no mountpoint that includes in this path */

      goto errout;
    }

  /* Get the search results */

  inode = desc.node;
  DEBUGASSERT(inode != NULL);

  /* Verify that the inode is a valid mountpoint. */

  if (!INODE_IS_MOUNTPT(inode) || !inode->u.i_mops || !inode->i_private)
    {
      ret = -ENXIO;
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

      ret = -EACCES;
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
  return ret;
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
