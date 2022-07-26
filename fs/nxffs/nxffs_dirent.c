/****************************************************************************
 * fs/nxffs/nxffs_dirent.c
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
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/dirent.h>

#include "nxffs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_opendir
 *
 * Description:
 *   Open a directory for read access
 *
 ****************************************************************************/

int nxffs_opendir(FAR struct inode *mountpt, FAR const char *relpath,
                  FAR struct fs_dirent_s *dir)
{
  struct nxffs_volume_s *volume;
  int ret;

  finfo("relpath: \"%s\"\n", relpath ? relpath : "NULL");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover the file system state from the NuttX inode instance */

  volume = mountpt->i_private;
  ret = nxsem_wait(&volume->exclsem);
  if (ret < 0)
    {
      goto errout;
    }

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      ret = -ENOENT;
      goto errout_with_semaphore;
    }

  /* Set the offset to the offset to the first valid inode */

  dir->u.nxffs.nx_offset = volume->inoffset;
  ret = OK;

errout_with_semaphore:
  nxsem_post(&volume->exclsem);

errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_readdir
 *
 * Description: Read the next directory entry
 *
 ****************************************************************************/

int nxffs_readdir(FAR struct inode *mountpt,
                  FAR struct fs_dirent_s *dir,
                  FAR struct dirent *dentry)
{
  FAR struct nxffs_volume_s *volume;
  struct nxffs_entry_s entry;
  off_t offset;
  int ret;

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover the file system state from the NuttX inode instance */

  volume = mountpt->i_private;
  ret = nxsem_wait(&volume->exclsem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Read the next inode header from the offset */

  offset = dir->u.nxffs.nx_offset;
  ret = nxffs_nextentry(volume, offset, &entry);

  /* If the read was successful, then handle the reported inode.  Note
   * that when the last inode has been reported, the value -ENOENT will
   * be returned.. which is correct for the readdir() method.
   */

  if (ret == OK)
    {
      /* Return the filename and file type */

      finfo("Offset %jd: \"%s\"\n", (intmax_t)entry.hoffset, entry.name);
      dentry->d_type = DTYPE_FILE;
      strlcpy(dentry->d_name, entry.name, sizeof(dentry->d_name));

      /* Discard this entry and set the next offset. */

      dir->u.nxffs.nx_offset = nxffs_inodeend(volume, &entry);
      nxffs_freeentry(&entry);
      ret = OK;
    }

  nxsem_post(&volume->exclsem);

errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_rewindir
 *
 * Description:
 *   Reset directory read to the first entry
 *
 ****************************************************************************/

int nxffs_rewinddir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir)
{
  FAR struct nxffs_volume_s *volume;
  int ret;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt != NULL && mountpt->i_private != NULL);

  /* Recover the file system state from the NuttX inode instance */

  volume = mountpt->i_private;
  ret = nxsem_wait(&volume->exclsem);
  if (ret < 0)
    {
      goto errout;
    }

  /* Reset the offset to the FLASH offset to the first valid inode */

  dir->u.nxffs.nx_offset = volume->inoffset;
  ret = OK;

  nxsem_post(&volume->exclsem);

errout:
  return ret;
}
