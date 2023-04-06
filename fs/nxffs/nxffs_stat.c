/****************************************************************************
 * fs/nxffs/nxffs_stat.c
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

#include <sys/stat.h>
#include <sys/statfs.h>

#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_statfs
 *
 * Description: Return filesystem statistics
 *
 ****************************************************************************/

int nxffs_statfs(FAR struct inode *mountpt, FAR struct statfs *buf)
{
  FAR struct nxffs_volume_s *volume;
  int ret;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the NuttX inode structure */

  volume = mountpt->i_private;
  ret = nxmutex_lock(&volume->lock);
  if (ret < 0)
    {
      goto errout;
    }

  /* Fill in the statfs info
   *
   * REVISIT: Need f_bfree, f_bavail, f_files, f_ffree calculation
   */

  memset(buf, 0, sizeof(struct statfs));
  buf->f_type    = NXFFS_MAGIC;
  buf->f_bsize   = volume->geo.blocksize;
  buf->f_blocks  = volume->nblocks;
  buf->f_namelen = volume->geo.blocksize - SIZEOF_NXFFS_BLOCK_HDR -
                   SIZEOF_NXFFS_INODE_HDR;
  ret            = OK;

  nxmutex_unlock(&volume->lock);

errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_stat
 *
 * Description: Return information about a file or directory
 *
 ****************************************************************************/

int nxffs_stat(FAR struct inode *mountpt, FAR const char *relpath,
               FAR struct stat *buf)
{
  FAR struct nxffs_volume_s *volume;
  struct nxffs_entry_s entry;
  int ret;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private && buf);

  /* Get the mountpoint private data from the NuttX inode structure */

  volume = mountpt->i_private;
  ret = nxmutex_lock(&volume->lock);
  if (ret != OK)
    {
      goto errout;
    }

  /* Initialize the return stat instance */

  memset(buf, 0, sizeof(struct stat));
  buf->st_blksize = volume->geo.blocksize;

  /* The requested directory must be the volume-relative "root" directory */

  if (relpath && relpath[0] != '\0')
    {
      /* Not the top directory.. find the NXFFS inode with this name */

      ret = nxffs_findinode(volume, relpath, &entry);
      if (ret < 0)
        {
          ferr("ERROR: Inode '%s' not found: %d\n", relpath, -ret);
          goto errout_with_lock;
        }

      /* Return status information based on the directory entry */

      buf->st_blocks  = entry.datlen /
                        (volume->geo.blocksize - SIZEOF_NXFFS_BLOCK_HDR);
      buf->st_mode    = S_IFREG | S_IXOTH | S_IXGRP | S_IXUSR;
      buf->st_size    = entry.datlen;
      buf->st_atime   = entry.utc;
      buf->st_mtime   = entry.utc;
      buf->st_ctime   = entry.utc;

      /* Free inode resources */

      nxffs_freeentry(&entry);
    }
  else
    {
      /* It's a read/execute-only directory name */

      buf->st_mode   = S_IFDIR | S_IROTH | S_IRGRP | S_IRUSR | S_IXOTH |
                       S_IXGRP | S_IXUSR;
    }

  ret = OK;

errout_with_lock:
  nxmutex_unlock(&volume->lock);

errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_fstat
 *
 * Description:
 *   Obtain information about an open file associated with the file
 *   descriptor 'fd', and will write it to the area pointed to by 'buf'.
 *
 ****************************************************************************/

int nxffs_fstat(FAR const struct file *filep, FAR struct stat *buf)
{
  FAR struct nxffs_volume_s *volume;
  FAR struct nxffs_ofile_s *ofile;
  int ret;

  finfo("Buf %p\n", buf);
  DEBUGASSERT(filep != NULL && buf != NULL);

  /* Recover the open file state from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
  ofile = (FAR struct nxffs_ofile_s *)filep->f_priv;

  /* Recover the volume state from the open file */

  volume = (FAR struct nxffs_volume_s *)filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  /* Get exclusive access to the volume.  Note that the volume lock
   * protects the open file list.
   */

  ret = nxmutex_lock(&volume->lock);
  if (ret != OK)
    {
      ferr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
    }

  /* Return status information based on the directory entry */

  buf->st_blocks = ofile->entry.datlen /
                   (volume->geo.blocksize - SIZEOF_NXFFS_BLOCK_HDR);
  buf->st_mode   = S_IFREG | S_IXOTH | S_IXGRP | S_IXUSR;
  buf->st_size   = ofile->entry.datlen;
  buf->st_atime  = ofile->entry.utc;
  buf->st_mtime  = ofile->entry.utc;
  buf->st_ctime  = ofile->entry.utc;

  nxmutex_unlock(&volume->lock);
  return OK;
}
