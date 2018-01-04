/****************************************************************************
 * fs/nxffs/nxffs_stat.c
 *
 *   Copyright (C) 2011, 2017-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * References: Linux/Documentation/filesystems/romfs.txt
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

#include <sys/stat.h>
#include <sys/statfs.h>

#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>
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
  ret = nxsem_wait(&volume->exclsem);
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
  buf->f_namelen = volume->geo.blocksize - SIZEOF_NXFFS_BLOCK_HDR - SIZEOF_NXFFS_INODE_HDR;
  ret            = OK;

  nxsem_post(&volume->exclsem);

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
  ret = nxsem_wait(&volume->exclsem);
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
          ferr("ERROR: Inode '%s' not found: %d\n", -ret);
          goto errout_with_semaphore;
        }

      /* Return status information based on the directory entry */

      buf->st_blocks  = entry.datlen / (volume->geo.blocksize - SIZEOF_NXFFS_BLOCK_HDR);
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

errout_with_semaphore:
  nxsem_post(&volume->exclsem);

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

  finfo("Buf %s\n", buf);
  DEBUGASSERT(filep != NULL && buf != NULL);

  /* Recover the open file state from the struct file instance */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);
  ofile = (FAR struct nxffs_ofile_s *)filep->f_priv;

  /* Recover the volume state from the open file */

  volume = (FAR struct nxffs_volume_s *)filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  /* Get exclusive access to the volume.  Note that the volume exclsem
   * protects the open file list.
   */

  ret = nxsem_wait(&volume->exclsem);
  if (ret != OK)
    {
      ferr("ERROR: nxsem_wait failed: %d\n", ret);
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

  nxsem_post(&volume->exclsem);
  return OK;
}
