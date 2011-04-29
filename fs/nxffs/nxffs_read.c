/****************************************************************************
 * fs/nxffs/nxffs_read.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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

#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs.h>
#include <nuttx/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_rdseek
 *
 * Description:
 *   Seek to the file position before read or write access.  Note that the
 *   simplier nxffs_ioseek() cannot be used for this purpose.  File offsets
 *   are not easily mapped to FLASH offsets due to intervening block and
 *   data headers.
 *
 * Input Parameters:
 *   volume - Describes the current volume
 *   entry  - Describes the open inode
 *   fpos   - The desired file position
 *
 ****************************************************************************/

static ssize_t nxffs_rdseek(FAR struct nxffs_volume_s *volume,
                            FAR struct nxffs_entry_s *entry,
                            off_t fpos)
{
#warning "Missing Logic"
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * Name: nxffs_read
 *
 * Description:
 *   This is an implementation of the NuttX standard file system read
 *   method.
 *
 ****************************************************************************/

ssize_t nxffs_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct nxffs_volume_s *volume;
  FAR struct nxffs_ofile_s *ofile;
  ssize_t ret;

  fvdbg("Read %d bytes from offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the open file state from the struct file instance */

  ofile = (FAR struct nxffs_ofile_s *)filep->f_priv;

  /* Recover the volume state from the open file */

  volume = (FAR struct nxffs_volume_s *)filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  /* Get exclusive access to the volume.  Note that the volume exclsem
   * protects the open file list.
   */

  ret = sem_wait(&volume->exclsem);
  if (ret != OK)
    {
      ret = -errno;
      fdbg("sem_wait failed: %d\n", ret);
      goto errout;
    }

  /* Check if the file was opened with read access */

  if ((ofile->mode & O_RDOK) == 0)
    {
      fdbg("File not open for read access\n");
      ret = -EACCES;
      goto errout_with_semaphore;
    }

  /* Seek to the current file offset */

  ret = nxffs_rdseek(volume, &ofile->entry, filep->f_pos);
  if (ret < 0)
    {
      fdbg("nxffs_fwseek failed: %d\n", -ret);
      ret = -EACCES;
      goto errout_with_semaphore;
    }

  /* Read data from that file offset */

  ret = nxffs_rddata(volume, (FAR uint8_t *)buffer, buflen);
  if (ret > 0)
    {
      /* Update the file offset */

      filep->f_pos += ret;
    }

errout_with_semaphore:
  sem_post(&volume->exclsem);
errout:
  return ret;
}

