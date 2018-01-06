/****************************************************************************
 * fs/nxffs/nxffs_truncate.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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

#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/semaphore.h>

#include "nxffs.h"

#ifdef __NO_TRUNCATE_SUPPORT__

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_truncate
 *
 * Description:
 *   Set the length of the open, regular file associated with the file
 *   structure 'filep' to 'length'.
 *
 ****************************************************************************/

int nxffs_truncate(FAR struct file *filep, off_t length)
{
  FAR struct nxffs_volume_s *volume;
  FAR struct nxffs_wrfile_s *wrfile;
  off_t oldsize;
  int ret;

  finfo("Write %d bytes to offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the open file state from the struct file instance */

  wrfile = (FAR struct nxffs_wrfile_s *)filep->f_priv;

  /* Recover the volume state from the open file */

  volume = (FAR struct nxffs_volume_s *)filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  /* Get exclusive access to the volume.  Note that the volume exclsem
   * protects the open file list.
   */

  ret = nxsem_wait(&volume->exclsem);
  if (ret < 0)
    {
      ferr("ERROR: nxsem_wait failed: %d\n", ret);
      goto errout;
    }

  /* Check if the file was opened with write access */

  if ((wrfile->ofile.oflags & O_WROK) == 0)
    {
      ferr("ERROR: File not open for write access\n");
      ret = -EACCES;
      goto errout_with_semaphore;
    }

  /* Are we shrinking the file?  Or extending it? */

  oldsize = wrfile->ofile.entry.datlen;
  if (oldsize == length)
    {
      ret = OK;
      goto errout_with_semaphore;
    }
  else if (oldsize > length)
    {
      /* We are shrinking the file */
      /* REVISIT:  Logic to shrink the file has not yet been implemented */

      ret = -ENOSYS;
    }
  else
    {
      /* We are zero-extending the file.  This essential amount to a write-
       * append operation with zero data.
       */

      ret = nxffs_wrextend(volume, wrfile, length);
    }

errout_with_semaphore:
  nxsem_post(&volume->exclsem);

errout:
  return ret;
}

#endif /* __NO_TRUNCATE_SUPPORT__ */
