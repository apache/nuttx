/****************************************************************************
 * fs/nxffs/nxffs_truncate.c
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

#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

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
      /* We are shrinking the file.
       *
       * REVISIT:  Logic to shrink the file has not yet been implemented.
       */

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
