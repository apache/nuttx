/****************************************************************************
 * fs/nxffs/nxffs_ioctl.c
 *
 *   Copyright (C) 2011, 2013, 2017-2018 Gregory Nutt. All rights reserved.
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

#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_ioctl
 *
 * Description:
 *   Standard mountpoint ioctl method.
 *
 ****************************************************************************/

int nxffs_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct nxffs_volume_s *volume;
  int ret;

  finfo("cmd: %d arg: %08lx\n", cmd, arg);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the file system state from the open file */

  volume = filep->f_inode->i_private;
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

  /* Only a reformat and optimize commands are supported */

  if (cmd == FIOC_REFORMAT)
    {
      finfo("Reformat command\n");

      /* We cannot reformat the volume if there are any open inodes */

      if (volume->ofiles)
        {
          ferr("ERROR: Open files\n");
          ret = -EBUSY;
          goto errout_with_semaphore;
        }

      /* Re-format the volume -- all is lost */

      ret = nxffs_reformat(volume);
    }

  else if (cmd == FIOC_OPTIMIZE)
    {
      finfo("Optimize command\n");

      /* Pack the volume */

      ret = nxffs_pack(volume);
    }
  else
    {
      /* Command not recognized, forward to the MTD driver */

      ret = MTD_IOCTL(volume->mtd, cmd, arg);
    }

errout_with_semaphore:
  nxsem_post(&volume->exclsem);
errout:
  return ret;
}
