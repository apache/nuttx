/****************************************************************************
 * fs/nxffs/nxffs_ioctl.c
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

  /* Get exclusive access to the volume.  Note that the volume lock
   * protects the open file list.
   */

  ret = nxmutex_lock(&volume->lock);
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
          goto errout_with_excllock;
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

errout_with_excllock:
  nxmutex_unlock(&volume->lock);
errout:
  return ret;
}
