/****************************************************************************
 * fs/nxffs/nxffs_ioctl.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <nuttx/debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_volume_cmd
 *
 * Description:
 *   Carry out an ioctl against the volume.  Shared by the per-file and the
 *   per-volume entry points, which differ only in how they name the volume.
 *
 ****************************************************************************/

static int nxffs_volume_cmd(FAR struct nxffs_volume_s *volume, int cmd,
                            unsigned long arg)
{
  int ret;

  /* Get exclusive access to the volume.  Note that the volume lock
   * protects the open file list.
   */

  ret = nxmutex_lock(&volume->lock);
  if (ret < 0)
    {
      ferr("ERROR: nxmutex_lock failed: %d\n", ret);
      return ret;
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
          goto errout_with_lock;
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

errout_with_lock:
  nxmutex_unlock(&volume->lock);
  return ret;
}

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

  finfo("cmd: %d arg: %08lx\n", cmd, arg);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL);

  /* Recover the file system state from the open file */

  volume = filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  return nxffs_volume_cmd(volume, cmd, arg);
}

/****************************************************************************
 * Name: nxffs_ioctldir
 *
 * Description:
 *   ioctl method for a descriptor on the mountpoint directory.
 *   FIOC_REFORMAT is only reachable this way: it refuses to run while any
 *   file on the volume is open, and the per-file method is itself such a
 *   file.  The open directory is not needed -- the volume is recovered from
 *   the mountpoint inode -- so dir is unused.
 *
 ****************************************************************************/

int nxffs_ioctldir(FAR struct inode *mountpt, FAR struct fs_dirent_s *dir,
                   int cmd, unsigned long arg)
{
  FAR struct nxffs_volume_s *volume;

  UNUSED(dir);

  finfo("cmd: %d arg: %08lx\n", cmd, arg);

  volume = mountpt->i_private;
  DEBUGASSERT(volume != NULL);

  return nxffs_volume_cmd(volume, cmd, arg);
}
