/****************************************************************************
 * fs/nxffs/nxffs_unlink.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_rminode
 *
 * Description:
 *   Remove an inode from FLASH.  This is the internal implementation of
 *   the file system unlink operation.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.
 *   name - the name of the inode to be deleted.
 *
 * Returned Value:
 *   Zero is returned if the inode is successfully deleted.  Otherwise, a
 *   negated errno value is returned indicating the nature of the failure.
 *
 ****************************************************************************/

int nxffs_rminode(FAR struct nxffs_volume_s *volume, FAR const char *name)
{
  FAR struct nxffs_ofile_s *ofile;
  FAR struct nxffs_inode_s *inode;
  struct nxffs_entry_s entry;
  int ret;

  /* Check if the file is open */

  ofile = nxffs_findofile(volume, name);
  if (ofile)
    {
      /* We can't remove the inode if it is open */

      ferr("ERROR: Inode '%s' is open\n", name);
      ret = -EBUSY;
      goto errout;
    }

  /* Find the NXFFS inode */

  ret = nxffs_findinode(volume, name, &entry);
  if (ret < 0)
    {
      ferr("ERROR: Inode '%s' not found\n", name);
      goto errout;
    }

  /* Set the position to the FLASH offset of the file header (nxffs_findinode
   * should have left the block in the cache).
   */

  nxffs_ioseek(volume, entry.hoffset);

  /* Make sure that the block is in the cache */

  ret = nxffs_rdcache(volume, volume->ioblock);
  if (ret < 0)
    {
      ferr("ERROR: Failed to read block %jd into cache: %d\n",
           (intmax_t)volume->ioblock, ret);
      goto errout_with_entry;
    }

  /* Change the file status... it is no longer valid */

  inode = (FAR struct nxffs_inode_s *)&volume->cache[volume->iooffset];
  inode->state = INODE_STATE_DELETED;

  /* Then write the cached block back to FLASH */

  ret = nxffs_wrcache(volume);
  if (ret < 0)
    {
      ferr("ERROR: Failed to write block %jd: %d\n",
           (intmax_t)volume->ioblock, ret);
    }

errout_with_entry:
  nxffs_freeentry(&entry);
errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_unlink
 *
 * Description: Remove a file
 *
 ****************************************************************************/

int nxffs_unlink(FAR struct inode *mountpt, FAR const char *relpath)
{
  FAR struct nxffs_volume_s *volume;
  int ret;

  finfo("Entry\n");

  /* Sanity checks */

  DEBUGASSERT(mountpt && mountpt->i_private);

  /* Get the mountpoint private data from the NuttX inode structure */

  volume = mountpt->i_private;
  ret = nxmutex_lock(&volume->lock);
  if (ret != OK)
    {
      goto errout;
    }

  /* Then remove the NXFFS inode */

  ret = nxffs_rminode(volume, relpath);

  nxmutex_unlock(&volume->lock);

errout:
  return ret;
}
