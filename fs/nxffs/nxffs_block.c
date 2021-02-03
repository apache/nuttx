/****************************************************************************
 * fs/nxffs/nxffs_block.c
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
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mtd/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_verifyblock
 *
 * Description:
 *   Assure that the provided (logical) block number is in the block cache
 *   and that it has a valid block header (i.e., proper magic and
 *   marked good)
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   block - The (logical) block number to load and verify.
 *
 * Returned Value:
 *   OK (zero( is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure:
 *
 *     -EIO is returned if we failed to read the block.  If we are using
 *        NAND memory, then this probably means that the block has
 *        uncorrectable bit errors.
 *     -ENOENT is returned if the block is a bad block.
 *
 ****************************************************************************/

int nxffs_verifyblock(FAR struct nxffs_volume_s *volume, off_t block)
{
  FAR struct nxffs_block_s *blkhdr;
  int ret;

  /* Make sure that the block is in the cache */

  ret = nxffs_rdcache(volume, block);
  if (ret < 0)
    {
      /* Perhaps we are at the end of the media */

      ferr("ERROR: Failed to read data into cache: %d\n", ret);
      return -EIO;
    }

  /* Check if the block has a magic number (meaning that it is not
   * erased) and that it is valid (meaning that it is not marked
   * for deletion)
   */

  blkhdr = (FAR struct nxffs_block_s *)volume->cache;
  if (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) == 0)
    {
      /* This does appear to be a block */

      if (blkhdr->state == BLOCK_STATE_GOOD)
        {
          /* The block is valid */

          return OK;
        }
      else if (blkhdr->state == BLOCK_STATE_BAD)
        {
          /* -ENOENT is a special indication that this is a properly marked
           * bad block
           */

          return -ENOENT;
        }
    }

  /* Whatever is here where a block header should be is invalid */

  return -EINVAL;
}

/****************************************************************************
 * Name: nxffs_validblock
 *
 * Description:
 *   Find the next valid (logical) block in the volume.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   block  - On entry, this provides the starting block number.  If the
 *     function is succesfful, then this memory location will hold the
 *     block number of the next valid block on return.
 *
 *  Returned Value:
 *    Zero on success otherwise a negated errno value indicating the nature
 *    of the failure.
 *
 ****************************************************************************/

int nxffs_validblock(struct nxffs_volume_s *volume, off_t *block)
{
  off_t i;
  int ret;

  DEBUGASSERT(volume && block);

  /* Loop for each possible block or until a valid block is found */

  for (i = *block; i < volume->nblocks; i++)
    {
      /* Loop until we find a valid block */

      ret = nxffs_verifyblock(volume, i);
      if (ret == OK)
        {
          /* We found it, return the block number */

          *block = i;
          return OK;
        }
    }

  /* ENOSPC is special return value that means that there is no further,
   * valid blocks left in the volume.
   */

  ferr("ERROR: No valid block found\n");
  return -ENOSPC;
}
