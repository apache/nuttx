/****************************************************************************
 * fs/nxffs/nxffs_cache.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>
#include <inttypes.h>
#include <stdint.h>

#include <nuttx/mtd/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_rdcache
 *
 * Description:
 *   Read one I/O block into the volume block cache memory.
 *
 * Input Parameters:
 *   volume - Describes the current volume
 *   block  - The first logical block to read
 *
 * Returned Value:
 *   Negated errnos are returned only in the case of MTD reported failures.
 *   Nothing in the volume data itself will generate errors.
 *
 ****************************************************************************/

int nxffs_rdcache(FAR struct nxffs_volume_s *volume, off_t block)
{
  size_t nxfrd;

  /* Check if the requested data is already in the cache */

  if (block != volume->cblock)
    {
      /* Read the specified blocks into cache */

      nxfrd = MTD_BREAD(volume->mtd, block, 1, volume->cache);
      if (nxfrd != 1)
        {
          ferr("ERROR: Read block %jd failed: %zu\n",
               (intmax_t)block, nxfrd);
          return -EIO;
        }

      /* Remember what is in the cache */

      volume->cblock  = block;
    }

  return OK;
}

/****************************************************************************
 * Name: nxffs_wrcache
 *
 * Description:
 *   Write one or more logical blocks from the volume cache memory.
 *
 * Input Parameters:
 *   volume - Describes the current volume
 *
 * Returned Value:
 *   Negated errnos are returned only in the case of MTD reported failures.
 *
 ****************************************************************************/

int nxffs_wrcache(FAR struct nxffs_volume_s *volume)
{
  size_t nxfrd;

  /* Write the current block from the cache */

  nxfrd = MTD_BWRITE(volume->mtd, volume->cblock, 1, volume->cache);
  if (nxfrd != 1)
    {
      ferr("ERROR: Write block %jd failed: %zu\n",
           (intmax_t)volume->cblock, nxfrd);
      return -EIO;
    }

  /* Write was successful */

  return OK;
}

/****************************************************************************
 * Name: nxffs_ioseek
 *
 * Description:
 *   Seek to a position in FLASH memory.  This simply sets up the offsets
 *   and pointer values.  This is a necessary step prior to using
 *   nxffs_getc().
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   offset - The physical offset in bytes from the beginning of the FLASH
 *     in bytes.
 *
 ****************************************************************************/

void nxffs_ioseek(FAR struct nxffs_volume_s *volume, off_t offset)
{
  /* Convert the offset into a block number and a byte offset into the
   * block.
   */

  volume->ioblock  = offset / volume->geo.blocksize;
  volume->iooffset = offset - volume->geo.blocksize * volume->ioblock;
}

/****************************************************************************
 * Name: nxffs_iotell
 *
 * Description:
 *   Report the current position.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *
 * Returned Value:
 *   The offset from the beginning of FLASH to the current seek position.
 *
 ****************************************************************************/

off_t nxffs_iotell(FAR struct nxffs_volume_s *volume)
{
  return volume->ioblock * volume->geo.blocksize + volume->iooffset;
}

/****************************************************************************
 * Name: nxffs_getc
 *
 * Description:
 *   Get the next byte from FLASH.  This function allows the data in the
 *   formatted FLASH blocks to be read as a continuous byte stream, skipping
 *   over bad blocks and block headers as necessary.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.  The parameters ioblock and
 *     iooffset in the volume structure determine the behavior of
 *     nxffs_getc().
 *   reserve - If less than this much space is available at the end of the
 *     block, then skip to the next block.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno indicating the
 *   nature of the failure.
 *
 ****************************************************************************/

int nxffs_getc(FAR struct nxffs_volume_s *volume, uint16_t reserve)
{
  int ret;

  DEBUGASSERT(reserve > 0);

  /* Loop to skip over bad blocks */

  do
    {
      /* Check if we have the reserve amount at the end of the current
       * block
       */

      if (volume->iooffset + reserve > volume->geo.blocksize)
        {
          /* Check for attempt to read past the end of FLASH */

          off_t nextblock = volume->ioblock + 1;
          if (nextblock >= volume->nblocks)
            {
              finfo("End of FLASH encountered\n");
              return -ENOSPC;
            }

          /* Set up the seek to the data just after the header in the
           * next block.
           */

          volume->ioblock  = nextblock;
          volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
        }

      /* Make sure that the block is in the cache.  The special error
       * -ENOENT indicates the block was read successfully but was not
       * marked as a good block.  In this case we need to skip to the
       * next block.  All other errors are fatal.
       */

      ret = nxffs_verifyblock(volume, volume->ioblock);
      if (ret < 0 && ret != -ENOENT)
        {
#ifndef CONFIG_NXFFS_NAND
          /* Read errors are fatal */

          ferr("ERROR: Failed to read valid data into cache: %d\n", ret);
          return ret;
#else
          /* A read error occurred.  This probably means that we are
           * using NAND memory this block has an uncorrectable bit error.
           * Ignore the error (after complaining) and try the next
           * block.
           */

          ferr("ERROR: Failed to read valid data into cache: %d\n", ret);
#endif
        }
    }
  while (ret != OK);

  /* Return the character at this offset.  Note that on return,
   * iooffset could point to the byte outside of the current block.
   */

  ret = (int)volume->cache[volume->iooffset];
  volume->iooffset++;
  return ret;
}
