/****************************************************************************
 * fs/nxffs/nxffs_blockstats.c
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
#include <debug.h>

#include <nuttx/mtd/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_blockstats
 *
 * Description:
 *   Analyze the NXFFS volume.  This operation must be performed when the
 *   volume is first mounted in order to detect if the volume has been
 *   formatted and contains a usable NXFFS file system.
 *
 * Input Parameters:
 *   volume - Describes the current NXFFS volume.
 *   stats  - On return, will hold nformation describing the state of the
 *     volume.
 *
 * Returned Value:
 *   Negated errnos are returned only in the case of MTD reported failures.
 *   Nothing in the volume data itself will generate errors.
 *
 ****************************************************************************/

int nxffs_blockstats(FAR struct nxffs_volume_s *volume,
                     FAR struct nxffs_blkstats_s *stats)
{
#ifndef CONFIG_NXFFS_NAND
  FAR uint8_t *bptr;     /* Pointer to next block data */
  int lblock;            /* Logical block index */
#endif
  off_t ioblock;         /* I/O block number */
  int ret;

  /* Process each erase block */

  memset(stats, 0, sizeof(struct nxffs_blkstats_s));

#ifndef CONFIG_NXFFS_NAND
  for (ioblock = 0; ioblock < volume->nblocks; ioblock += volume->blkper)
    {
      /* Read the full erase block */

      ret = MTD_BREAD(volume->mtd, ioblock, volume->blkper, volume->pack);
      if (ret < volume->blkper)
        {
          ferr("ERROR: Failed to read erase block %jd: %d\n",
               (intmax_t)(ioblock / volume->blkper), ret);
          return ret;
        }

      /* Then examine each logical block in the erase block */

      for (bptr = volume->pack, lblock = 0;
           lblock < volume->blkper;
           bptr += volume->geo.blocksize, lblock++)
        {
          /* We read the block successfully, now check for errors tagged
           * in the NXFFS data.
           */

          FAR struct nxffs_block_s *blkhdr = (FAR struct nxffs_block_s *)
                                             bptr;

          /* Increment the total count of blocks examined */

          stats->nblocks++;

          /* Collect statistics.
           *
           * Check if this is a block that should be recognized by NXFFS.
           */

          if (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) != 0)
            {
              /* Nope.. block must not be formatted */

              stats->nunformat++;
            }
          else if (blkhdr->state == BLOCK_STATE_BAD)
            {
              /* The block is marked as bad */

               stats->nbad++;
            }
          else if (blkhdr->state == BLOCK_STATE_GOOD)
            {
              /* The block is marked as good */

              stats-> ngood++;
            }
          else
            {
              /* The good/bad mark is not recognized.  Let's call this
               * corrupt (vs. unformatted).
               */

              stats->ncorrupt++;
            }
        }
    }

  finfo("Number blocks:        %jd\n", (intmax_t)stats->nblocks);
  finfo("  Good blocks:        %jd\n", (intmax_t)stats->ngood);
  finfo("  Bad blocks:         %jd\n", (intmax_t)stats->nbad);
  finfo("  Unformatted blocks: %jd\n", (intmax_t)stats->nunformat);
  finfo("  Corrupt blocks:     %jd\n", (intmax_t)stats->ncorrupt);

#else
  for (ioblock = 0; ioblock < volume->nblocks; ioblock++)
    {
      /* Increment the total count of blocks examined */

      stats->nblocks++;

      /* Read each logical block, one at a time.  We could read all of the
       * blocks in the erase block into volume->pack at once.  But this would
       * be a problem for NAND which may generate read errors due to bad ECC
       * on individual blocks.
       */

      ret = MTD_BREAD(volume->mtd, ioblock, 1, volume->pack);
      if (ret < 1)
        {
          /* This should not happen at all on most kinds of FLASH.  But a
           * bad read will happen normally with a NAND device that has
           * uncorrectable blocks.  So, just for NAND, we keep the count
           * of unreadable blocks.
           */

          ferr("ERROR: Failed to read block %d: %d\n", ioblock, ret);

          /* Increment the count of un-readable blocks */

          stats->nbadread++;
        }
      else
        {
          /* We read the block successfully, now check for errors tagged
           * in the NXFFS data.
           */

          FAR struct nxffs_block_s *blkhdr = (FAR struct nxffs_block_s *)
                                             volume->pack;

          /* Collect statistics.
           *
           * Check if this is a block that should be recognized by NXFFS.
           */

          if (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) != 0)
            {
              /* Nope.. block must not be formatted */

              stats->nunformat++;
            }
          else if (blkhdr->state == BLOCK_STATE_BAD)
            {
              /* The block is marked as bad */

              stats->nbad++;
            }
          else if (blkhdr->state == BLOCK_STATE_GOOD)
            {
              /* The block is marked as good */

              stats-> ngood++;
            }
          else
            {
              /* The good/bad mark is not recognized.  Let's call this
               * corrupt (vs. unformatted).
               */

              stats->ncorrupt++;
            }
        }
    }

  finfo("Number blocks:        %d\n", stats->nblocks);
  finfo("  Good blocks:        %d\n", stats->ngood);
  finfo("  Bad blocks:         %d\n", stats->nbad);
  finfo("  Unformatted blocks: %d\n", stats->nunformat);
  finfo("  Corrupt blocks:     %d\n", stats->ncorrupt);
  finfo("  Unreadable blocks:  %d\n", stats->nbadread);

#endif
  return OK;
}
