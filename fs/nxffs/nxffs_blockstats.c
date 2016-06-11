/****************************************************************************
 * fs/nxffs/nxffs_blockstats.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
          ferr("ERROR: Failed to read erase block %d: %d\n",
               ioblock / volume->blkper, ret);
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

          FAR struct nxffs_block_s *blkhdr = (FAR struct nxffs_block_s *)bptr;

          /* Increment the total count of blocks examined */

          stats->nblocks++;

          /* Collect statistics */
          /* Check if this is a block that should be recognized by NXFFS */

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

          FAR struct nxffs_block_s *blkhdr = (FAR struct nxffs_block_s *)volume->pack;

          /* Collect statistics */
          /* Check if this is a block that should be recognized by NXFFS */

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
