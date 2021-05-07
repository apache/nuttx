/****************************************************************************
 * fs/nxffs/nxffs_reformat.c
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
#include <debug.h>

#include <nuttx/mtd/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_format
 *
 * Description:
 *   Erase and reformat the entire volume.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume to be reformatted.
 *
 * Returned Value:
 *   Zero on success or a negated errno on a failure.  Failures will be
 *   returned n the case of MTD reported failures o.
 *   Nothing in the volume data itself will generate errors.
 *
 ****************************************************************************/

static int nxffs_format(FAR struct nxffs_volume_s *volume)
{
  FAR uint8_t *blkptr;   /* Pointer to next block data */
  off_t eblock;          /* Erase block number */
  off_t lblock;          /* Logical block number */
  ssize_t nxfrd;         /* Number of blocks transferred */
  int i;
  int ret;

  /* Create an image of one properly formatted erase sector */

  for (blkptr = volume->pack, i = 0;
       i < volume->blkper;
       blkptr += volume->geo.blocksize, i++)
    {
      nxffs_blkinit(volume, blkptr, BLOCK_STATE_GOOD);
    }

  /* Erase and format each erase block */

  for (eblock = 0; eblock < volume->geo.neraseblocks; eblock++)
    {
      /* Erase the block */

      ret = MTD_ERASE(volume->mtd, eblock, 1);
      if (ret < 0)
        {
          ferr("ERROR: Erase block %jd failed: %d\n", (intmax_t)eblock, ret);
          return ret;
        }

      /* Write the formatted image to the erase block */

      lblock = eblock * volume->blkper;
      nxfrd = MTD_BWRITE(volume->mtd, lblock, volume->blkper, volume->pack);
      if (nxfrd != volume->blkper)
        {
          ferr("ERROR: Write erase block %jd failed: %zd\n",
               (intmax_t)lblock, nxfrd);
          return -EIO;
        }
    }

  return OK;
}

/****************************************************************************
 * Name: nxffs_badblocks
 *
 * Description:
 *   Verify each block and mark improperly erased blocks as bad.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume to be reformatted.
 *
 * Returned Value:
 *   Zero on success or a negated errno on a failure.  Failures will be
 *   returned n the case of MTD reported failures o.
 *   Nothing in the volume data itself will generate errors.
 *
 ****************************************************************************/

static int nxffs_badblocks(FAR struct nxffs_volume_s *volume)
{
  FAR uint8_t *blkptr;   /* Pointer to next block data */
  off_t eblock;          /* Erase block number */
  off_t lblock;          /* Logical block number of the erase block */
#ifdef CONFIG_NXFFS_NAND
  off_t block;           /* Working block number */
#endif
  ssize_t nxfrd;         /* Number of blocks transferred */
  bool good;             /* TRUE: block is good */
  bool modified;         /* TRUE: The erase block has been modified */
  int i;

  /* Read and verify each erase block */

  for (eblock = 0; eblock < volume->geo.neraseblocks; eblock++)
    {
      /* Get the logical block number of the erase block */

      lblock = eblock * volume->blkper;

#ifndef CONFIG_NXFFS_NAND
      /* Read the entire erase block */

      nxfrd  = MTD_BREAD(volume->mtd, lblock, volume->blkper, volume->pack);
      if (nxfrd != volume->blkper)
        {
          ferr("ERROR: Read erase block %jd failed: %zd\n",
               (intmax_t)lblock, nxfrd);
          return -EIO;
        }
#endif

      /* Keep track if any part of the erase block gets modified */

       modified = false;

      /* Process each logical block */

#ifndef CONFIG_NXFFS_NAND
      for (blkptr = volume->pack, i = 0;
           i < volume->blkper;
           blkptr += volume->geo.blocksize, i++)
#else
      for (i = 0, block = lblock, blkptr = volume->pack;
           i < volume->blkper;
           i++, block++, blkptr += volume->geo.blocksize)
#endif
        {
          FAR struct nxffs_block_s *blkhdr =
            (FAR struct nxffs_block_s *)blkptr;

          /* Assume that this is a good block until we learn otherwise */

          good = true;

#ifdef CONFIG_NXFFS_NAND
          /* Read the next block in the erase block */

          nxfrd = MTD_BREAD(volume->mtd, block, 1, blkptr);
          if (nxfrd < 0)
            {
              /* Failed to read the block.  This should never happen with
               * most FLASH.  However, for NAND this probably means that we
               * read a block with uncorrectable bit errors.
               */

              ferr("ERROR: Failed to read block %d: %d\n",
                   block, (int)nxfrd);

              good = false;
            }
          else
#endif
          /* Check block header */

          if (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) != 0 ||
              blkhdr->state != BLOCK_STATE_GOOD)
            {
              /* The block is not formatted with the NXFFS magic bytes or
               * else the block is specifically marked bad.
               */

              good = false;
            }

          /* This is a properly formatted, good NXFFS block. Check that the
           * block data payload is erased.
           */

          else
            {
              size_t blocksize = volume->geo.blocksize -
                                 SIZEOF_NXFFS_BLOCK_HDR;
              size_t erasesize = nxffs_erased(
                                     &blkptr[SIZEOF_NXFFS_BLOCK_HDR],
                                     blocksize);
              good = (blocksize == erasesize);
            }

          /* If the block is bad, attempt to re-write the block header
           * indicating a bad block (of course, if the block has failed,
           * this may not be possible, depending upon failure modes).
           */

          if (!good)
            {
              nxffs_blkinit(volume, blkptr, BLOCK_STATE_BAD);
              modified = true;
            }
        }

      /* If the erase block was modified, then re-write it */

      if (modified)
        {
          nxfrd = MTD_BWRITE(volume->mtd, lblock, volume->blkper,
                             volume->pack);
          if (nxfrd != volume->blkper)
            {
              ferr("ERROR: Write erase block %jd failed: %zd\n",
                   (intmax_t)lblock, nxfrd);
              return -EIO;
            }
        }
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_reformat
 *
 * Description:
 *   Erase and reformat the entire volume.  Verify each block and mark
 *   improperly erased blocks as bad.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume to be reformatted.
 *
 * Returned Value:
 *   Zero on success or a negated errno on a failure.  Failures will be
 *   returned n the case of MTD reported failures o.
 *   Nothing in the volume data itself will generate errors.
 *
 ****************************************************************************/

int nxffs_reformat(FAR struct nxffs_volume_s *volume)
{
  int ret;

  /* Erase and reformat the entire volume */

  ret = nxffs_format(volume);
  if (ret < 0)
    {
      ferr("ERROR: Failed to reformat the volume: %d\n", -ret);
      return ret;
    }

  /* Check for bad blocks */

  ret = nxffs_badblocks(volume);
  if (ret < 0)
    {
      ferr("ERROR: Bad block check failed: %d\n", -ret);
    }

  return ret;
}

/****************************************************************************
 * Name: nxffs_blkinit
 *
 * Description:
 *   Initialize an NXFFS block to the erased state with the specified block
 *   status.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume (needed for the blocksize).
 *   blkptr - Pointer to the logic block to initialize.
 *   state  - Either BLOCK_STATE_GOOD or BLOCK_STATE_BAD.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void nxffs_blkinit(FAR struct nxffs_volume_s *volume, FAR uint8_t *blkptr,
                   uint8_t state)
{
  FAR struct nxffs_block_s *blkhdr = (FAR struct nxffs_block_s *)blkptr;

  memset(blkptr, CONFIG_NXFFS_ERASEDSTATE, volume->geo.blocksize);
  memcpy(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE);
  blkhdr->state = state;
}
