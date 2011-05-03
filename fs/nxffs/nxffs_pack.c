/****************************************************************************
 * fs/nxffs/nxffs_pack.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
#include <assert.h>
#include <debug.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/
/* This structure supports access to one inode data stream */

struct nxffs_packstream_s
{
  struct nxffs_entry_s entry;      /* Described the inode header */
  off_t                fpos;       /* Current file position */
  off_t                blkoffset;  /* Offset to the current data block */
  uint16_t             blklen;     /* Size of this block */
  uint16_t             blkpos;     /* Position in block corresponding to fpos */
};

/* The structure supports the overall packing operation */

struct nxffs_pack_s
{
  /* These describe the source and destination streams */

  struct nxffs_packstream_s src;
  struct nxffs_packstream_s dest;

  /* These describe the state of the current contents of the (destination)
   * volume->pack buffer.
   */

  FAR uint8_t         *iobuffer;   /* I/O block start position */
  off_t                ioblock;    /* I/O block number */
  uint16_t             iooffset;   /* I/O block offset */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_packseek
 *
 * Description:
 *   Seek to the destination offset in the pack buffer
 *
 * Input Parameters:
 *   volume - The volume to be packed.
 *   pack   - The volume packing state structure.
 *   offset - The desired offset
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static void nxffs_packseek(FAR struct nxffs_volume_s *volume,
                           FAR struct nxffs_pack_s *pack, off_t offset)
{
  pack->ioblock  = offset / volume->geo.blocksize;
  pack->iooffset = offset - pack->ioblock * volume->geo.blocksize;
}

/****************************************************************************
 * Name: nxffs_packvalid
 *
 * Description:
 *   Check if the current destination block is valid.
 *
 * Input Parameters:
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   None
 *
 ****************************************************************************/

static inline bool nxffs_packvalid(FAR struct nxffs_pack_s *pack)
{
  FAR struct nxffs_block_s *blkhdr;

  blkhdr = (FAR struct nxffs_block_s *)pack->iobuffer;
  return (memcmp(blkhdr->magic, g_blockmagic, NXFFS_MAGICSIZE) == 0 &&
          blkhdr->state == BLOCK_STATE_GOOD);
}

/****************************************************************************
 * Name: nxffs_mediacheck
 *
 * Description:
 *   Verify that there is at least one valid block and at least one valid
 *   inode header on the media.  On successful return, the volume packing
 *   structure is initialized and contains the offset to the first valid
 *   inode header is returned.
 *
 * Input Parameters:
 *   volume - The volume to be packed.
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   The offset to the data area on the first valid block.  Zero is return
 *   if there are no valid blocks or if there are no valid inode headers
 *   after the first valid block.
 *
 ****************************************************************************/

static inline off_t nxffs_mediacheck(FAR struct nxffs_volume_s *volume,
                                     FAR struct nxffs_pack_s *pack)
{
  off_t froffset;
  int ret;

  /* Find the FLASH offset to the first valid block */

  volume->ioblock = 0;
  ret = nxffs_validblock(volume, &volume->ioblock);
  if (ret < 0)
   {
     /* No valid blocks?  Return offset zero. */

     return 0;
   }

  /* The offset to the free location to pack is then just after the block
   * header in this block.
   */

  volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
  froffset         = nxffs_iotell(volume);

  /* Get the offset to the first valid inode entry after this free offset */

  ret = nxffs_nextentry(volume, froffset, &pack->src.entry);
  if (ret < 0)
    {
      /* No valid entries on the media -- Return offset zero */

     return -ENOSPC;
    }

  /* Okay.. the start block and first entry have been found */

  return froffset;
}

/****************************************************************************
 * Name: nxffs_startblock
 *
 * Description:
 *   Find the position in FLASH memory where we should begin packing.  That
 *   position is the place where there is a gap between the last and the next
 *   valid inode.  On entry, the volume packing structure should be as it
 *   was initialized by nxffx_mediacheck.  on successful return, the volume
 *   packing state structure will be updated to begin the packing operation.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *   offset - location to return the pointer to first valid inode header.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int nxffs_startblock(FAR struct nxffs_volume_s *volume,
                                   FAR struct nxffs_pack_s *pack,
                                   off_t offset)
{
  struct nxffs_blkentry_s blkentry;
  off_t wasted;
  off_t nbytes;
  int ret;

  /* Loop until we find a gap of unused FLASH large enough to warrant the
   * compression.
   */

  for(;;)
    {
      /* Is there wasted space between the offset where the we could have
       * valid data and the offset to the beginning of the first valid
       * inode header?  NOTE:  The threshold check is not accurate, there
       * may or may not be intervening block headers making the separation
       * seem larger than it is.
       */

      DEBUGASSERT(pack->src.entry.hoffset >= offset);
      wasted = pack->src.entry.hoffset - offset;
      if (wasted > CONFIG_NXFFS_PACKTHRESHOLD)
        {
          /* This is where we must begin packing */

          memcpy(&pack->dest.entry, &pack->src.entry, sizeof(struct nxffs_entry_s));
          
          pack->dest.entry.hoffset = offset;
          pack->src.entry.name     = NULL;
          return OK;
        }

      /* Free the allocated memory in the entry */

      nxffs_freeentry(&pack->src.entry);

      /* Update the offset to the first byte at the end of the last data
       * block.
       */

      nbytes = 0;
      offset = pack->src.entry.doffset;

      while (nbytes < pack->src.entry.datlen)
        {
          /* Read the next data block header */

          ret = nxffs_nextblock(volume, offset, &blkentry);
          if (ret < 0)
            {
              fdbg("Failed to find next data block: %d\n", -ret);
              return ret;
            }

          /* Get the number of blocks and pointer to where the next
           * data block might lie.
           */

          nbytes += blkentry.datlen;
          offset  = blkentry.hoffset + SIZEOF_NXFFS_DATA_HDR + blkentry.datlen;
        }

      /* Make sure there is space at this location for an inode header */

      nxffs_ioseek(volume, offset);
      if (volume->iooffset + SIZEOF_NXFFS_INODE_HDR > volume->geo.blocksize)
        {
          /* No.. not enough space here. Find the next valid block */

          volume->ioblock++;
          ret = nxffs_validblock(volume, &volume->ioblock);
          if (ret < 0)
            {
               /* No valid blocks? Then there is nothing we can do.  Return
                * the end-of-flash indication.
                */

               return -ENOSPC;
            }

          volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
          offset = nxffs_iotell(volume);
        }

      /* Get the offset to the next valid inode entry */

      ret = nxffs_nextentry(volume, offset, &pack->src.entry);
      if (ret < 0)
        {
          /* No more valid inode entries.  Just return an end-of-flash error
           * indication.
           */

          return -ENOSPC;
        }
    }

  /* We won't get here */

  return -ENOSYS;
}

/****************************************************************************
 * Name: nxffs_packblock
 *
 * Description:
 *   Resume packing from the source stream into the newly identified
 *   destination block.
 *
 * Input Parameters:
 *   volume - The volume to be packed
 *   pack   - The volume packing state structure.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

static inline int nxffs_packblock(FAR struct nxffs_volume_s *volume,
                                  FAR struct nxffs_pack_s *pack)
{
  int ret;

  /* Are we currently processing a block from the source stream? */

  if (pack->src.blkoffset)
    {
      /* No, start with the first data block */

      pack->src.blkoffset = pack->src.entry.doffset;
      pack->src.fpos      = 0;
      pack->src.blkpos    = 0;

      /* Seek to the data block header, read and verify the block header */

      ret = nxffs_rdblkhdr(volume, pack->src.blkoffset, &pack->src.blklen);
      if (ret < 0)
        {
          fdbg("Failed to verify the data block header: %d\n", -ret);
          return ret;
        }
    }

  /* Loop, transferring data from the source block to the destination pack
   * buffer until either (1) the source stream is exhausted, (2) the destination
   * block is full, or (3) an error occurs.
   */

  for (;;)
    {
       /* Determine how much data is available in the dest pack buffer */

       /* Dermined how much data is available in the src data block */

       /* Transfer the data */
#warning "Missing logic"
       return -ENOSYS;
    }

  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_pack
 *
 * Description:
 *   Pack and re-write the filesystem in order to free up memory at the end
 *   of FLASH.
 *
 * Input Parameters:
 *   volume - The volume to be packed.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

int nxffs_pack(FAR struct nxffs_volume_s *volume)
{
  struct nxffs_pack_s pack;
  off_t iooffset;
  off_t eblock;
  off_t block0;
  off_t block;
  int i;
  int ret;

  /* Get the offset to the first valid inode entry */

  iooffset = nxffs_mediacheck(volume, &pack);
  if (iooffset == 0)
    {
      /* Offset zero is only returned if no valid blocks were found on the
       * FLASH media or if there are no valid inode entries on the FLASH after
       * the first valid block.  In this case, the media needs to be re-
       * formatted.
       */

      return nxffs_reformat(volume);
    }

  /* There is a valid format and valid inodes on the media.. setup up to
   * begin the packing operation.
   */

  ret = nxffs_startblock(volume, &pack, iooffset);
  if (ret < 0)
    {
      /* This is a normal situation if the volume is full */

      if (ret == -ENOSPC)
        {
          return OK;
        }
      else
        {
          fvdbg("Failed to find a packing position: %d\n", -ret);
          return ret;
        }
    }

  /* Otherwise, begin pack at this src/dest block combination.  Get the erase
   * block associated with the destination header offset.
   */

  nxffs_packseek(volume, &pack, pack.dest.entry.hoffset);

  for (eblock = pack.ioblock / volume->blkper;
       eblock < volume->geo.neraseblocks;
       eblock++)
    {
      /* Read the erase block into the pack buffer. */

      block0 = eblock * volume->blkper;
      ret = MTD_BREAD(volume->mtd, block0, volume->blkper, volume->pack);
      if (ret < 0)
        {
          fdbg("Failed to read erase block %d: %d\n", eblock, -ret);
          goto errout_with_pack;
        }

      /* Pack each I/O block */

      for (i = 0, block = block0, pack.iobuffer = volume->pack;
           i < volume->blkper;
           i++, block++, pack.iobuffer += volume->geo.blocksize)
        {
           /* The first time here, the ioblock may point to an offset into
            * the erase block.  We just need to skip over those cases.
            */

           if (block >= pack.ioblock)
              {
                pack.ioblock = block;

                /* Check if this is a valid block */

                if (nxffs_packvalid(&pack))
                  {
                     /* Yes.. pack data into this block */

                     ret = nxffs_packblock(volume, &pack);
                     if (ret < 0)
                       {
                         fdbg("Failed to pack into block %d: %d\n", block, ret);
                         goto errout_with_pack;
                       }
                   }
              }
         }

      /* Write the packed I/O block to FLASH */

      ret = MTD_BWRITE(volume->mtd, block0, volume->blkper, volume->pack);
      if (ret < 0)
        {
          fdbg("Failed to write erase block %d: %d\n", eblock, -ret);
          goto errout_with_pack;
        }
    }

errout_with_pack:
  nxffs_freeentry(&pack.src.entry);
  nxffs_freeentry(&pack.dest.entry);
  return ret;
}
