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
  struct nxffs_packstream_s src;   /* Describes the src stream */
  struct nxffs_packstream_s dest;  /* Describes the destination stream */
};

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_startblock
 *
 * Description:
 *   Find the position in FLASH memory where we should begin packing.  That
 *   position is the place where there is a gap between the last and the next
 *   valid inode.
 *
 * Input Parameters:
 *   volume - The volume to be packed.
 *
 * Returned Values:
 *   Zero on success; Otherwise, a negated errno value is returned to
 *   indicate the nature of the failure.
 *
 ****************************************************************************/

static int nxffs_startblock(FAR struct nxffs_volume_s *volume,
                            FAR struct nxffs_pack_s *pack)
{
  struct nxffs_blkentry_s blkentry;
  off_t wasted;
  off_t offset;
  off_t nbytes;
  int ret;

  memset(pack, 0, sizeof(struct nxffs_pack_s));

  /* Find the FLASH offset to the first valid block */

  volume->ioblock = 0;
  ret = nxffs_validblock(volume, &volume->ioblock);
  if (ret < 0)
   {
     /* No valid blocks? */

     return nxffs_reformat(volume);
   }

  volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
  offset = nxffs_iotell(volume);

  /* Loop until we find a gap of unused FLASH large enough to warrant the
   * compression.
   */

  for(;;)
    {
      /* Get the offset to the first valid inode entry */

      ret = nxffs_nextentry(volume, offset, &pack->src.entry);
      if (ret < 0)
        {
          /* No valid entries -- reformat the flash */

          return nxffs_reformat(volume);
        }

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
          /* Find the next valid block */

          volume->ioblock++;
          ret = nxffs_validblock(volume, &volume->ioblock);
          if (ret < 0)
            {
               /* No valid blocks? Then there is nothing we can do */

               return -ENOSPC;
            }

          volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
          offset = nxffs_iotell(volume);
        }
    }

  /* We won't get here */

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
  int ret;

  /* Get the offset to the first valid inode entry */

  ret = nxffs_startblock(volume, &pack);
  if (ret < 0)
    {
      fdbg("Failed to find a start block: %d\n", -ret);
      return ret;
    }

  /* A special case is where the entire FLASH needs to be reformatted.  In
   * this case, the source and destination offsets will both be zero.
   */

  if (pack.src.entry.hoffset == 0 && pack.dest.entry.hoffset == 0)
    {
      return OK;
    }

  /* Otherwise, begin pack at this src/dest block combination */

# warning "Missing logic"
  return -ENOSYS;
}
