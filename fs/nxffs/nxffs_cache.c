/****************************************************************************
 * fs/nxffs/nxffs_cache.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mtd.h>

#include "nxffs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nxffs_rdcache
 *
 * Description:
 *   Read one or more logical blocks into the volume block cache memory.
 *
 * Input Parameters:
 *   volume - Describes the current volume
 *   block  - The first logical block to read
 *   nblocks - The number of logical blocks to be read.
 *
 * Returned Value:
 *   Negated errnos are returned only in the case of MTD reported failures.
 *   Nothing in the volume data itself will generate errors.
 *
 ****************************************************************************/

int nxffs_rdcache(FAR struct nxffs_volume_s *volume, off_t block,
                  uint8_t nblocks)
{
  size_t nxfrd;

  DEBUGASSERT(nblocks <= volume->blkper);

  /* Check if the requested data is already in the cache */

  if (block != volume->cblock || nblocks <= volume->ncached)
    {
      /* Read the specified blocks into cache */

      nxfrd = MTD_BREAD(volume->mtd, block, nblocks, volume->cache);
      if (nxfrd != nblocks)
        {
          fdbg("Read block %d-%d failed: %d\n",
                block, block + nblocks -1, nxfrd);
          return -EIO;
        }

      /* Remember what is in the cache */

      volume->cblock  = block;
      volume->ncached = nblocks;
    }
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
 * Name: nxffs_getc
 *
 * Description:
 *   Get the next byte from FLASH.  This function allows the data in the
 *   formatted FLASH blocks to be read as a continuous byte stream, skipping
 *   over bad blocks and block headers as necessary.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.  The paramters ioblock and iooffset
 *     in the volume structure determine the behavior of nxffs_getc().
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno indicating the
 *   nature of the failure.
 *   
 ****************************************************************************/

int nxffs_getc(FAR struct nxffs_volume_s *volume)
{
  int ret;

  /* Loop to skip over bad blocks */
 
  do
    {
      /* Check if we have read past the current block */

      if (volume->iooffset >= volume->geo.blocksize)
        {
          volume->ioblock++;
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
          fdbg("Failed to read valid data into cache: %d\n", ret);
          return ret;
        }
    }
  while (ret == -ENOENT);

  /* Return the the character at this offset.  Note that on return,
   * iooffset could point to the byte outside of the current block.
   */

  ret = (int)volume->cache[volume->iooffset];
  volume->iooffset++;
  return ret;
}

/****************************************************************************
 * Name: nxffs_rddata
 *
 * Description:
 *   Read a sequence of data bytes from the FLASH memory.  This function
 *   allows the data in the formatted FLASH blocks to be read as a continuous\
 *   byte stream, skipping over bad blocks and block headers as necessary.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume.  The paramters ioblock and iooffset
 *     in the volume structure determine the behavior of nxffs_getc().
 *
 * Returned Value:
 *   The number of bytes read is returned on success.  Otherwise, a negated
 *   errno indicating the nature of the failure.
 *
 ****************************************************************************/

ssize_t nxffs_rddata(FAR struct nxffs_volume_s *volume,
                     FAR uint8_t *buffer, size_t buflen)
{
  size_t nbytes;
  int ch;

  for (nbytes = buflen; nbytes > 0; nbytes--)
    {
      /* Read the next character (which could be in the next block) */

      ch = nxffs_getc(volume);
      if (ch < 0)
        {
          fdbg("Failed to read byte: %d\n", -ch);
          return ch;
        }

      /* Add the next character to the user buffer */

      *buffer++ = (uint8_t)ch;
    }

  return buflen;
}

