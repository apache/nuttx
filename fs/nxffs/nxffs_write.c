/****************************************************************************
 * fs/nxffs/nxffs_write.c
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
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/fs.h>
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
 * Name: nxffs_write
 *
 * Description:
 *   This is an implementation of the NuttX standard file system write
 *   method.
 *
 ****************************************************************************/

ssize_t nxffs_write(FAR struct file *filep, FAR const char *buffer, size_t buflen)
{
  FAR struct nxffs_volume_s *volume;
  FAR struct nxffs_wrfile_s *wrfile;
  ssize_t ret;

  fvdbg("Write %d bytes to offset %d\n", buflen, filep->f_pos);

  /* Sanity checks */

  DEBUGASSERT(filep->f_priv != NULL && filep->f_inode != NULL);

  /* Recover the open file state from the struct file instance */

  wrfile = (FAR struct nxffs_wrfile_s *)filep->f_priv;

  /* Recover the volume state from the open file */

  volume = (FAR struct nxffs_volume_s *)filep->f_inode->i_private;
  DEBUGASSERT(volume != NULL);

  /* Get exclusive access to the volume.  Note that the volume exclsem
   * protects the open file list.
   */

  ret = sem_wait(&volume->exclsem);
  if (ret != OK)
    {
      ret = -errno;
      fdbg("sem_wait failed: %d\n", ret);
      goto errout;
    }

  /* Check if the file was opened with write access */

#ifdef CONFIG_DEBUG
  if ((wrfile->ofile.mode & O_WROK) == 0)
    {
      fdbg("File not open for write access\n");
      ret = -EACCES;
      goto errout_with_semaphore;
    }
#endif

  /* Seek to the current file offset */

  nxffs_ioseek(volume, wrfile->dathdr + wrfile->wrlen);

  /* Write data to that file offset */

  ret = nxffs_wrdata(volume, (FAR const uint8_t*)buffer, buflen);
  if (ret > 0)
    {
      /* Update the file offset */

      filep->f_pos += ret;
    }
#warning "Add check for block full"

errout_with_semaphore:
  sem_post(&volume->exclsem);
errout:
  return ret;
}

/****************************************************************************
 * Name: nxffs_wrreserve
 *
 * Description:
 *   Find a valid location for a file system object of 'size'.  A valid
 *   location will have these properties:
 *
 *   1. It will lie in the free flash region.
 *   2. It will have enough contiguous memory to hold the entire object
 *   3. The memory at this location will be fully erased.
 *
 *   This function will only perform the checks of 1) and 2).  The
 *   end-of-filesystem offset, froffset, is update past this memory which,
 *   in effect, reserves the memory.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   size - The size of the object to be reserved.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.  Of special interest
 *   the return error of -ENOSPC which means that the FLASH volume is
 *   full and should be repacked.
 *
 *   On successful return the following are also valid:
 *
 *   volume->ioblock - Read/write block number of the block containing the
 *     candidate oject position
 *   volume->iooffset - The offset in the block to the candidate object
 *     position.
 *   volume->froffset - Updated offset to the first free FLASH block after
 *     the reserved memory.
 *
 ****************************************************************************/

int nxffs_wrreserve(FAR struct nxffs_volume_s *volume, size_t size)
{
  off_t offset;
  int ret;

  /* Seek to the beginning of the free FLASH region */

  offset = volume->froffset;
  nxffs_ioseek(volume, offset);

  /* Check for a seek past the end of the volume */

  if (volume->ioblock >= volume->nblocks)
    {
      /* Return -ENOSPC to indicate that the volume is full */

      return -ENOSPC;
    }

  /* Make sure that there is space there to hold the entire object */

  DEBUGASSERT(volume->iooffset >= SIZEOF_NXFFS_BLOCK_HDR);
  if (volume->iooffset + size > volume->geo.blocksize)
    {
      /* We will need to skip to the next block.  But first, check if we are
       * already at the final block.
       */
 
      if (volume->ioblock + 1 >= volume->geo.neraseblocks)
        {
          /* Return -ENOSPC to indicate that the volume is full */

          fdbg("No space in last block\n");
          return -ENOSPC;
        }

      /* This is not the last block in the volume, so just seek to the
       * beginning of the next, valid block.
       */

      volume->ioblock++;
      ret = nxffs_validblock(volume, &volume->ioblock);
      if (ret < 0)
        {
          fdbg("No more valid blocks\n");
          return ret;
        }

      volume->iooffset = SIZEOF_NXFFS_BLOCK_HDR;
      offset           = volume->ioblock * volume->geo.blocksize + SIZEOF_NXFFS_BLOCK_HDR;
    }

  /* Update the pointer to the first next free FLASH memory -- reserving this
   * block of memory.
   */

  volume->froffset = offset + size;
  return OK;
}

/****************************************************************************
 * Name: nxffs_wrverify
 *
 * Description:
 *   Find a valid location for the object.  A valid location will have
 *   these properties:
 *
 *   1. It will lie in the free flash region.
 *   2. It will have enough contiguous memory to hold the entire header
 *      (excluding the file name which may lie in the next block).
 *   3. The memory at this location will be fully erased.
 *
 *   This function will only perform the check 3). On entry it assumes the
 *   following settings (left by nxffs_wrreserve()):
 *
 *   volume->ioblock - Read/write block number of the block containing the
 *     candidate oject position
 *   volume->iooffset - The offset in the block to the candidate object
 *     position.
 *
 * Input Parameters:
 *   volume - Describes the NXFFS volume
 *   size - The size of the object to be verifed.
 *
 * Returned Value:
 *   Zero is returned on success.  Otherwise, a negated errno value is
 *   returned indicating the nature of the failure.  Of special interest
 *   the return error of -ENOSPC which means that the FLASH volume is
 *   full and should be repacked.
 *
 *   On successful return the following are also valid:
 *
 *   volume->ioblock - Read/write block number of the block containing the
 *     verified object position
 *   volume->iooffset - The offset in the block to the verified object
 *     position.
 *   volume->froffset - Updated offset to the first free FLASH block.
 *
 ****************************************************************************/

int nxffs_wrverify(FAR struct nxffs_volume_s *volume, size_t size)
{
  uint16_t iooffset;
  int nerased;
  int ret;
  int i;

  /* Search to the very last block in the volume if we have to */

  while (volume->ioblock < volume->nblocks)
    {
      /* Make sure that the block is in memory */
 
      ret = nxffs_rdcache(volume, volume->ioblock, 1);
      if (ret < 0)
        {
          fdbg("nxffsx_rdcache failed: %d\n", -ret);
          return ret;
        }

      /* Search to the very end of this block if we have to */

      iooffset = volume->iooffset;
      nerased  = 0;

      for (i = volume->iooffset; i <= volume->geo.blocksize - size; i++)
        {
          /* Is this byte erased? */

          if (volume->cache[i] == CONFIG_NXFFS_ERASEDSTATE)
            {
              /* Yes.. increment the count of contiguous, erased bytes */

              nerased++;

              /* Is the whole header memory erased? */

              if (nerased >= size)
                {
                   /* Yes.. this this is where we will put the object */

                   off_t offset = volume->ioblock * volume->geo.blocksize + iooffset;

                   /* Update the free flash offset and return success */

                   volume->froffset = offset + size;
                   return OK;
                }
            }

          /* This byte is not erased!  (It should be unless the block is bad) */

          else
            {
              nerased  = 0;
              iooffset = volume->iooffset + 1;
            }
        }

      /* If we get here, then we have looked at every byte in the the block
       * and did not find any sequence of erased bytes long enough to hold
       * the object.  Skip to the next, valid block.
       */

      volume->ioblock++;
      ret = nxffs_validblock(volume, &volume->ioblock);
      if (ret < 0)
        {
          fdbg("No more valid blocks\n");
          return ret;
        }

      volume->iooffset = size;
      volume->froffset = volume->ioblock * volume->geo.blocksize + size;
    }

  /* Return -ENOSPC if there is no erased memory left in the volume for
   * the object.
   */

  fdbg("Not enough memory left to hold the file header\n");
  return -ENOSPC;
}

/****************************************************************************
 * Name: nxffs_wrblkhdr
 *
 * Description:
 *   Write the block header information.  This is done (1) whenever the end-
 *   block is encountered and (2) also when the file is closed in order to
 *   flush the final block of data to FLASH.
 *
 * Input Parameters:
 *   volume - Describes the state of the NXFFS volume
 *   wrfile - Describes the state of the open file
 *
 * Returned Value:
 *   Zero is returned on success; Otherwise, a negated errno value is
 *   returned to indicate the nature of the failure.
 *
 ****************************************************************************/

int nxffs_wrblkhdr(FAR struct nxffs_volume_s *volume,
                   FAR struct nxffs_wrfile_s *wrfile)
{
#warning "Missing logic"
  return -ENOSYS;
}

