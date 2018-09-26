/****************************************************************************
 * fs/spiffs.h/spiffs_mtd.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/mtd/mtd.h>

#include "spiffs.h"
#include "spiffs_mtd.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_mtd_write
 *
 * Description:
 *   Write data to FLASH memory
 *
 * Input Parameters:
 *   fs     - A reference to the volume structure
 *   offset - The byte offset to write to
 *   len    - The number of bytes to write
 *   src    - A reference to the bytes to be written
 *
 * Returned Value:
 *   On success, the number of bytes written is returned.  On failure, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

ssize_t spiffs_mtd_write(FAR struct spiffs_s *fs, off_t offset, size_t len,
                         FAR const uint8_t *src)
{
  int16_t blksize;
  size_t blkoffset;
  size_t remaining;
  ssize_t nblocks;
  ssize_t ret;
  off_t blkmask;
  off_t blkstart;
  off_t blkend;

  DEBUGASSERT(fs != NULL && fs->mtd != NULL && src != NULL);

#ifdef CONFIG_MTD_BYTE_WRITE
  /* Try to do the byte write */

  ret = MTD_WRITE(fs->mtd, offset, len, src);
  if (ret < 0)
#endif
    {
      /* We will have to do block write(s)
       *
       * blksize   - Size of one block.  We assume that the block size is a
       *             power of two.
       * blkmask   - Mask that isolates fractional block bytes.
       * blkoffset - The offset of data in the first block written
       * blkstart  - Start block number (aligned down)
       * blkend    - End block number (aligned up)
       */

      blksize   = fs->geo.blocksize;
      blkmask   = blksize - 1;
      blkoffset = blksize & ~blkmask;
      blkstart  = offset / blksize;
      blkend    = (offset + len + blkmask) / blksize;

      /* Check if we have to do a read-modify-write on the first block.  We
       * need to do this if the blkoffset is not zero.  In that case we need
       * write only the data at the end of the block.
       */

      if (blkoffset != 0)
        {
          FAR uint8_t *wptr = fs->work;
          size_t maxcopy;
          ssize_t nbytes;

#warning "REVISIT: is fs->work available here?"
          ret = MTD_BREAD(fs->mtd, blkstart, 1, wptr);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BREAD() failed: %d\n", ret);
              return (ssize_t)ret;
            }

          /* Copy the data in place */

          maxcopy = blksize - blkoffset;
          nbytes  = MIN(remaining, maxcopy);

          memcpy(&wptr[blkoffset], src, nbytes);

          /* Write back the modified block */

          ret = MTD_BWRITE(fs->mtd, blkstart, 1, wptr);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BWRITE() failed: %d\n", ret);
              return (ssize_t)ret;
            }

          /* Update block numbers and counts */

          blkoffset  = 0;
          remaining -= nbytes;
          src       += nbytes;
          blkstart++;
        }

      /* Write all intervening complete blocks... all at once */

      nblocks = blkend - blkstart;

      if ((remaining & blkmask) == 0)
        {
          nblocks++;  /* Include the final block */
        }

      ret = MTD_BWRITE(fs->mtd, blkstart, nblocks, src);
      if (ret < 0)
        {
          ferr("ERROR: MTD_BWRITE() failed: %d\n", ret);
          return (ssize_t)ret;
        }

      src       += (remaining & ~blkmask);
      remaining  = (remaining & blkmask);

      /* Check if we need to perform a read-modify-write on the final block.
       * If the remaining bytes to write is less then a full block, then we
       * need write only the data at the beginning of the block.
       */

      if (remaining > 0)
        {
#warning "REVISIT: is fs->work available here?"
          ret = MTD_BREAD(fs->mtd, blkend, 1, fs->work);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BREAD() failed: %d\n", ret);
              return (ssize_t)ret;
            }

          /* Copy the data in place */

          memcpy(fs->work, src, remaining);

          /* Write back the modified block */

          ret = MTD_BWRITE(fs->mtd, blkend, 1, fs->work);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BWRITE() failed: %d\n", ret);
              return (ssize_t)ret;
            }
        }
    }

  return (ssize_t)len;
}

/****************************************************************************
 * Name: spiffs_mtd_read
 *
 * Description:
 *   Read data from FLASH memory
 *
 * Input Parameters:
 *   fs     - A reference to the volume structure
 *   offset - The byte offset to read from
 *   len    - The number of bytes to read
 *   dest   - The user provide location to store the bytes read from FLASH.
 *
 * Returned Value:
 *   On success, the number of bytes read is returned.  On failure, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

ssize_t spiffs_mtd_read(FAR struct spiffs_s *fs, off_t offset, size_t len,
                        FAR uint8_t *dest)
{
  int16_t blksize;
  int16_t nblocks;
  size_t blkoffset;
  size_t remaining;
  ssize_t ret;
  off_t blkmask;
  off_t blkstart;
  off_t blkend;

  DEBUGASSERT(fs != NULL && fs->mtd != NULL && dest != NULL);

  /* Try to do the byte read */

  ret = MTD_READ(fs->mtd, offset, len, dest);
  if (ret < 0)
    {
      /* We will have to do block read(s)
       *
       * blksize   - Size of one block.  We assume that the block size is a
       *             power of two.
       * blkmask   - Mask that isolates fractional block bytes.
       * blkoffset - The offset of data in the first block read.
       * blkstart  - Start block number (aligned down)
       * blkend    - End block number (aligned up)
       */

      blksize   = fs->geo.blocksize;
      blkmask   = blksize - 1;
      blkoffset = blksize & ~blkmask;
      blkstart  = offset / blksize;
      blkend    = (offset + len + blkmask) / blksize;

      /* Check if we have to do a partial read on the first block.  We
       * need to do this if the blkoffset is not zero.  In that case we need
       * read only the data at the end of the block.
       */

      if (blkoffset != 0)
        {
          FAR uint8_t *wptr = fs->work;
          size_t maxcopy;
          ssize_t nbytes;

#warning "REVISIT: is fs->work available here?"
          ret = MTD_BREAD(fs->mtd, blkstart, 1, wptr);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BREAD() failed: %d\n", ret);
              return (ssize_t)ret;
            }

          /* Copy the data from the block */

          maxcopy = blksize - blkoffset;
          nbytes  = MIN(remaining, maxcopy);

          memcpy(dest, &wptr[blkoffset], nbytes);

          /* Update block numbers and counts */

          blkoffset  = 0;
          remaining -= nbytes;
          dest      += nbytes;
          blkstart++;
        }

      /* Read all intervening complete blocks... all at once */

      nblocks = blkend - blkstart;

      if ((remaining & blkmask) == 0)
        {
          nblocks++;  /* Include the final block */
        }

      ret = MTD_BREAD(fs->mtd, blkstart, nblocks, dest);
      if (ret < 0)
        {
          ferr("ERROR: MTD_BREAD() failed: %d\n", ret);
          return (ssize_t)ret;
        }

      dest      += (remaining & ~blkmask);
      remaining  = (remaining & blkmask);

      /* Check if we need to perform a partial read on the final block.
       * If the remaining bytes to write is less then a full block, then we
       * need write only the data at the beginning of the block.
       */

      if (remaining > 0)
        {
#warning "REVISIT: is fs->work available here?"
          ret = MTD_BREAD(fs->mtd, blkend, 1, fs->work);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BREAD() failed: %d\n", ret);
              return (ssize_t)ret;
            }

          /* Copy the data from the block */

          memcpy(dest, fs->work, remaining);
        }
    }

  return (ssize_t)len;
}

/****************************************************************************
 * Name: spiffs_mtd_erase
 *
 * Description:
 *   Read data from FLASH memory
 *
 * Input Parameters:
 *   fs     - A reference to the volume structure
 *   offset - The physical offset to begin erasing
 *   len    - The number of bytes to erase
 *
 * Returned Value:
 *   On success, the number of bytes erased is returned.  On failure, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

ssize_t spiffs_mtd_erase(FAR struct spiffs_s *fs, off_t offset, size_t len)
{
  int16_t blksize;
  off_t blkstart;
  off_t blkend;

  DEBUGASSERT(fs != NULL && fs->mtd != NULL);

#warning REVISIT:  What are units of offset and len?

  blksize  = fs->geo.erasesize;
  DEBUGASSERT(offset = offset % blksize);
  DEBUGASSERT(len    = len    % blksize);

  blkstart = offset / blksize;                       /* Truncates to floor */
  blkend   = (offset + len + blksize - 1) / blksize; /* Rounds up to ceil */

  return MTD_ERASE(fs->mtd, blkstart, blkend - blkstart);
}
