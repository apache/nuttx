/****************************************************************************
 * fs/spiffs/src/spiffs_mtd.c
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
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_SPIFFS_MTDDUMP
static inline void spiffs_mtd_dump(FAR const char *msg,
                                   FAR const uint8_t *buffer,
                                   unsigned int buflen)
{
  lib_dumpbuffer(msg, buffer, buflen);
}

#else
#  define spiffs_mtd_dump(m,b,l)
#endif

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
  size_t remaining;
  ssize_t ret;
  off_t blkmask;
  off_t blkstart;
  off_t blkend;
  int16_t blksize;
  int16_t nblocks;
  int16_t blkoffset;

  spiffs_mtdinfo("offset=%ld len=%lu\n", (long)offset, (unsigned long)len);
  spiffs_mtd_dump("Writing", src, len);

  DEBUGASSERT(fs != NULL && fs->mtd != NULL && src != NULL && len > 0);

  remaining = len;

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
       * blkend    - End block number (aligned down)
       */

      blksize   = fs->geo.blocksize;
      blkmask   = blksize - 1;
      blkoffset = offset & blkmask;
      blkstart  = offset / blksize;
      blkend    = (offset + len - 1) / blksize;

      spiffs_mtdinfo("blkoffset=%d blkstart=%ld blkend=%ld\n",
                     blkoffset, (long)blkstart, (long)blkend);

      /* Check if we have to do a read-modify-write on the first block.  We
       * need to do this if the blkoffset is not zero.  In that case we need
       * write only the data at the end of the block.
       */

      if (blkoffset != 0)
        {
          FAR uint8_t *wptr = fs->mtd_work;
          int16_t maxcopy;
          int16_t nbytes;

          ret = MTD_BREAD(fs->mtd, blkstart, 1, wptr);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BREAD() failed: %zd\n", ret);
              return (ssize_t)ret;
            }

          spiffs_mtd_dump("Read leading partial block", wptr, blksize);

          /* Copy the data in place */

          maxcopy = blksize - blkoffset;
          nbytes  = MIN(remaining, maxcopy);

          spiffs_mtdinfo("Leading partial block: "
                         "blkstart=%jd blkoffset=%d nbytes=%d\n",
                         (intmax_t)blkstart, blkoffset, nbytes);

          memcpy(&wptr[blkoffset], src, nbytes);

          /* Write back the modified block */

          spiffs_mtd_dump("Write leading partial block", wptr, blksize);

          ret = MTD_BWRITE(fs->mtd, blkstart, 1, wptr);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BWRITE() failed: %zd\n", ret);
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
      if (remaining > 0 && (remaining & blkmask) == 0)
        {
          /* The final block is a complete transfer */

          nblocks++;
        }

      spiffs_mtdinfo("Whole blocks=%d blkstart=%lu remaining=%lu\n",
                     nblocks, (unsigned long)blkstart,
                     (unsigned long)remaining);

      if (nblocks > 0)
        {
          spiffs_mtd_dump("Write whole blocks", src, blksize * nblocks);

          ret = MTD_BWRITE(fs->mtd, blkstart, nblocks, src);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BWRITE() failed: %zd\n", ret);
              return (ssize_t)ret;
            }

          src       += (remaining & ~blkmask);
          remaining  = (remaining & blkmask);

          /* The real end block is the next block (if remainder > 0) */

          blkend++;
        }

      /* Check if we need to perform a read-modify-write on the final block.
       * If the remaining bytes to write is less then a full block, then we
       * need write only the data at the beginning of the block.
       */

      if (remaining > 0)
        {
          ret = MTD_BREAD(fs->mtd, blkend, 1, fs->mtd_work);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BREAD() failed: %zd\n", ret);
              return (ssize_t)ret;
            }

          spiffs_mtd_dump("Read trailing partial block",
                          fs->mtd_work, blksize);

          /* Copy the data in place */

          spiffs_mtdinfo("Trailing partial block: "
                         "blkend=%ld remaining=%lu\n",
                         (long)blkend, (unsigned long)remaining);

          memcpy(fs->mtd_work, src, remaining);

          /* Write back the modified block */

          spiffs_mtd_dump("Write trailing partial block",
                          fs->mtd_work, blksize);

          ret = MTD_BWRITE(fs->mtd, blkend, 1, fs->mtd_work);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BWRITE() failed: %zd\n", ret);
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
#ifdef CONFIG_SPIFFS_MTDDUMP
  FAR uint8_t *saved_dest = dest;
#endif
  size_t remaining;
  ssize_t ret;
  off_t blkmask;
  off_t blkstart;
  off_t blkend;
  int16_t blksize;
  int16_t nblocks;
  int16_t blkoffset;

  spiffs_mtdinfo("offset=%ld len=%lu\n", (long)offset, (unsigned long)len);

  DEBUGASSERT(fs != NULL && fs->mtd != NULL && dest != NULL && len > 0);

  remaining = len;

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
       * blkend    - End block number (aligned down)
       */

      blksize   = fs->geo.blocksize;
      blkmask   = blksize - 1;
      blkoffset = offset & blkmask;
      blkstart  = offset / blksize;
      blkend    = (offset + len - 1) / blksize;

      spiffs_mtdinfo("blkoffset=%d blkstart=%ld blkend=%ld\n",
                     blkoffset, (long)blkstart, (long)blkend);

      /* Check if we have to do a partial read on the first block.  We
       * need to do this if the blkoffset is not zero.  In that case we need
       * read only the data at the end of the block.
       */

      if (blkoffset != 0)
        {
          FAR uint8_t *wptr = fs->mtd_work;
          size_t maxcopy;
          ssize_t nbytes;

          ret = MTD_BREAD(fs->mtd, blkstart, 1, wptr);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BREAD() failed: %zd\n", ret);
              return (ssize_t)ret;
            }

          spiffs_mtd_dump("Read leading partial block", wptr, blksize);

          /* Copy the data from the block */

          maxcopy = blksize - blkoffset;
          nbytes  = MIN(remaining, maxcopy);

          spiffs_mtdinfo("Leading partial block: "
                         "blkstart=%jd blkoffset=%d nbytes=%zd\n",
                         (intmax_t)blkstart, blkoffset, nbytes);

          memcpy(dest, &wptr[blkoffset], nbytes);

          /* Update block numbers and counts */

          blkoffset  = 0;
          remaining -= nbytes;
          dest      += nbytes;
          blkstart++;
        }

      /* Read all intervening complete blocks... all at once */

      nblocks = blkend - blkstart;
      if (remaining > 0 && (remaining & blkmask) == 0)
        {
          /* The final block is a complete transfer */

          nblocks++;
        }

      spiffs_mtdinfo("Whole blocks=%d blkstart=%lu remaining=%lu\n",
                     nblocks, (unsigned long)blkstart,
                     (unsigned long)remaining);

      if (nblocks > 0)
        {
          ret = MTD_BREAD(fs->mtd, blkstart, nblocks, dest);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BREAD() failed: %zd\n", ret);
              return (ssize_t)ret;
            }

          spiffs_mtd_dump("Read whole blocks", dest, blksize * nblocks);

          dest      += (remaining & ~blkmask);
          remaining  = (remaining & blkmask);

          /* The real end block is the next block (if remainder > 0) */

          blkend++;
        }

      /* Check if we need to perform a partial read on the final block.
       * If the remaining bytes to write is less then a full block, then we
       * need write only the data at the beginning of the block.
       */

      if (remaining > 0)
        {
          ret = MTD_BREAD(fs->mtd, blkend, 1, fs->mtd_work);
          if (ret < 0)
            {
              ferr("ERROR: MTD_BREAD() failed: %zd\n", ret);
              return (ssize_t)ret;
            }

          /* Copy the data from the block */

          spiffs_mtdinfo("Trailing partial block: "
                         "blkend=%ld remaining=%lu\n",
                         (long)blkend, (unsigned long)remaining);
          spiffs_mtd_dump("Read trailing partial block",
                          fs->mtd_work, blksize);

          memcpy(dest, fs->mtd_work, remaining);
        }
    }

  spiffs_mtd_dump("Read", saved_dest, len);
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
 *   offset - The byte offset to begin erasing
 *   len    - The number of bytes to erase
 *
 * Returned Value:
 *   On success, the number of bytes erased is returned.  On failure, a
 *   negated errno value is returned.
 *
 ****************************************************************************/

ssize_t spiffs_mtd_erase(FAR struct spiffs_s *fs, off_t offset, size_t len)
{
  int16_t erasesize;
  off_t eblkstart;
  off_t eblkend;
  ssize_t nerased;

  spiffs_mtdinfo("offset=%ld len=%lu\n", (long)offset, (unsigned long)len);

  DEBUGASSERT(fs != NULL && fs->mtd != NULL);

  /* We will have to do block read(s)
   *
   * erasesize  - Size of one erase block.
   * eblkstart  - Start erase block number (aligned down)
   * eblkend    - End block number (aligned down)
   */

  erasesize = fs->geo.erasesize;
  eblkstart = offset / erasesize;
  eblkend   = (offset + len) / erasesize;

  /* Must be even multiples of the erase block size */

  DEBUGASSERT(offset == erasesize * eblkstart);
  DEBUGASSERT(len    == erasesize * eblkend - offset);

  nerased = MTD_ERASE(fs->mtd, eblkstart, eblkend - eblkstart);
  if (nerased < 0)
    {
      ferr("ERROR: MTD_ERASE() failed: %zd\n", nerased);
      return nerased;
    }

  return erasesize * nerased;
}
