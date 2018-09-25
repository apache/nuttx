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
#include <assert.h>
#include <errno.h>

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
 *   fs    - A reference to the volume structure
 *   offset - The physical offset to write to
 *   len   - The number of bytes to write
 *   src   - A reference to the bytes to be written
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
  off_t blkstart;
  off_t blkend;
  ssize_t ret;

  DEBUGASSERT(fs != NULL && fs->mtd != NULL && src != NULL);

#warning REVISIT:  What are units of offset and len?

#ifdef CONFIG_MTD_BYTE_WRITE
  ret = MTD_WRITE(fs->mtd, offset, len, src);
  if (ret < 0)
#endif
    {
      blksize  = fs->geo.blocksize;
      DEBUGASSERT(offset = offset % blksize);
      DEBUGASSERT(len    = len    % blksize);

      blkstart = offset / blksize;                       /* Truncates to floor */
      blkend   = (offset + len + blksize - 1) / blksize; /* Rounds up to ceil */

      ret      = MTD_BWRITE(fs->mtd, blkstart, blkend - blkstart, src);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_mtd_read
 *
 * Description:
 *   Read data from FLASH memory
 *
 * Input Parameters:
 *   fs     - A reference to the volume structure
 *   offset - The physical offset to read from
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
  off_t blkstart;
  off_t blkend;
  ssize_t ret;

  DEBUGASSERT(fs != NULL && fs->mtd != NULL && dest != NULL);

#warning REVISIT:  What are units of offset and len?

  ret = MTD_READ(fs->mtd, offset, len, dest);
  if (ret < 0)
    {
      blksize  = fs->geo.blocksize;
      DEBUGASSERT(offset = offset % blksize);
      DEBUGASSERT(len    = len    % blksize);

      blkstart = offset / blksize;                       /* Truncates to floor */
      blkend   = (offset + len + blksize - 1) / blksize; /* Rounds up to ceil */

      ret      = MTD_BREAD(fs->mtd, blkstart, blkend - blkstart, dest);
    }

  return ret;
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
