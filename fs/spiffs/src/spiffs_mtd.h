/****************************************************************************
 * fs/spiffs/src/spiffs_mtd.h
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

#ifndef __FS_SPIFFS_SRC_SPIFFS_MTD_H
#define __FS_SPIFFS_SRC_SPIFFS_MTD_H

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The MTD interface does not deal with pages explicitly.  With most
 * hardware a logical block is the same as an SPIFFS page.
 */

#define SPIFFS_GEO_MEDIA_SIZE(fs)       ((fs)->media_size)
#define SPIFFS_GEO_EBLOCK_COUNT(fs)     ((fs)->geo.neraseblocks)
#define SPIFFS_GEO_EBLOCK_SIZE(fs)      ((fs)->geo.erasesize)
#define SPIFFS_GEO_BLOCK_COUNT(fs)      ((fs)->geo.neraseblocks)
#define SPIFFS_GEO_BLOCK_SIZE(fs)       ((fs)->geo.erasesize)
#define SPIFFS_GEO_PAGE_COUNT(fs)       ((fs)->total_pages)
#define SPIFFS_GEO_PAGE_SIZE(fs)        ((fs)->geo.blocksize)
#define SPIFFS_GEO_PAGES_PER_BLOCK(fs)  ((fs)->pages_per_block)

/* Debug */

#ifdef CONFIG_SPIFFS_MTDDBG
#  define spiffs_mtdinfo                _info
#else
#  define spiffs_mtdinfo                _none
#endif

/* Commonly used Macros */

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

/****************************************************************************
 * Public Function Prototypes
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
                         FAR const uint8_t *src);

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
                        FAR uint8_t *dest);

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

ssize_t spiffs_mtd_erase(FAR struct spiffs_s *fs, off_t offset, size_t len);

#if defined(__cplusplus)
}
#endif

#endif /* __FS_SPIFFS_SRC_SPIFFS_MTD_H */
