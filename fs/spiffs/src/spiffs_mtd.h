/****************************************************************************
 * fs/spiffs/src/spiffs_mtd.h
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

#include <sys/param.h>

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
