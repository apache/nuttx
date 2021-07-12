/****************************************************************************
 * fs/spiffs/src/spiffs_cache.h
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

#ifndef __FS_SPIFFS_SRC_SPIFFS_CACHE_H
#define __FS_SPIFFS_SRC_SPIFFS_CACHE_H

#if defined(__cplusplus)
extern "C"
{
#endif

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SPIFFS_CACHE_FLAG_DIRTY       (1 << 0)
#define SPIFFS_CACHE_FLAG_WRTHRU      (1 << 1)
#define SPIFFS_CACHE_FLAG_OBJLU       (1 << 2)
#define SPIFFS_CACHE_FLAG_OBJNDX      (1 << 3)
#define SPIFFS_CACHE_FLAG_DATA        (1 << 4)
#define SPIFFS_CACHE_FLAG_TYPE_WR     (1 << 7)

#define SPIFFS_CACHE_PAGE_SIZE(fs) \
  (sizeof(struct spiffs_cache_page_s) + SPIFFS_GEO_PAGE_SIZE(fs))

#define spiffs_get_cache(fs) \
  ((FAR struct spiffs_cache_s *)((fs)->cache))

#define spiffs_get_cache_page_hdr(fs, c, cpndx) \
  ((FAR struct spiffs_cache_page_s *)(&((c)->cpages[(cpndx) * \
  SPIFFS_CACHE_PAGE_SIZE(fs)])))

#define spiffs_get_cache_page(fs, c, cpndx) \
  ((FAR uint8_t *)(&((c)->cpages[(cpndx) * SPIFFS_CACHE_PAGE_SIZE(fs)])) + \
  sizeof(struct spiffs_cache_page_s))

/* Debug */

#ifdef CONFIG_SPIFFS_CACHEDBG
#  define spiffs_cacheinfo                _info
#else
#  define spiffs_cacheinfo                _none
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* cache page struct */

struct spiffs_cache_page_s
{
  uint8_t flags;             /* Cache flags */
  uint8_t cpndx;             /* Cache page index */
  uint32_t last_access;      /* Last access of this cache page */
  union
    {
      struct                 /* Type read cache */
        {
          int16_t pgndx;     /* Read cache page index */
        };

      struct                 /* Type write cache */
        {
          int16_t objid;     /* Write cache */
          uint32_t offset;   /* Offset in cache page */
          uint16_t size;     /* Size of cache page */
        };
    };
};

/* Cache structure */

struct spiffs_cache_s
{
  uint8_t cpage_count;
  uint32_t last_access;
  uint32_t cpage_use_map;
  uint32_t cpage_use_mask;
  FAR uint8_t *cpages;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

struct spiffs_s;  /* Forward reference */

/****************************************************************************
 * Name: spiffs_cache_initialize
 *
 * Description:
 *   Initializes the cache
 *
 * Input Parameters:
 *   fs  - A reference to the SPIFFS volume object instance
 *   cp  - The cache page to be released
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spiffs_cache_initialize(FAR struct spiffs_s *fs);

/****************************************************************************
 * Name: spiffs_cache_drop_page
 *
 * Description:
 *   Drops the cache page for give page index
 *
 * Input Parameters:
 *   fs    - A reference to the SPIFFS volume object instance
 *   pgndx - Page index to be dropped
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spiffs_cache_drop_page(FAR struct spiffs_s *fs, int16_t pgndx);

/****************************************************************************
 * Name: spiffs_cache_read
 *
 * Description:
 *   Reads from SPI FLASH or the cache
 *
 * Input Parameters:
 *   fs    - A reference to the SPIFFS volume object instance
 *   op    - Read options
 *   objid - File object ID to read
 *   addr  - Address to read from
 *   len   - The number of bytes to be read
 *   dest  - The location in which the read data is to be returned.
 *
 * Returned Value:
 *   The number of bytes read (len) is returned on success; A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

ssize_t spiffs_cache_read(FAR struct spiffs_s *fs, uint8_t op, int16_t objid,
                          off_t addr, size_t len, FAR uint8_t *dest);

/****************************************************************************
 * Name: spiffs_cache_write
 *
 * Description:
 *   Writes to SPI FLASH and/or the cache
 *
 * Input Parameters:
 *   fs    - A reference to the SPIFFS volume object instance
 *   op    - Write options
 *   objid - File object ID to write
 *   addr  - Address to write to
 *   len   - The number of bytes to be write
 *   dest  - The location that provide the data to be written.
 *
 * Returned Value:
 *   The number of bytes written (len) is returned on success; A negated
 *   errno value is returned on any failure.
 *
 ****************************************************************************/

size_t spiffs_cache_write(FAR struct spiffs_s *fs, uint8_t op, int16_t objid,
                          off_t addr, size_t len, FAR uint8_t *src);

/****************************************************************************
 * Name: spiffs_cache_write
 *
 * Description:
 *   Returns the cache page that this file object refers to (or NULL if
 *   there is no cache page
 *
 * Input Parameters:
 *   fs    - A reference to the SPIFFS volume object instance
 *   fobj  - The file object instance
 *
 * Returned Value:
 *   Returns the cache page that this file object refers to (or NULL if
 *   there is no cache page
 *
 ****************************************************************************/

FAR struct spiffs_cache_page_s *
  spiffs_cache_page_get_byobjid(FAR struct spiffs_s *fs,
                                FAR struct spiffs_file_s *fobj);

/****************************************************************************
 * Name: spiffs_cache_page_allocate_byobjid
 *
 * Description:
 *   Allocates a new cache page and refers this to given object ID.  It
 *   flushes an old cache page if all cache pages are busy
 *
 * Input Parameters:
 *   fs    - A reference to the SPIFFS volume object instance
 *   fobj  - The file object instance
 *
 * Returned Value:
 *   Returns the allocated cache page(or NULL if the cache page could not
 *   be allocated).
 *
 ****************************************************************************/

FAR struct spiffs_cache_page_s *
  spiffs_cache_page_allocate_byobjid(FAR struct spiffs_s *fs,
                                     FAR struct spiffs_file_s *fobj);

/****************************************************************************
 * Name: spiffs_cache_page_release
 *
 * Description:
 *   "Unrefers" all file objects that this cache page refers to and releases
 *   the cache page
 *
 * Input Parameters:
 *   fs  - A reference to the SPIFFS volume object instance
 *   cp  - The cache page to be released
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void spiffs_cache_page_release(FAR struct spiffs_s *fs,
                               FAR struct spiffs_cache_page_s *cp);

#if defined(__cplusplus)
}
#endif

#endif /* __FS_SPIFFS_SRC_SPIFFS_CACHE_H */
