/****************************************************************************
 * fs/spiffs/src/spiffs_cache.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of version 0.3.7 of SPIFFS by Peter Andersion.  That
 * version was originally released under the MIT license but is here re-
 * released under the NuttX BSD license.
 *
 *   Copyright (c) 2013-2017 Peter Andersson (pelleplutt1976@gmail.com)
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
#include <debug.h>

#include <nuttx/mtd/mtd.h>

#include "spiffs.h"
#include "spiffs_mtd.h"
#include "spiffs_core.h"
#include "spiffs_cache.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_cache_page_get
 *
 * Description:
 *  Returns cached page for give page index, or null if no such cached page
 *
 * Input Parameters:
 *   fs    - A reference to the SPIFFS volume object instance
 *   pgndx - Page index to get
 *
 * Returned Value:
 *   Reference to struct spiffs_cache_page_s instance.
 *
 ****************************************************************************/

FAR static FAR struct spiffs_cache_page_s *
  spiffs_cache_page_get(FAR struct spiffs_s *fs, int16_t pgndx)
{
  FAR struct spiffs_cache_s *cache;
  FAR struct spiffs_cache_page_s *cp;
  int i;

  cache = spiffs_get_cache(fs);
  if ((cache->cpage_use_map & cache->cpage_use_mask) == 0)
    {
      return 0;
    }

  for (i = 0; i < cache->cpage_count; i++)
    {
      cp = spiffs_get_cache_page_hdr(fs, cache, i);

      if ((cache->cpage_use_map & (1 << i)) != 0 &&
          (cp->flags & SPIFFS_CACHE_FLAG_TYPE_WR) == 0 &&
           cp->pgndx == pgndx)
        {
          cp->last_access = cache->last_access;
          return cp;
        }
    }

  return NULL;
}

/****************************************************************************
 * Name: spiffs_cache_page_free
 *
 * Description:
 *  Frees cached page
 *
 * Input Parameters:
 *   fs         - A reference to the SPIFFS volume object instance
 *   cpndx      - Cache page index
 *   write_back - True:  Write dirty pages back to hardware
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int spiffs_cache_page_free(FAR struct spiffs_s *fs, int cpndx,
                                  bool write_back)
{
  FAR struct spiffs_cache_s *cache;
  FAR struct spiffs_cache_page_s *cp;
  int ret = OK;

  cache = spiffs_get_cache(fs);
  cp    = spiffs_get_cache_page_hdr(fs, cache, cpndx);

  if (cache->cpage_use_map & (1 << cpndx))
    {
      if (write_back &&
          (cp->flags & SPIFFS_CACHE_FLAG_TYPE_WR) == 0 &&
          (cp->flags & SPIFFS_CACHE_FLAG_DIRTY) != 0)
        {
          FAR uint8_t *mem;

          mem = spiffs_get_cache_page(fs, cache, cpndx);

          spiffs_cacheinfo("Write cache page=%d pgndx %04x\n",
                           cpndx, cp->pgndx);

          ret = spiffs_mtd_write(fs, SPIFFS_PAGE_TO_PADDR(fs, cp->pgndx),
                                 SPIFFS_GEO_PAGE_SIZE(fs), mem);
        }

      if (cp->flags & SPIFFS_CACHE_FLAG_TYPE_WR)
        {
          spiffs_cacheinfo("Free cache page=%d objid=%04x\n",
                           cpndx, cp->objid);
        }
      else
        {
          spiffs_cacheinfo("Free cache page %d pgndx %04x\n",
                           cpndx, cp->pgndx);
        }

      cache->cpage_use_map &= ~(1 << cpndx);
      cp->flags = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_cache_page_remove_oldest
 *
 * Description:
 *   Removes the oldest accessed cached page
 *
 * Input Parameters:
 *   fs         - A reference to the SPIFFS volume object instance
 *   cpndx      - Cache page index
 *   write_back - True:  Write dirty pages back to hardware
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int spiffs_cache_page_remove_oldest(FAR struct spiffs_s *fs,
                                           uint8_t flag_mask, uint8_t flags)
{
  FAR struct spiffs_cache_s *cache = spiffs_get_cache(fs);
  uint32_t oldest_val = 0;
  int cpndx = -1;
  int ret = OK;
  int i;

  /* Don't remove any cache pages unless there are no free cache pages */

  if ((cache->cpage_use_map & cache->cpage_use_mask) !=
       cache->cpage_use_mask)
    {
      /* At least one free cpage */

      return OK;
    }

  /* All busy, scan through all to find the cache page which has oldest
   * access time.
   */

  for (i = 0; i < cache->cpage_count; i++)
    {
      FAR struct spiffs_cache_page_s *cp;

      cp = spiffs_get_cache_page_hdr(fs, cache, i);
      if ((cache->last_access - cp->last_access) > oldest_val &&
          (cp->flags & flag_mask) == flags)
        {
          oldest_val = cache->last_access - cp->last_access;
          cpndx = i;
        }
    }

  /* Then free that cache page */

  if (cpndx >= 0)
    {
      ret = spiffs_cache_page_free(fs, cpndx, true);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_cache_page_allocate
 *
 * Description:
 *   Allocates a new cached page and returns it, or null if all cache pages
 *   are busy.
 *
 * Input Parameters:
 *   fs         - A reference to the SPIFFS volume object instance
 *   cpndx      - Cache page index
 *   write_back - True:  Write dirty pages back to hardware
 *
 * Returned Value:
 *   A reference to the allocated cache page.  NULL is returned if we were
 *   unable to allocated a cache page.
 *
 ****************************************************************************/

static FAR struct spiffs_cache_page_s *
  spiffs_cache_page_allocate(FAR struct spiffs_s *fs)
{
  FAR struct spiffs_cache_s *cache;
  int i;

  /* Check if any cache pages are available */

  cache = spiffs_get_cache(fs);
  if (cache->cpage_use_map == 0xffffffff)
    {
      /* No.. Out of cache memory */

      return NULL;
    }

  /* Search for a free cache page */

  for (i = 0; i < cache->cpage_count; i++)
    {
      if ((cache->cpage_use_map & (1 << i)) == 0)
        {
          FAR struct spiffs_cache_page_s *cp;

          /* We found one */

          cp                    = spiffs_get_cache_page_hdr(fs, cache, i);
          cache->cpage_use_map |= (1 << i);
          cp->last_access       = cache->last_access;
          return cp;
        }
    }

  /* Out of cache entries */

  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

void spiffs_cache_initialize(FAR struct spiffs_s *fs)
{
  FAR struct spiffs_cache_s *cp;
  struct spiffs_cache_s cache;
  uint32_t cache_mask = 0;
  uint32_t sz;
  int i;
  int cache_entries;

  sz            = fs->cache_size;
  cache_entries = (sz - sizeof(struct spiffs_cache_s)) /
                  (SPIFFS_CACHE_PAGE_SIZE(fs));

  if (fs->cache == 0)
    {
      return;
    }

  if (cache_entries <= 0)
    {
      return;
    }

  for (i = 0; i < cache_entries; i++)
    {
      cache_mask <<= 1;
      cache_mask |= 1;
    }

  memset(&cache, 0, sizeof(struct spiffs_cache_s));
  cache.cpage_count    = cache_entries;
  cache.cpages         = (FAR uint8_t *)
    ((FAR uint8_t *)fs->cache + sizeof(struct spiffs_cache_s));

  cache.cpage_use_map  = 0xffffffff;
  cache.cpage_use_mask = cache_mask;
  memcpy(fs->cache, &cache, sizeof(struct spiffs_cache_s));

  cp = spiffs_get_cache(fs);
  memset(cp->cpages, 0, cp->cpage_count * SPIFFS_CACHE_PAGE_SIZE(fs));

  cp->cpage_use_map &= ~(cp->cpage_use_mask);
  for (i = 0; i < cache.cpage_count; i++)
    {
      spiffs_get_cache_page_hdr(fs, cp, i)->cpndx = i;
    }
}

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

void spiffs_cache_drop_page(FAR struct spiffs_s *fs, int16_t pgndx)
{
  FAR struct spiffs_cache_page_s *cp = spiffs_cache_page_get(fs, pgndx);
  if (cp)
    {
      spiffs_cache_page_free(fs, cp->cpndx, false);
    }
}

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
                         off_t addr, size_t len, FAR uint8_t *dest)
{
  FAR struct spiffs_cache_s *cache;
  FAR struct spiffs_cache_page_s *cp;
  int ret = OK;

  spiffs_cacheinfo("op=%02x, objid=%04x addr=%ld len=%lu\n",
                   op, objid, (long)addr, (unsigned long)len);

  cache = spiffs_get_cache(fs);
  cp    = spiffs_cache_page_get(fs, SPIFFS_PADDR_TO_PAGE(fs, addr));

  cache->last_access++;
  if (cp != NULL)
    {
      FAR uint8_t *mem;

      /* We've already got a cache page */

#ifdef CONFIG_SPIFFS_CACHEDBG
      fs->cache_hits++;
#endif

      cp->last_access = cache->last_access;
      mem             = spiffs_get_cache_page(fs, cache, cp->cpndx);
      memcpy(dest, &mem[SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr)], len);
    }
  else
    {
      /* Check for second layer lookup */

      if ((op & SPIFFS_OP_TYPE_MASK) == SPIFFS_OP_T_OBJ_LU2)
        {
          /* For second layer lookup functions, we do not cache in order to
           * prevent shredding
           */

          ret = spiffs_mtd_read(fs, addr, len, dest);
        }
      else
        {
#ifdef CONFIG_SPIFFS_CACHEDBG
          fs->cache_misses++;
#endif

          /* This operation will always free one cache page (unless all
           * already free), the result code stems from the write operation
           * of the possibly freed cache page
           */

          ret = spiffs_cache_page_remove_oldest(fs,
                                                SPIFFS_CACHE_FLAG_TYPE_WR,
                                                0);

          /* Allocate a new cache page */

          cp = spiffs_cache_page_allocate(fs);
          if (cp != NULL)
            {
              FAR uint8_t *mem;

              cp->flags = SPIFFS_CACHE_FLAG_WRTHRU;
              cp->pgndx = SPIFFS_PADDR_TO_PAGE(fs, addr);

              spiffs_cacheinfo("Allocated cache page %d for pgndx %04x\n",
                               cp->cpndx, cp->pgndx);

              ret = spiffs_mtd_read(fs, addr -
                                SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr),
                                SPIFFS_GEO_PAGE_SIZE(fs),
                                spiffs_get_cache_page(fs, cache, cp->cpndx));

              mem = spiffs_get_cache_page(fs, cache, cp->cpndx);
              memcpy(dest, &mem[SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr)], len);
            }
          else
            {
              /* This will never happen, last resort for sake of symmetry */

             ret = spiffs_mtd_read(fs, addr, len, dest);
            }
        }
    }

  if (ret < 0)
    {
      ferr("ERROR: spiffs_mtd_read: failed: %d\n", ret);
      return (ssize_t)ret;
    }

  return (ssize_t)len;
}

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
                          off_t addr, size_t len, FAR uint8_t *src)
{
  FAR struct spiffs_cache_s *cache;
  FAR struct spiffs_cache_page_s *cp;
  int16_t pgndx;
  int ret = OK;

  spiffs_cacheinfo("op=%02x, objid=%04x addr=%ld len=%lu\n",
                   op, objid, (long)addr, (unsigned long)len);

  pgndx = SPIFFS_PADDR_TO_PAGE(fs, addr);
  cache = spiffs_get_cache(fs);
  cp    = spiffs_cache_page_get(fs, pgndx);

  /* Check for write-through */

  if (cp != NULL && (op & SPIFFS_OP_COM_MASK) != SPIFFS_OP_C_WRTHRU)
    {
      /* Have a cache page.  Copy in data to cache page */

      if ((op & SPIFFS_OP_COM_MASK) == SPIFFS_OP_C_DELE &&
          (op & SPIFFS_OP_TYPE_MASK) != SPIFFS_OP_T_OBJ_LU)
        {
          /* Page is being deleted, wipe from cache - unless it is a lookup
           * page
           */

          spiffs_cache_page_free(fs, cp->cpndx, false);
          ret = spiffs_mtd_write(fs, addr, len, src);
        }
      else
        {
          uint8_t *mem = spiffs_get_cache_page(fs, cache, cp->cpndx);
          memcpy(&mem[SPIFFS_PADDR_TO_PAGE_OFFSET(fs, addr)], src, len);

          cache->last_access++;
          cp->last_access = cache->last_access;

          if (cp->flags & SPIFFS_CACHE_FLAG_WRTHRU)
            {
              /* Page is being updated, no write-cache, just pass through */

              ret = spiffs_mtd_write(fs, addr, len, src);
            }
          else
            {
              ret = OK;
            }
        }
    }
  else
    {
      /* No cache page, no write cache - just write through */

      ret = spiffs_mtd_write(fs, addr, len, src);
    }

  if (ret < 0)
    {
      ferr("ERROR: spiffs_mtd_write: failed: %d\n", ret);
      return (ssize_t)ret;
    }

  return (ssize_t)len;
}

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
                                FAR struct spiffs_file_s *fobj)
{
  FAR struct spiffs_cache_s *cache = spiffs_get_cache(fs);
  int i;

  if ((cache->cpage_use_map & cache->cpage_use_mask) == 0)
    {
      /* All cache pages free, no cache page can be assigned to the ID */

      return NULL;
    }

  /* Look at each cache index */

  for (i = 0; i < cache->cpage_count; i++)
    {
      FAR struct spiffs_cache_page_s *cp;

      /* Is this page available?  Is is writable?  Do the object IDs match? */

      cp = spiffs_get_cache_page_hdr(fs, cache, i);
      if ((cache->cpage_use_map & (1 << i)) &&
          (cp->flags & SPIFFS_CACHE_FLAG_TYPE_WR) &&
           cp->objid == fobj->objid)
        {
          /* Yes... return the cache page reference */

          return cp;
        }
    }

  /* Not found */

  return NULL;
}

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
                                     FAR struct spiffs_file_s *fobj)
{
  FAR struct spiffs_cache_page_s *cp;

  /* before this function is called, it is ensured that there is no already
   * existing cache page with same object ID
   */

  spiffs_cache_page_remove_oldest(fs, SPIFFS_CACHE_FLAG_TYPE_WR, 0);

  cp = spiffs_cache_page_allocate(fs);
  if (cp == NULL)
    {
      /* could not get cache page */

      return NULL;
    }

  cp->flags        = SPIFFS_CACHE_FLAG_TYPE_WR;
  cp->objid         = fobj->objid;
  fobj->cache_page = cp;

  spiffs_cacheinfo("Allocated cache page %d for objid=%d\n",
                   cp->cpndx, fobj->objid);
  return cp;
}

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
                               FAR struct spiffs_cache_page_s *cp)
{
  FAR struct spiffs_file_s *fobj;

  /* Verify that a non-NULL cache page was provided */

  if (cp != NULL)
    {
      /* Visit each file object */

      for (fobj  = (FAR struct spiffs_file_s *)dq_peek(&fs->objq);
           fobj != NULL;
           fobj  =
                 (FAR struct spiffs_file_s *)dq_next((FAR dq_entry_t *)fobj))
        {
          /* "Unrefer" the cache page */

          if (fobj->cache_page == cp)
            {
              fobj->cache_page = NULL;
            }
        }

      /* Free the cache page */

      spiffs_cache_page_free(fs, cp->cpndx, false);
      cp->objid = 0;
    }
}
