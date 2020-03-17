/****************************************************************************
 * fs/spiffs.h/spiffs_core.c
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

#include <nuttx/kmalloc.h>

#include "spiffs.h"
#include "spiffs_mtd.h"
#include "spiffs_gc.h"
#include "spiffs_cache.h"
#include "spiffs_core.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spiffs_free_objid_s
{
  int16_t min_objid;
  int16_t max_objid;
  uint32_t compaction;
  FAR const uint8_t *conflicting_name;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_page_data_check
 *
 * Description:
 *
 ****************************************************************************/

static int spiffs_page_data_check(FAR struct spiffs_s *fs,
                                  FAR struct spiffs_file_s *fobj,
                                  int16_t pgndx, int16_t spndx)
{
#ifdef CONFIG_SPIFFS_PAGE_CHECK
  struct spiffs_page_header_s ph;
#endif
  int ret = OK;

  if (pgndx == (int16_t)-1)
    {
      /* Referring to page 0xffff...., bad object index */

      return SPIFFS_ERR_INDEX_REF_FREE;
    }

  if (pgndx % SPIFFS_GEO_PAGES_PER_BLOCK(fs) < SPIFFS_OBJ_LOOKUP_PAGES(fs))
    {
      /* Referring to an object lookup page, bad object index */

      return SPIFFS_ERR_INDEX_REF_LU;
    }

  if (pgndx > SPIFFS_GEO_PAGE_COUNT(fs))
    {
      /* Referring to a bad page */

      return SPIFFS_ERR_INDEX_REF_INVALID;
    }

#ifdef CONFIG_SPIFFS_PAGE_CHECK
  ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_READ,
                          fobj->objid, SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                          sizeof(struct spiffs_page_header_s),
                          (FAR uint8_t *)&ph);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
    }
  else if ((ph.flags & SPIFFS_PH_FLAG_USED) != 0)
    {
      ret = SPIFFS_ERR_IS_FREE;
    }
  else if ((ph.flags & SPIFFS_PH_FLAG_DELET) == 0)
    {
      ret = SPIFFS_ERR_DELETED;
    }
  else if ((ph.flags & SPIFFS_PH_FLAG_FINAL) != 0)
    {
      ret = SPIFFS_ERR_NOT_FINALIZED;
    }
  else if ((ph.flags & SPIFFS_PH_FLAG_INDEX) == 0)
    {
      ret = SPIFFS_ERR_IS_INDEX;
    }
  else if (ph.spndx != (spndx))
    {
      ret = SPIFFS_ERR_DATA_SPAN_MISMATCH;
    }
#endif

  return ret;
}

/****************************************************************************
 * Name: spiffs_page_index_check
 *
 * Description:
 *
 ****************************************************************************/

static int spiffs_page_index_check(FAR struct spiffs_s *fs,
                                   FAR struct spiffs_file_s *fobj,
                                   int16_t pgndx, int16_t spndx)
{
#ifdef CONFIG_SPIFFS_PAGE_CHECK
  struct spiffs_page_header_s ph;
  int ret;
#endif

  if (pgndx == (int16_t) - 1)
    {
      /* Referring to page 0xffff...., bad object index */

      return SPIFFS_ERR_INDEX_FREE;
    }

  if (pgndx % SPIFFS_GEO_PAGES_PER_BLOCK(fs) < SPIFFS_OBJ_LOOKUP_PAGES(fs))
    {
      /* Referring to an object lookup page, bad object index */

      return SPIFFS_ERR_INDEX_LU;
    }

  if (pgndx > SPIFFS_GEO_PAGE_COUNT(fs))
    {
      /* Referring to a bad page */

      return SPIFFS_ERR_INDEX_INVALID;
    }

#ifdef CONFIG_SPIFFS_PAGE_CHECK
  ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_READ,
                   fobj->objid, SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                   sizeof(struct spiffs_page_header_s), (uint8_t *) & ph);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
      return ret;
    }

  ret = spiffs_validate_objndx(&ph, fobj->objid, spndx);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_validate_objndx() failed: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}

/****************************************************************************
 * Name: spiffs_objlu_scan_callback
 *
 * Description:
 *
 ****************************************************************************/

static int spiffs_objlu_scan_callback(FAR struct spiffs_s *fs, int16_t objid,
                                      int16_t blkndx, int entry,
                                      FAR const void *user_const,
                                      FAR void *user_var)
{
  if (objid == SPIFFS_OBJID_FREE)
    {
      if (entry == 0)
        {
          /* TODO:  Optimize further, return SPIFFS_NEXT_BLOCK */

          fs->free_blocks++;
        }
    }
  else if (objid == SPIFFS_OBJID_DELETED)
    {
      fs->deleted_pages++;
    }
  else
    {
      fs->alloc_pages++;
    }

  return SPIFFS_VIS_COUNTINUE;
}

/****************************************************************************
 * Name: spiffs_objlu_find_id_and_span_callback
 *
 * Description:
 *
 ****************************************************************************/

static int spiffs_objlu_find_id_and_span_callback(FAR struct spiffs_s *fs,
                                                  int16_t objid, int16_t blkndx,
                                                  int entry,
                                                  FAR const void *user_const,
                                                  FAR void *user_var)
{
  struct spiffs_page_header_s ph;
  int16_t pgndx;
  int ret;

  pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);

  ret = spiffs_cache_read(fs, 0, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                   SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                   sizeof(struct spiffs_page_header_s), (FAR uint8_t *)&ph);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
      return ret;
    }

  if (ph.objid == objid && ph.spndx == *((FAR int16_t *)user_var) &&
      (ph.flags & (SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_DELET |
                   SPIFFS_PH_FLAG_USED)) == SPIFFS_PH_FLAG_DELET &&
      !((objid & SPIFFS_OBJID_NDXFLAG) != 0 &&
        (ph.flags & SPIFFS_PH_FLAG_NDXDELE) == 0 && ph.spndx == 0) &&
      (user_const == NULL || *((FAR const int16_t *)user_const) != pgndx))
    {
      return OK;
    }
  else
    {
      return SPIFFS_VIS_COUNTINUE;
    }
}

/****************************************************************************
 * Name: spiffs_find_objhdr_pgndx_callback
 *
 * Description:
 *
 ****************************************************************************/

static int spiffs_find_objhdr_pgndx_callback(FAR struct spiffs_s *fs, int16_t objid,
                                             int16_t blkndx, int entry,
                                             FAR const void *user_const,
                                             FAR void *user_var)
{
  struct spiffs_pgobj_ndxheader_s objhdr;
  int16_t pgndx;
  int ret;

  if (objid == SPIFFS_OBJID_FREE || objid == SPIFFS_OBJID_DELETED ||
      (objid & SPIFFS_OBJID_NDXFLAG) == 0)
    {
      return SPIFFS_VIS_COUNTINUE;
    }

  pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);

  ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                          0, SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                          sizeof(struct spiffs_pgobj_ndxheader_s),
                          (FAR uint8_t *)&objhdr);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
      return ret;
    }

  if (objhdr.phdr.spndx == 0 &&
      (objhdr.phdr.flags & (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_FINAL |
                            SPIFFS_PH_FLAG_NDXDELE)) ==
      (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_NDXDELE))
    {
      if (strcmp((FAR const char *)user_const, (FAR char *)objhdr.name) == 0)
        {
          return OK;
        }
    }

  return SPIFFS_VIS_COUNTINUE;
}

/****************************************************************************
 * Name: spiffs_objlu_find_free_objid_bitmap_callback
 *
 * Description:
 *
 ****************************************************************************/

static int
  spiffs_objlu_find_free_objid_bitmap_callback(FAR struct spiffs_s *fs,
                                               int16_t objid, int16_t blkndx,
                                               int entry,
                                               FAR const void *user_const,
                                               FAR void *user_var)
{
  if (objid != SPIFFS_OBJID_FREE && objid != SPIFFS_OBJID_DELETED)
    {
      int16_t min_objid = *((FAR int16_t *)user_var);
      FAR const uint8_t *conflicting_name = (FAR const uint8_t *)user_const;
      uint32_t bitndx;
      int bytendx;

      /* If conflicting name parameter is given, also check if this name is
       * found in object index headers
       */

      if (conflicting_name != NULL && (objid & SPIFFS_OBJID_NDXFLAG) != 0)
        {
          struct spiffs_pgobj_ndxheader_s objhdr;
          int16_t pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);
          int ret;

          ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                  0, SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                                  sizeof(struct spiffs_pgobj_ndxheader_s),
                                  (FAR uint8_t *)&objhdr);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
              return ret;
            }

          if (objhdr.phdr.spndx == 0 &&
              (objhdr.phdr.flags & (SPIFFS_PH_FLAG_DELET |
                                    SPIFFS_PH_FLAG_FINAL |
                                    SPIFFS_PH_FLAG_NDXDELE)) ==
              (SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_NDXDELE))
            {
              if (strcmp((FAR const char *)user_const,
                         (FAR char *)objhdr.name) == 0)
                {
                  return -EEXIST;
                }
            }
        }

      objid   &= ~SPIFFS_OBJID_NDXFLAG;
      bitndx   = (objid - min_objid) & 7;
      bytendx  = (objid - min_objid) >> 3;

      if (bytendx >= 0 && (uint32_t)bytendx < SPIFFS_GEO_PAGE_SIZE(fs))
        {
          fs->work[bytendx] |= (1 << bitndx);
        }
    }

  return SPIFFS_VIS_COUNTINUE;
}

/****************************************************************************
 * Name: spiffs_objlu_find_free_objid_compact_callback
 *
 * Description:
 *
 ****************************************************************************/

static int
spiffs_objlu_find_free_objid_compact_callback(FAR struct spiffs_s *fs,
                                              int16_t objid, int16_t blkndx,
                                              int entry,
                                              FAR const void *user_const,
                                              FAR void *user_var)
{
  if (objid != SPIFFS_OBJID_FREE && objid != SPIFFS_OBJID_DELETED &&
      (objid & SPIFFS_OBJID_NDXFLAG) != 0)
    {
      FAR const struct spiffs_free_objid_s *state =
        (FAR const struct spiffs_free_objid_s *)user_const;
      struct spiffs_pgobj_ndxheader_s objhdr;
      int ret;

      ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ, 0,
                              SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, blkndx, entry),
                              sizeof(struct spiffs_pgobj_ndxheader_s),
                              (FAR uint8_t *)&objhdr);
      if (ret >= 0 && objhdr.phdr.spndx == 0 &&
          ((objhdr.phdr.flags & (SPIFFS_PH_FLAG_INDEX |
                                 SPIFFS_PH_FLAG_FINAL |
                                 SPIFFS_PH_FLAG_DELET)) ==
           (SPIFFS_PH_FLAG_DELET)))
        {
          /* OK object look up entry */

          if (state->conflicting_name &&
              strcmp((FAR const char *)state->conflicting_name,
                     (FAR char *)objhdr.name) == 0)
            {
              return -EEXIST;
            }

          objid &= ~SPIFFS_OBJID_NDXFLAG;
          if (objid >= state->min_objid && objid <= state->max_objid)
            {
              FAR uint8_t *map = (FAR uint8_t *)fs->work;
              int ndx = (objid - state->min_objid) / state->compaction;
              map[ndx]++;
            }
        }
    }

  return SPIFFS_VIS_COUNTINUE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_validate_objndx
 *
 * Description:
 *   Returns zero (OK) if everything checks out.  Otherwise, a negated errno
 *   value is return to indicate the nature of the problem
 *
 ****************************************************************************/

int spiffs_validate_objndx(FAR struct spiffs_page_header_s *ph,
                           int16_t objid, int16_t spndx)
{
  int ret = OK;

  if ((ph->flags & SPIFFS_PH_FLAG_USED) != 0)
    {
      ret = SPIFFS_ERR_IS_FREE;
    }
  else if ((ph->flags & SPIFFS_PH_FLAG_DELET) == 0)
    {
      ret = SPIFFS_ERR_DELETED;
    }
  else if ((ph->flags & SPIFFS_PH_FLAG_FINAL) != 0)
    {
      ret = SPIFFS_ERR_NOT_FINALIZED;
    }
  else if ((ph->flags & SPIFFS_PH_FLAG_INDEX) != 0)
    {
      ret = SPIFFS_ERR_NOT_INDEX;
    }
  else if ((objid & SPIFFS_OBJID_NDXFLAG) == 0)
    {
      ret = SPIFFS_ERR_NOT_INDEX;
    }
  else if (ph->spndx != spndx)
    {
      ret = SPIFFS_ERR_INDEX_SPAN_MISMATCH;
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_phys_cpy
 *
 * Description:
 *
 ****************************************************************************/

int spiffs_phys_cpy(FAR struct spiffs_s *fs,
                    int16_t objid, uint32_t dest, uint32_t src, uint32_t len)
{
  uint8_t cpybuf[CONFIG_SPIFFS_COPYBUF_STACK];
  int ret;

  while (len > 0)
    {
      uint32_t chunk_size = MIN(CONFIG_SPIFFS_COPYBUF_STACK, len);
      ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_MOVS,
                              objid, src, chunk_size, cpybuf);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
          return ret;
        }

      ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_MOVD,
                               objid, dest, chunk_size, cpybuf);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
          return ret;
        }

      len -= chunk_size;
      src += chunk_size;
      dest += chunk_size;
    }

  return OK;
}

/****************************************************************************
 * Name: spiffs_foreach_objlu
 *
 * Description:
 *   Find object lookup entry containing given objid with visitor.
 *   Iterate over object lookup pages in each block until a given object
 *   objid entry is found.
 *
 *   When found, the visitor function is called with block index, entry
 *   index and user data.
 *
 *   If visitor returns SPIFFS_VIS_CONTINUE, the search goes on. Otherwise,
 *   the search will be ended and visitor's return code is returned to
 *   caller.
 *
 *   If no visitor is given (0) the search returns on first entry with
 *   matching object objid.
 *
 *   If no match is found in all look up, SPIFFS_VIS_END is returned.
 *
 * Input Parameters:
 *   fs                - The file system
 *   starting_block    - The starting block to start search in
 *   starting_lu_entry - The look up index entry to start search in
 *   flags             - ORed combination of SPIFFS_VIS_CHECK_ID,
 *                       SPIFFS_VIS_CHECK_PH, SPIFFS_VIS_NO_WRAP
 *   objid             - Argument object ID
 *   cb                - Visitor callback function
 *   user_const        - Any const pointer, passed to the callback visitor
 *                       function
 *   user_var          - Any pointer, passed to the callback visitor function
 *   blkndx            - Reported block index where match was found
 *   lu_entry          - Reported look up index where match was found
 *
 ****************************************************************************/

int spiffs_foreach_objlu(FAR struct spiffs_s *fs, int16_t starting_block,
                         int starting_lu_entry, uint8_t flags, int16_t objid,
                         spiffs_callback_t cb, FAR const void *user_const,
                         FAR void *user_var, FAR int16_t *blkndx,
                         FAR int *lu_entry)
{
  FAR int16_t *objlu_buf;
  uint32_t cur_block_addr;
  int32_t entry_count;
  int16_t cur_block;
  int cur_entry;
  int entries_per_page;
  int ret;

  entry_count      = SPIFFS_GEO_BLOCK_COUNT(fs) * SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs);
  cur_block        = starting_block;
  cur_block_addr   = starting_block * SPIFFS_GEO_BLOCK_SIZE(fs);
  objlu_buf        = (FAR int16_t *)fs->lu_work;
  cur_entry        = starting_lu_entry;
  entries_per_page = (SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(int16_t));
  ret              = OK;

  /* Wrap initial */

  if (cur_entry > (int)SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs) - 1)
    {
      cur_entry = 0;
      cur_block++;
      cur_block_addr = cur_block * SPIFFS_GEO_BLOCK_SIZE(fs);

      if (cur_block >= SPIFFS_GEO_BLOCK_COUNT(fs))
        {
          if (flags & SPIFFS_VIS_NO_WRAP)
            {
              return SPIFFS_VIS_END;
            }
          else
            {
              /* Block wrap */

              cur_block      = 0;
              cur_block_addr = 0;
            }
        }
    }

  /* Check each block */

  while (ret >= 0 && entry_count > 0)
    {
      int obj_lookup_page = cur_entry / entries_per_page;

      /* Check each object lookup page */

      while (ret >= 0 &&
             obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
          int entry_offset;
          off_t physoff;

          entry_offset = obj_lookup_page * entries_per_page;
          physoff      = cur_block_addr +
                         SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page);

          ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                                  0, physoff, SPIFFS_GEO_PAGE_SIZE(fs),
                                  fs->lu_work);

          /* Check each entry */

          while (ret >= 0 &&
                 cur_entry - entry_offset < entries_per_page &&
                 cur_entry < (int)SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs))
            {
              if ((flags & SPIFFS_VIS_CHECK_ID) == 0 ||
                  objlu_buf[cur_entry - entry_offset] == objid)
                {
                  if (blkndx)
                    {
                      *blkndx = cur_block;
                    }

                  if (lu_entry)
                    {
                      *lu_entry = cur_entry;
                    }

                  if (cb)
                    {
                      ret = cb(fs,
                               (flags & SPIFFS_VIS_CHECK_PH) ? objid :
                               objlu_buf[cur_entry - entry_offset],
                               cur_block, cur_entry, user_const, user_var);

                      if (ret == SPIFFS_VIS_COUNTINUE ||
                          ret == SPIFFS_VIS_COUNTINUE_RELOAD)
                        {
                          if (ret == SPIFFS_VIS_COUNTINUE_RELOAD)
                            {
                              physoff = cur_block_addr +
                                        SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page);

                              ret =  spiffs_cache_read(fs,
                                                       SPIFFS_OP_T_OBJ_LU |
                                                       SPIFFS_OP_C_READ, 0,
                                                       physoff,
                                                       SPIFFS_GEO_PAGE_SIZE(fs),
                                                       fs->lu_work);
                              if (ret < 0)
                                {
                                  ferr("ERROR: spiffs_cache_read() failed: %d\n",
                                       ret);
                                  return ret;
                                }
                            }

                          ret = OK;
                          cur_entry++;
                          entry_count--;
                          continue;
                        }
                      else
                        {
                          return ret;
                        }
                    }
                  else
                    {
                      return OK;
                    }
                }

              entry_count--;
              cur_entry++;
            }

          obj_lookup_page++;
        }

      cur_entry = 0;
      cur_block++;
      cur_block_addr += SPIFFS_GEO_BLOCK_SIZE(fs);

      if (cur_block >= SPIFFS_GEO_BLOCK_COUNT(fs))
        {
          if (flags & SPIFFS_VIS_NO_WRAP)
            {
              return SPIFFS_VIS_END;
            }
          else
            {
              /* Block wrap */

              cur_block      = 0;
              cur_block_addr = 0;
            }
        }
    }

  if (ret < 0)
    {
      return ret;
    }

  return SPIFFS_VIS_END;
}

/****************************************************************************
 * Name: spiffs_erase_block
 *
 * Description:
 *
 ****************************************************************************/

int spiffs_erase_block(FAR struct spiffs_s *fs, int16_t blkndx)
{
  uint32_t addr = SPIFFS_BLOCK_TO_PADDR(fs, blkndx);
  int32_t size  = SPIFFS_GEO_BLOCK_SIZE(fs);
  int ret;

  /* Here we ignore the return value and just try erasing the block */

  while (size > 0)
    {
      finfo("erase %08lx:%d\n",
            (unsigned long)addr, SPIFFS_GEO_EBLOCK_SIZE(fs));

      spiffs_mtd_erase(fs, addr, SPIFFS_GEO_EBLOCK_SIZE(fs));

      addr += SPIFFS_GEO_EBLOCK_SIZE(fs);
      size -= SPIFFS_GEO_EBLOCK_SIZE(fs);
    }

  fs->free_blocks++;

  /* Register erase count for this block */

  ret = spiffs_cache_write(fs, SPIFFS_OP_C_WRTHRU | SPIFFS_OP_T_OBJ_LU2, 0,
                           SPIFFS_ERASE_COUNT_PADDR(fs, blkndx),
                           sizeof(int16_t),
                           (FAR uint8_t *)&fs->max_erase_count);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
      return ret;
    }

  fs->max_erase_count++;
  if (fs->max_erase_count == SPIFFS_OBJID_NDXFLAG)
    {
      fs->max_erase_count = 0;
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_objlu_scan
 *
 * Description:
 *   Scans through all obj lu and counts free, deleted and used pages.
 *   Find the maximum block erase count  Checks magic if enabled
 *
 ****************************************************************************/

int spiffs_objlu_scan(FAR struct spiffs_s *fs)
{
  int16_t blkndx;
  int16_t erase_count_final;
  int16_t erase_count_min;
  int16_t erase_count_max;
  int entry;
  int ret;

  /* Find out erase count. */

  blkndx          = 0;
  erase_count_min = SPIFFS_OBJID_FREE;
  erase_count_max = 0;

  while (blkndx < SPIFFS_GEO_BLOCK_COUNT(fs))
    {
      int16_t erase_count;

      ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                              0, SPIFFS_ERASE_COUNT_PADDR(fs, blkndx),
                              sizeof(int16_t), (FAR uint8_t *)&erase_count);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
          return ret;
        }

      if (erase_count != SPIFFS_OBJID_FREE)
        {
          erase_count_min = MIN(erase_count_min, erase_count);
          erase_count_max = MAX(erase_count_max, erase_count);
        }

      blkndx++;
    }

  if (erase_count_min == 0 && erase_count_max == SPIFFS_OBJID_FREE)
    {
      /* Clean system, set counter to zero */

      erase_count_final = 0;
    }
  else if (erase_count_max - erase_count_min > (SPIFFS_OBJID_FREE / 2))
    {
      /* Wrap, take min */

      erase_count_final = erase_count_min + 1;
    }
  else
    {
      erase_count_final = erase_count_max + 1;
    }

  fs->max_erase_count = erase_count_final;

  /* Count blocks */

  fs->free_blocks   = 0;
  fs->alloc_pages   = 0;
  fs->deleted_pages = 0;

  ret = spiffs_foreach_objlu(fs, 0, 0, 0, 0, spiffs_objlu_scan_callback,
                             0, 0, &blkndx, &entry);

  if (ret == SPIFFS_VIS_END)
    {
      ret = OK;
    }
  else if (ret < 0)
    {
      ferr("ERROR: spiffs_foreach_objlu() failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_objlu_find_free
 *
 * Description:
 *   Find free object lookup entry.  Iterate over object lookup pages in each
 *   block until a free object ID entry is found
 *
 ****************************************************************************/

int spiffs_objlu_find_free(FAR struct spiffs_s *fs, int16_t starting_block,
                           int starting_lu_entry, FAR int16_t *blkndx,
                           FAR int *lu_entry)
{
  int ret;

  if (fs->free_blocks < 2)
    {
      ret = spiffs_gc_quick(fs, 0);

      /* spiffs_gc_quick() will return -ENODATA if no blocks were freed */

      if (ret == -ENODATA)
        {
          ret = OK;
        }
      else if (ret < 0)
        {
          return ret;
        }

      if (fs->free_blocks < 2)
        {
          return -ENOSPC;
        }
    }

  ret = spiffs_objlu_find_id(fs, starting_block, starting_lu_entry,
                             SPIFFS_OBJID_FREE, blkndx, lu_entry);
  if (ret >= 0)
    {
      fs->free_blkndx = *blkndx;
      fs->free_entry  = (*lu_entry) + 1;

      if (*lu_entry == 0)
        {
          fs->free_blocks--;
        }
    }

  if (ret == -ENOSPC)
    {
      fwarn("WARNING: Filesystem full\n");
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_objlu_find_id
 *
 * Description:
 *   Find object lookup entry containing given objid.  Iterate over object
 *   lookup pages in each block until a given object objid entry is found
 *
 ****************************************************************************/

int spiffs_objlu_find_id(FAR struct spiffs_s *fs, int16_t starting_block,
                         int starting_lu_entry, int16_t objid,
                         FAR int16_t *blkndx, FAR int *lu_entry)
{
  int ret = spiffs_foreach_objlu(fs, starting_block, starting_lu_entry,
                                 SPIFFS_VIS_CHECK_ID, objid, 0, 0, 0,
                                 blkndx, lu_entry);
  if (ret == SPIFFS_VIS_END)
    {
      ret = -ENOENT;
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_objlu_find_id_and_span
 *
 * Description:
 *   Find object lookup entry containing given objid and span index.
 *   Iterate over object lookup pages in each block until a given object
 *   objid entry is found
 *
 ****************************************************************************/

int spiffs_objlu_find_id_and_span(FAR struct spiffs_s *fs, int16_t objid,
                                  int16_t spndx, int16_t exclusion_pgndx,
                                  FAR int16_t *pgndx)
{
  int16_t blkndx;
  int entry;
  int ret;

  ret = spiffs_foreach_objlu(fs, fs->lu_blkndx, fs->lu_entry,
                             SPIFFS_VIS_CHECK_ID, objid,
                             spiffs_objlu_find_id_and_span_callback,
                             exclusion_pgndx ? &exclusion_pgndx : 0,
                             &spndx, &blkndx, &entry);

  if (ret == SPIFFS_VIS_END)
    {
      ret = -ENOENT;
    }
  else if (ret < 0)
    {
      ferr("ERROR: spiffs_foreach_objlu() failed: %d\n", ret);
      return ret;
    }

  if (pgndx != NULL)
    {
      *pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);
    }

  fs->lu_blkndx = blkndx;
  fs->lu_entry  = entry;

  return ret;
}

/****************************************************************************
 * Name: spiffs_objlu_find_id_and_span_byphdr
 *
 * Description:
 *   Find object lookup entry containing given objid and span index in page
 *   headers only.  Iterate over object lookup pages in each block until a
 *   given object objid entry is found
 *
 ****************************************************************************/

int spiffs_objlu_find_id_and_span_byphdr(FAR struct spiffs_s *fs,
                                         int16_t objid, int16_t spndx,
                                         int16_t exclusion_pgndx,
                                         FAR int16_t *pgndx)
{
  int16_t blkndx;
  int entry;
  int ret;

  ret = spiffs_foreach_objlu(fs, fs->lu_blkndx, fs->lu_entry,
                             SPIFFS_VIS_CHECK_PH, objid,
                             spiffs_objlu_find_id_and_span_callback,
                             exclusion_pgndx ? &exclusion_pgndx : 0,
                             &spndx, &blkndx, &entry);

  if (ret == SPIFFS_VIS_END)
    {
      ret = -ENOENT;
    }

  if (ret < 0)
    {
      return ret;
    }

  if (pgndx != NULL)
    {
      *pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);
    }

  fs->lu_blkndx = blkndx;
  fs->lu_entry  = entry;
  return ret;
}

/****************************************************************************
 * Name: spiffs_page_allocate_data
 *
 * Description:
 *   Allocates a free defined page with given objid.   Occupies object
 *   lookup entry and page.  Data may be NULL; where only page header is
 *   stored, len and page_offs is ignored
 *
 ****************************************************************************/

int spiffs_page_allocate_data(FAR struct spiffs_s *fs, int16_t objid,
                              FAR struct spiffs_page_header_s *ph,
                              FAR uint8_t *data, uint32_t len,
                              uint32_t page_offs,
                              bool finalize, FAR int16_t *pgndx)
{
  int16_t blkndx;
  int entry;
  int ret = OK;

  /* Find free entry */

  ret = spiffs_objlu_find_free(fs, fs->free_blkndx, fs->free_entry,
                               &blkndx, &entry);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_objlu_find_free() failed: %d\n", ret);
      return ret;
    }

  /* Occupy page in object lookup */

  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT,
                           0, SPIFFS_BLOCK_TO_PADDR(fs, blkndx) +
                           entry * sizeof(int16_t), sizeof(int16_t),
                           (FAR uint8_t *)&objid);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
      return ret;
    }

  fs->alloc_pages++;

  /* Write page header */

  ph->flags &= ~SPIFFS_PH_FLAG_USED;
  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT, 0,
                           SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, blkndx, entry),
                           sizeof(struct spiffs_page_header_s),
                           (FAR uint8_t *)ph);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
      return ret;
    }

  /* Write page data */

  if (data)
    {
      ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT, 0,
                               SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, blkndx, entry) +
                               sizeof(struct spiffs_page_header_s) + page_offs,
                               len, data);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
          return ret;
        }
    }

  /* Finalize header if necessary */

  if (finalize && (ph->flags & SPIFFS_PH_FLAG_FINAL) != 0)
    {
      ph->flags &= ~SPIFFS_PH_FLAG_FINAL;
      ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT, 0,
                               SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, blkndx, entry) +
                               offsetof(struct spiffs_page_header_s, flags),
                               sizeof(uint8_t), (FAR uint8_t *)&ph->flags);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
          return ret;
        }
    }

  /* Return written page */

  if (pgndx != NULL)
    {
      *pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_page_move
 *
 * Description:
 *   Moves a page from src to a free page and finalizes it.  Updates page
 *   index.  Page data is given in param page.  If page data is NULL,
 *   provided header is used for meta-info and page data is physically
 *   copied.
 *
 ****************************************************************************/

int spiffs_page_move(FAR struct spiffs_s *fs,
                     int16_t objid, FAR uint8_t *page_data, int16_t ndx,
                     FAR struct spiffs_page_header_s *page_hdr,
                     int16_t src_pgndx, FAR int16_t *dst_pgndx)
{
  struct spiffs_page_header_s *phdr;
  bool was_final = false;
  int16_t free_pgndx;
  int16_t blkndx;
  int entry;
  int ret;

  /* Find free entry */

  ret = spiffs_objlu_find_free(fs, fs->free_blkndx, fs->free_entry,
                               &blkndx, &entry);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_gc_epage_stats() failed: %d\n", ret);
      return ret;
    }

  free_pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);

  if (dst_pgndx != NULL)
    {
      *dst_pgndx = free_pgndx;
    }

  phdr = page_data ? (struct spiffs_page_header_s *) page_data : page_hdr;
  if (page_data != NULL)
    {
      /* Got page data */

      was_final = (phdr->flags & SPIFFS_PH_FLAG_FINAL) == 0;

      /* Write unfinalized page */

      phdr->flags |= SPIFFS_PH_FLAG_FINAL;
      phdr->flags &= ~SPIFFS_PH_FLAG_USED;
      ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT, 0,
                               SPIFFS_PAGE_TO_PADDR(fs, free_pgndx),
                               SPIFFS_GEO_PAGE_SIZE(fs), page_data);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
          return ret;
        }
    }
  else
    {
      /* Copy page data */

      ret = spiffs_phys_cpy(fs, objid, SPIFFS_PAGE_TO_PADDR(fs, free_pgndx),
                            SPIFFS_PAGE_TO_PADDR(fs, src_pgndx),
                            SPIFFS_GEO_PAGE_SIZE(fs));
      if (ret < 0)
        {
          ferr("ERROR: spiffs_phys_cpy() failed: %d\n", ret);
          return ret;
        }
    }

  /* Mark entry in destination object lookup */

  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT, 0,
                           SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs, free_pgndx)) +
                           SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, free_pgndx) *
                           sizeof(int16_t), sizeof(int16_t),
                           (FAR uint8_t *)&ndx);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
      return ret;
    }

  fs->alloc_pages++;

  if (was_final)
    {
      /* Mark finalized in destination page */

      phdr->flags &= ~(SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_USED);
      ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT, objid,
                               SPIFFS_PAGE_TO_PADDR(fs, free_pgndx) +
                               offsetof(struct spiffs_page_header_s, flags),
                               sizeof(uint8_t), (FAR uint8_t *)&phdr->flags);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
          return ret;
        }
    }

  /* Mark source deleted */

  ret = spiffs_page_delete(fs, src_pgndx);
  return ret;
}

/****************************************************************************
 * Name: spiffs_page_delete
 *
 * Description:
 *   Deletes a page and removes it from object lookup.
 *
 ****************************************************************************/

int spiffs_page_delete(FAR struct spiffs_s *fs, int16_t pgndx)
{
  uint8_t flags;
  int ret;

  /* Mark deleted entry in source object lookup */

  int16_t d_objid = SPIFFS_OBJID_DELETED;
  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_DELE, 0,
                           SPIFFS_BLOCK_TO_PADDR(fs, SPIFFS_BLOCK_FOR_PAGE(fs, pgndx)) +
                           SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, pgndx) *
                           sizeof(int16_t), sizeof(int16_t),
                           (FAR uint8_t *)&d_objid);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
      return ret;
    }

  fs->deleted_pages++;
  fs->alloc_pages--;

  /* Mark deleted in source page */

  flags = 0xff;

#ifdef CONFIG_SPIFFS_NO_BLIND_WRITES
  /* Perform read-modify-write operation */

  physoff = SPIFFS_PAGE_TO_PADDR(fs, pgndx) +
            offsetof(struct spiffs_page_header_s, flags)
  ret     = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_READ, 0,
                              physoff, sizeof(flags), &flags);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
      return ret;
    }
#endif

  flags &= ~(SPIFFS_PH_FLAG_DELET | SPIFFS_PH_FLAG_USED);
  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_DELE, 0,
                           SPIFFS_PAGE_TO_PADDR(fs, pgndx) +
                           offsetof(struct spiffs_page_header_s, flags),
                           sizeof(flags), &flags);

  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_fobj_create
 *
 * Description:
 *   Create an object index header page with empty index and undefined length
 *
 ****************************************************************************/

int spiffs_fobj_create(FAR struct spiffs_s *fs,
                       int16_t objid, const uint8_t name[],
                       uint8_t type, FAR int16_t *objhdr_pgndx)
{
  struct spiffs_pgobj_ndxheader_s objndx_hdr;
  int16_t blkndx;
  int entry;
  int ret = OK;

  ret = spiffs_gc_check(fs, SPIFFS_DATA_PAGE_SIZE(fs));
  if (ret < 0)
    {
      ferr("ERROR: spiffs_gc_check() failed: %d\n", ret);
      return ret;
    }

  objid |= SPIFFS_OBJID_NDXFLAG;

  /* Find free entry */

  ret = spiffs_objlu_find_free(fs, fs->free_blkndx, fs->free_entry,
                               &blkndx, &entry);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_objlu_find_free() failed: %d\n", ret);
      return ret;
    }

  finfo("Found free page @ %04x blkndx=%04x entry=%04x\n",
        (int16_t) SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry),
        blkndx, entry);

  /* Occupy page in object lookup */

  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_UPDT, 0,
                           SPIFFS_BLOCK_TO_PADDR(fs, blkndx) +
                           entry * sizeof(int16_t), sizeof(int16_t),
                           (FAR uint8_t *)&objid);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
      return ret;
    }

  fs->alloc_pages++;

  /* Write empty object index page */

  objndx_hdr.phdr.objid = objid;
  objndx_hdr.phdr.spndx = 0;
  objndx_hdr.phdr.flags =
    0xff & ~(SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_INDEX | SPIFFS_PH_FLAG_USED);
  objndx_hdr.type       = type;
  objndx_hdr.size       = SPIFFS_UNDEFINED_LEN;

  strncpy((char *)objndx_hdr.name, (const char *)name, CONFIG_SPIFFS_NAME_MAX);

  /* Update page */

  ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT, 0,
                           SPIFFS_OBJ_LOOKUP_ENTRY_TO_PADDR(fs, blkndx, entry),
                           sizeof(struct spiffs_pgobj_ndxheader_s),
                           (FAR uint8_t *)&objndx_hdr);

  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
      return ret;
    }

  spiffs_fobj_event(fs, (FAR struct spiffs_page_objndx_s *)&objndx_hdr,
                    SPIFFS_EV_NDXNEW, objid, 0,
                    SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry),
                    SPIFFS_UNDEFINED_LEN);

  if (objhdr_pgndx)
    {
      *objhdr_pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);
    }

  return ret < 0 ? ret : OK;
}

/****************************************************************************
 * Name: spiffs_fobj_update_ndxhdr
 *
 * Description:
 *   Update object index header with any combination of name/size/index.
 *   new_objhdr_data may be NULL, if so the object index header page is
 *   loaded.  Name may be NULL, if so name is not changed. size may be NULL,
 *   if so size is not changed
 *
 ****************************************************************************/

int spiffs_fobj_update_ndxhdr(FAR struct spiffs_s *fs,
                              FAR struct spiffs_file_s *fobj,
                              int16_t objid, int16_t objhdr_pgndx,
                              FAR uint8_t *new_objhdr_data,
                              const uint8_t name[],
                              uint32_t size, FAR int16_t *new_pgndx)
{
  FAR struct spiffs_pgobj_ndxheader_s *objhdr;
  int16_t new_objhdr_pgndx;
  int ret = OK;

  objid |= SPIFFS_OBJID_NDXFLAG;

  if (new_objhdr_data)
    {
      /* object index header page already given to us, no need to load it */

      objhdr = (FAR struct spiffs_pgobj_ndxheader_s *) new_objhdr_data;
    }
  else
    {
      /* Read object index header page */

      ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_READ,
                              fobj->objid,
                              SPIFFS_PAGE_TO_PADDR(fs, objhdr_pgndx),
                              SPIFFS_GEO_PAGE_SIZE(fs), fs->work);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
          return ret;
        }

      objhdr = (FAR struct spiffs_pgobj_ndxheader_s *)fs->work;
    }

  ret = spiffs_validate_objndx(&objhdr->phdr, objid, 0);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_validate_objndx() failed: %d\n", ret);
      return ret;
    }

  /* Change name */

  if (name != NULL)
    {
      strncpy((FAR char *)objhdr->name, (FAR const char *)name,
              CONFIG_SPIFFS_NAME_MAX);
    }

  if (size != 0)
    {
      objhdr->size = size;
    }

  /* Move and update page */

  ret = spiffs_page_move(fs, fobj == NULL ? 0 : fobj->objid,
                         (FAR uint8_t *)objhdr, objid, 0, objhdr_pgndx,
                         &new_objhdr_pgndx);

  if (ret >= 0)
    {
      if (new_pgndx != NULL)
        {
          *new_pgndx = new_objhdr_pgndx;
        }

      /* Object index update */

      spiffs_fobj_event(fs, (FAR struct spiffs_page_objndx_s *)objhdr,
                        new_objhdr_data ? SPIFFS_EV_NDXUPD :
                        SPIFFS_EV_NDXUPD_HDR, objid,
                        objhdr->phdr.spndx, new_objhdr_pgndx,
                        objhdr->size);
      if (fobj != NULL)
        {
          fobj->objhdr_pgndx = new_objhdr_pgndx;  /* If this is not in the
                                                   * registered cluster */
        }
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_fobj_event
 *
 * Description:
 *
 ****************************************************************************/

void spiffs_fobj_event(FAR struct spiffs_s *fs,
                       FAR struct spiffs_page_objndx_s *objndx,
                       int ev, int16_t objid_raw, int16_t spndx,
                       int16_t new_pgndx, uint32_t new_size)
{
#ifdef CONFIG_DEBUG_FS_INFO
  FAR static const char *evname[] =
  {
    "UPD", "NEW", "DEL", "MOV", "HUP", "???"
  };
#endif

  FAR struct spiffs_file_s *fobj;
  FAR struct spiffs_file_s *next;
  int16_t objid = objid_raw & ~SPIFFS_OBJID_NDXFLAG;

  finfo("Event=%s objid=%04x spndx=%04x npgndx=%04x nsz=%d\n",
        evname[MIN(ev, 5)], objid_raw, spndx, new_pgndx, new_size);

  /* Update index caches in all file descriptors */

  for (fobj  = (FAR struct spiffs_file_s *)dq_peek(&fs->objq);
       fobj != NULL;
       fobj  = next)
    {
      /* Set up for the next time through the loop (in case fobj is deleted) */

      next = (FAR struct spiffs_file_s *)dq_next((FAR dq_entry_t *)fobj);

      /* Is this the object we are looking for? */

      if ((fobj->objid & ~SPIFFS_OBJID_NDXFLAG) != objid)
        {
          continue;  /* Object not related to updated file */
        }

      if (spndx == 0)
        {
          /* Object index header update */

          if (ev != SPIFFS_EV_NDXDEL)
            {
              finfo("Setting objid=%d (offset=%d) objhdr_pgndx to %04x size=%d\n",
                    fobj->objid, fobj->offset, new_pgndx, new_size);

              fobj->objhdr_pgndx = new_pgndx;
              if (new_size != 0)
                {
                  uint32_t act_new_size = 0;

                  /* Update size and offsets for fobj to this file */

                  fobj->size = new_size;

                  if (new_size != SPIFFS_UNDEFINED_LEN)
                    {
                      act_new_size = new_size;
                    }

                  if (act_new_size > 0 && fobj->cache_page)
                    {
                      act_new_size =  MAX(act_new_size,
                                          fobj->cache_page->offset +
                                          fobj->cache_page->size);
                    }

                  if (fobj->offset > act_new_size)
                    {
                      fobj->offset = act_new_size;
                    }

                  if (fobj->cache_page &&
                      fobj->cache_page->offset > act_new_size + 1)
                    {
                      spiffs_cacheinfo("File truncated, dropping cache page=%d, "
                                       "no writeback\n",
                                       fobj->cache_page->cpndx);

                      spiffs_cache_page_release(fs, fobj->cache_page);
                    }
                }
            }
          else
            {
              /* Removing file */

              if (fobj->cache_page)
                {
                  spiffs_cacheinfo("File deleted, dropping cache page=%d, "
                                   "no writeback\n",
                                   fobj->cache_page->cpndx);

                  spiffs_cache_page_release(fs, fobj->cache_page);
                }

              finfo("Release objid=%d span=%04x objndx_pgndx to %04x\n",
                    fobj->objid, spndx, new_pgndx);

              /* Remove the file object */

              spiffs_fobj_free(fs, fobj, true);
            }
        }

      if (fobj->objndx_spndx == spndx)
        {
          if (ev != SPIFFS_EV_NDXDEL)
            {
              finfo("Setting objid=%d span=%04x objndx_pgndx to %04x\n",
                     fobj->objid, spndx, new_pgndx);

              fobj->objndx_pgndx = new_pgndx;
            }
          else
            {
              fobj->objndx_pgndx = 0;
            }
        }
    }
}

/****************************************************************************
 * Name: spiffs_fobj_open_bypage
 *
 * Description:
 *   Open object by page index
 *
 ****************************************************************************/

int spiffs_fobj_open_bypage(FAR struct spiffs_s *fs, int16_t pgndx,
                            FAR struct spiffs_file_s *fobj)
{
  struct spiffs_pgobj_ndxheader_s objndx_hdr;
  off_t physoff;
  int16_t objid;
  int16_t blkndx;
  int entry;
  int ret = OK;

  physoff = SPIFFS_PAGE_TO_PADDR(fs, pgndx);
  ret     = spiffs_cache_read(fs, SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_READ,
                              fobj->objid, physoff,
                              sizeof(struct spiffs_pgobj_ndxheader_s),
                              (FAR uint8_t *)&objndx_hdr);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
      return ret;
    }

  blkndx  = SPIFFS_BLOCK_FOR_PAGE(fs, pgndx);
  entry   = SPIFFS_OBJ_LOOKUP_ENTRY_FOR_PAGE(fs, pgndx);

  physoff = SPIFFS_BLOCK_TO_PADDR(fs, blkndx) + entry * sizeof(int16_t);
  ret     = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ, 0,
                              physoff, sizeof(int16_t), (FAR uint8_t *)&objid);

  /* Fill in the parts of the open file structure known only to the core
   * logic.
   */

  fobj->objhdr_pgndx = pgndx;
  fobj->size         = objndx_hdr.size;
  fobj->offset       = 0;
  fobj->objndx_pgndx = pgndx;
  fobj->objndx_spndx = 0;
  fobj->objid        = objid;

  ret = spiffs_validate_objndx(&objndx_hdr.phdr, fobj->objid, 0);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_validate_objndx() failed: %d\n", ret);
      return ret;
    }

  finfo("objid=%d\n", fobj->objid);
  return ret;
}

/****************************************************************************
 * Name: spiffs_fobj_append
 *
 * Description:
 *   Append to object.  Deep current object index (header) page in fs->work
 *   buffer
 *
 ****************************************************************************/

ssize_t spiffs_fobj_append(FAR struct spiffs_s *fs,
                           FAR struct spiffs_file_s *fobj, off_t offset,
                           FAR uint8_t *data, size_t len)
{
  struct spiffs_page_header_s phdr;
  FAR struct spiffs_pgobj_ndxheader_s *objhdr;
  FAR struct spiffs_page_objndx_s *objndx;
  ssize_t nwritten = 0;
  uint32_t page_offs;
  int16_t cur_objndx_spndx;
  int16_t prev_objndx_spndx;
  int16_t cur_objndx_pgndx;
  int16_t new_objhdr_page;
  int16_t data_spndx;
  int16_t data_page;
  int ret = OK;
  int ret2;

  finfo("Append %d bytes @ offs=%d of size=%d\n", len, offset, fobj->size);

  if (offset > fobj->size)
    {
      finfo("Offset replaced with size\n");
      offset = fobj->size < 0 ? 0 : fobj->size;
    }

  /* Add an extra page of data worth for meta */

  ret = spiffs_gc_check(fs, len + SPIFFS_DATA_PAGE_SIZE(fs));
  if (ret < 0)
    {
      ferr("ERROR: spiffs_gc_epage_stats() failed: %d\n", ret);
      return ret;
    }

  objhdr            = (FAR struct spiffs_pgobj_ndxheader_s *)fs->work;
  objndx            = (FAR struct spiffs_page_objndx_s *)fs->work;
  cur_objndx_spndx  = 0;
  prev_objndx_spndx = (int16_t)-1;
  cur_objndx_pgndx  = fobj->objhdr_pgndx;
  data_spndx        = offset / SPIFFS_DATA_PAGE_SIZE(fs);
  page_offs         = offset % SPIFFS_DATA_PAGE_SIZE(fs);

  /* Write all data */

  while (ret >= 0 && nwritten < len)
    {
      size_t to_write;

      /* Calculate object index page span index */

      cur_objndx_spndx = SPIFFS_OBJNDX_ENTRY_SPNDX(fs, data_spndx);

      /* Handle storing and loading of object indices */

      if (cur_objndx_spndx != prev_objndx_spndx)
        {
          /* New object index page.  Within this clause we return directly
           * if something fails, object index mess-up
           */

          if (nwritten > 0)
            {
              /* Store previous object index page, unless first pass */

              finfo("objid=%04x store objndx %04x:%04x, nwritten=%d\n",
                    fobj->objid, cur_objndx_pgndx, prev_objndx_spndx,
                    nwritten);

              if (prev_objndx_spndx == 0)
                {
                  /* This is an update to object index header page */

                  objhdr->size = offset + nwritten;
                  if (offset == 0)
                    {
                      /* Was an empty object, update same page (size was
                       * 0xffffffff)
                       */

                      ret = spiffs_page_index_check(fs, fobj, cur_objndx_pgndx, 0);
                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_page_index_check() failed: %d\n",
                                ret);
                          return ret;
                        }

                      ret =
                        spiffs_cache_write(fs,
                                           SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_UPDT,
                                           fobj->objid,
                                           SPIFFS_PAGE_TO_PADDR(fs, cur_objndx_pgndx),
                                           SPIFFS_GEO_PAGE_SIZE(fs),
                                           fs->work);
                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_cache_write() failed: %d\n",
                               ret);
                          return ret;
                        }
                    }
                  else
                    {
                      /* Was a nonempty object, update to new page */

                      ret = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                                      fobj->objhdr_pgndx,
                                                      fs->work, 0,
                                                      offset + nwritten,
                                                      &new_objhdr_page);
                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
                               ret);
                          return ret;
                        }

                      finfo("objid=%04x store new objhdr, %04x:%04x, nwritten=%d\n",
                            fobj->objid, new_objhdr_page, 0, nwritten);
                    }
                }
              else
                {
                  /* this is an update to an object index page */

                  ret = spiffs_page_index_check(fs, fobj, cur_objndx_pgndx,
                                                prev_objndx_spndx);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_page_index_check() failed: %d\n", ret);
                      return ret;
                    }

                  ret = spiffs_cache_write(fs,
                                           SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_UPDT,
                                           fobj->objid,
                                           SPIFFS_PAGE_TO_PADDR(fs, cur_objndx_pgndx),
                                           SPIFFS_GEO_PAGE_SIZE(fs),
                                           fs->work);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
                      return ret;
                    }

                  spiffs_fobj_event(fs, (FAR struct spiffs_page_objndx_s *)fs->work,
                                    SPIFFS_EV_NDXUPD, fobj->objid,
                                    objndx->phdr.spndx, cur_objndx_pgndx,
                                    0);

                  /* Update length in object index header page */

                  ret = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                                  fobj->objhdr_pgndx, 0, 0,
                                                  offset + nwritten,
                                                  &new_objhdr_page);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
                           ret);
                      return ret;
                    }

                  finfo("objid=%04x store new size I %d in objhdr, "
                        "%04x:%04x, nwritten=%d\n",
                        fobj->objid, offset + nwritten, new_objhdr_page, 0,
                        nwritten);
                }

              fobj->size   = offset + nwritten;
              fobj->offset = offset + nwritten;
            }

          /* create or load new object index page */

          if (cur_objndx_spndx == 0)
            {
              /* load object index header page, must always exist */

              finfo("objid=%04x load objndxhdr page %04x:%04x\n",
                    fobj->objid, cur_objndx_pgndx, cur_objndx_spndx);

              ret = spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_READ,
                                      fobj->objid,
                                      SPIFFS_PAGE_TO_PADDR(fs, cur_objndx_pgndx),
                                      SPIFFS_GEO_PAGE_SIZE(fs), fs->work);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
                  return ret;
                }

              ret = spiffs_validate_objndx(&objhdr->phdr, fobj->objid,
                                           cur_objndx_spndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_validate_objndx() failed: %d\n", ret);
                  return ret;
                }
            }
          else
            {
              int16_t len_objndx_spndx;

              /* On subsequent passes, create a new object index page */

              len_objndx_spndx =
                SPIFFS_OBJNDX_ENTRY_SPNDX(fs, (fobj->size - 1) /
                                            SPIFFS_DATA_PAGE_SIZE(fs));

              if (nwritten > 0 || cur_objndx_spndx > len_objndx_spndx)
                {
                  phdr.objid = fobj->objid | SPIFFS_OBJID_NDXFLAG;
                  phdr.spndx = cur_objndx_spndx;
                  phdr.flags =
                    0xff & ~(SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_INDEX);

                  ret = spiffs_page_allocate_data(fs,
                                                  fobj->objid | SPIFFS_OBJID_NDXFLAG,
                                                  &phdr, 0, 0, 0, 1,
                                                  &cur_objndx_pgndx);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_page_allocate_data() failed: %d\n",
                           ret);
                      return ret;
                    }

                  /* Quick "load" of new object index page */

                  memset(fs->work, 0xff, SPIFFS_GEO_PAGE_SIZE(fs));
                  memcpy(fs->work, &phdr, sizeof(struct spiffs_page_header_s));

                  spiffs_fobj_event(fs,
                                    (FAR struct spiffs_page_objndx_s *)fs->work,
                                    SPIFFS_EV_NDXNEW, fobj->objid,
                                    cur_objndx_spndx, cur_objndx_pgndx, 0);

                  finfo("objid=%04x create objndx page, %04x:%04x, nwritten=%d\n",
                        fobj->objid, cur_objndx_pgndx, cur_objndx_spndx,
                        nwritten);
                }
              else
                {
                  int16_t pgndx;

                  /* On first pass, we load existing object index page */

                  finfo("objid=%04x find objndx spndx=%04x\n",
                        fobj->objid, cur_objndx_spndx);

                  if (fobj->objndx_spndx == cur_objndx_spndx)
                    {
                      pgndx = fobj->objndx_pgndx;
                    }
                  else
                    {
                      ret =
                        spiffs_objlu_find_id_and_span(fs,
                                                      fobj->objid | SPIFFS_OBJID_NDXFLAG,
                                                      cur_objndx_spndx, 0,
                                                      &pgndx);
                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n",
                               ret);
                          return ret;
                        }
                    }

                  finfo("objid=%04x found object index at page=%04x [fobj size=%d]\n",
                        fobj->objid, pgndx, fobj->size);

                  ret = spiffs_cache_read(fs,
                                          SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_READ,
                                          fobj->objid,
                                          SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                                          SPIFFS_GEO_PAGE_SIZE(fs),
                                          fs->work);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
                      return ret;
                    }

                  ret = spiffs_validate_objndx(&objhdr->phdr, fobj->objid,
                                               cur_objndx_spndx);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_validate_objndx() failed: %d\n",
                           ret);
                      return ret;
                    }

                  cur_objndx_pgndx = pgndx;
                }

              fobj->objndx_pgndx = cur_objndx_pgndx;
              fobj->objndx_spndx = cur_objndx_spndx;
              fobj->offset       = offset + nwritten;
              fobj->size         = offset + nwritten;
            }

          prev_objndx_spndx = cur_objndx_spndx;
        }

      /* Write data */

      to_write = MIN(len - nwritten, SPIFFS_DATA_PAGE_SIZE(fs) - page_offs);

      if (page_offs == 0)
        {
          /* ATt beginning of a page, allocate and write a new page of data */

          phdr.objid = fobj->objid & ~SPIFFS_OBJID_NDXFLAG;
          phdr.spndx = data_spndx;
          phdr.flags = 0xff & ~(SPIFFS_PH_FLAG_FINAL); /* Finalize immediately */

          ret = spiffs_page_allocate_data(fs,
                                          fobj->objid & ~SPIFFS_OBJID_NDXFLAG,
                                          &phdr, &data[nwritten], to_write,
                                          page_offs, 1, &data_page);

          finfo("objid=%04x store new data page, %04x:%04x offset=%d, "
                "len=%d nwritten=%d\n",
                fobj->objid, data_page, data_spndx, page_offs, to_write,
                nwritten);
        }
      else
        {
          /* Append to existing page, fill out free data in existing page */

          if (cur_objndx_spndx == 0)
            {
              /* Get data page from object index header page */

              data_page =
                ((FAR int16_t *)((FAR uint8_t *)objhdr +
                  sizeof(struct spiffs_pgobj_ndxheader_s)))[data_spndx];
            }
          else
            {
              /* Get data page from object index page */

              data_page =
                ((FAR int16_t *)((FAR uint8_t *)objndx +
                  sizeof(struct spiffs_page_objndx_s)))
                    [SPIFFS_OBJNDX_ENTRY(fs, data_spndx)];
            }

          ret = spiffs_page_data_check(fs, fobj, data_page, data_spndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_page_data_check() failed: %d\n", ret);
              return ret;
            }

          ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                                   fobj->objid,
                                   SPIFFS_PAGE_TO_PADDR(fs, data_page) +
                                   sizeof(struct spiffs_page_header_s) + page_offs,
                                   to_write, &data[nwritten]);

          finfo("objid=%04x store to existing data page, %04x:%04x offset=%d, "
                "len=%d, nwritten=%d\n",
                fobj->objid, data_page, data_spndx, page_offs, to_write,
                nwritten);
        }

      if (ret < 0)
        {
          break;
        }

      /* Update memory representation of object index page with new data page */

      if (cur_objndx_spndx == 0)
        {
          /* Update object index header page */

          ((FAR int16_t *)((FAR uint8_t *) objhdr +
            sizeof(struct spiffs_pgobj_ndxheader_s)))[data_spndx] = data_page;

          finfo("objid=%04x wrote page %04x to objhdr entry=%04x in mem\n",
                fobj->objid, data_page, data_spndx);

          objhdr->size = offset + nwritten;
        }
      else
        {
          /* Update object index page */

          ((FAR int16_t *)((FAR uint8_t *) objndx +
            sizeof(struct spiffs_page_objndx_s)))
              [SPIFFS_OBJNDX_ENTRY(fs, data_spndx)] = data_page;

          finfo("objid=%04x wrote page=%04x to objndx entry %04x in mem\n",
                fobj->objid, data_page,
                (int16_t)SPIFFS_OBJNDX_ENTRY(fs, data_spndx));
        }

      /* Update internals */

      page_offs = 0;
      data_spndx++;
      nwritten  += to_write;
    }

  fobj->size         = offset + nwritten;
  fobj->offset       = offset + nwritten;
  fobj->objndx_pgndx = cur_objndx_pgndx;
  fobj->objndx_spndx = cur_objndx_spndx;

  /* Finalize updated object indices */

  ret2 = OK;
  if (cur_objndx_spndx != 0)
    {
      /* Wrote beyond object index header page.  Write last modified object
       * index page, unless object header index page
       */

      finfo("objid=%04x store objndx page, %04x:%04x, nwritten=%d\n",
            fobj->objid, cur_objndx_pgndx, cur_objndx_spndx, nwritten);

      ret2 = spiffs_page_index_check(fs, fobj, cur_objndx_pgndx,
                                     cur_objndx_spndx);
      if (ret2 < 0)
        {
          ferr("ERROR: spiffs_page_index_check() failed: %d\n", ret2);
          return ret2;
        }

      ret2 = spiffs_cache_write(fs, SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_UPDT,
                                fobj->objid,
                                SPIFFS_PAGE_TO_PADDR(fs, cur_objndx_pgndx),
                                SPIFFS_GEO_PAGE_SIZE(fs), fs->work);
      if (ret2 < 0)
        {
          ferr("ERROR: spiffs_cache_write() failed: %d\n", ret2);
          return ret2;
        }

      spiffs_fobj_event(fs,
                        (FAR struct spiffs_page_objndx_s *)fs->work,
                        SPIFFS_EV_NDXUPD, fobj->objid,
                        objndx->phdr.spndx, cur_objndx_pgndx, 0);

      /* Update size in object header index page */

      ret2 = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                       fobj->objhdr_pgndx, 0, 0,
                                       offset + nwritten,
                                       &new_objhdr_page);

      finfo("objid=%04x store new size II %d in objhdr, %04x:%04x, "
            "nwritten=%d, ret=%d\n",
            fobj->objid, offset + nwritten, new_objhdr_page, 0, nwritten,
            ret2);

      if (ret2 < 0)
        {
          ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
               ret2);
          return ret2;
        }
    }
  else
    {
      /* Wrote within object index header page */

      if (offset == 0)
        {
          /* Wrote to empty object - simply update size and write whole page */

          objhdr->size = offset + nwritten;

          finfo("objid=%04x store fresh objhdr page, %04x:%04x, nwritten=%d\n",
                fobj->objid, cur_objndx_pgndx, cur_objndx_spndx, nwritten);

          ret2 = spiffs_page_index_check(fs, fobj, cur_objndx_pgndx,
                                         cur_objndx_spndx);
          if (ret2 < 0)
            {
              ferr("ERROR: spiffs_page_index_check() failed: %d\n", ret2);
              return ret2;
            }

          ret2 = spiffs_cache_write(fs,
                                    SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_UPDT,
                                    fobj->objid,
                                    SPIFFS_PAGE_TO_PADDR(fs, cur_objndx_pgndx),
                                    SPIFFS_GEO_PAGE_SIZE(fs), fs->work);
          if (ret2 < 0)
            {
              ferr("ERROR: spiffs_cache_write() failed: %d\n", ret2);
              return ret2;
            }

          spiffs_fobj_event(fs,
                            (FAR struct spiffs_page_objndx_s *)fs->work,
                            SPIFFS_EV_NDXUPD_HDR, fobj->objid,
                            objhdr->phdr.spndx, cur_objndx_pgndx,
                            objhdr->size);
        }
      else
        {
          /* Modifying object index header page, update size and make new
           * copy.
           */

          ret2 = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                           fobj->objhdr_pgndx, fs->work, 0,
                                           offset + nwritten,
                                           &new_objhdr_page);

          finfo("objid=%04x store modified objhdr page, %04x:%04x, "
                "nwritten=%d\n",
                fobj->objid, new_objhdr_page, 0, nwritten);

          if (ret2 < 0)
            {
              ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
                   ret2);
              return ret2;
            }
        }
    }

  return nwritten;
}

/****************************************************************************
 * Name: spiffs_fobj_modify
 *
 * Description:
 *   Modify object.  Keep current object index (header) page in fs->work
 *   buffer
 *
 ****************************************************************************/

ssize_t spiffs_fobj_modify(FAR struct spiffs_s *fs,
                           FAR struct spiffs_file_s *fobj, off_t offset,
                           FAR uint8_t *data, size_t len)
{
  struct spiffs_page_header_s phdr;
  FAR struct spiffs_pgobj_ndxheader_s *objhdr;
  FAR struct spiffs_page_objndx_s *objndx;
  size_t nwritten = 0;
  uint32_t page_offs;
  int16_t cur_objndx_spndx;
  int16_t prev_objndx_spndx;
  int16_t cur_objndx_pgndx;
  int16_t new_objhdr_pgndx;
  int16_t data_spndx;
  int16_t data_pgndx;
  int ret = OK;
  int ret2;

  ret = spiffs_gc_check(fs, len + SPIFFS_DATA_PAGE_SIZE(fs));
  if (ret < 0)
    {
      ferr("ERROR: spiffs_gc_check() failed: %d\n", ret);
      return ret;
    }

  objhdr            = (FAR struct spiffs_pgobj_ndxheader_s *)fs->work;
  objndx            = (FAR struct spiffs_page_objndx_s *)fs->work;

  cur_objndx_spndx  = 0;
  prev_objndx_spndx = (int16_t) - 1;
  cur_objndx_pgndx  = fobj->objhdr_pgndx;

  data_spndx        = offset / SPIFFS_DATA_PAGE_SIZE(fs);
  page_offs         = offset % SPIFFS_DATA_PAGE_SIZE(fs);

  /* Write all data */

  while (ret >= 0 && nwritten < len)
    {
      size_t to_write;
      int16_t orig_data_pgndx;

      /* Calculate object index page span index */

      cur_objndx_spndx = SPIFFS_OBJNDX_ENTRY_SPNDX(fs, data_spndx);

      /* Handle storing and loading of object indices */

      if (cur_objndx_spndx != prev_objndx_spndx)
        {
          /* New object index page.  Within this clause we return directly
           * if something fails, object index mess-up
           */

          if (nwritten > 0)
            {
              /* Store previous object index (header) page, unless first
               * pass.
               */

              if (prev_objndx_spndx == 0)
                {
                  /* Store previous object index header page */

                  ret = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                                  fobj->objhdr_pgndx,
                                                  fs->work, 0, 0,
                                                  &new_objhdr_pgndx);

                  finfo("Store modified objhdr page, %04x:%04x, nwritten=%d\n",
                        new_objhdr_pgndx, 0, nwritten);

                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
                           ret);
                      return ret;
                    }
                }
              else
                {
                  /* Store new version of previous object index page */

                  int16_t new_objndx_pgndx;

                  ret = spiffs_page_index_check(fs, fobj, cur_objndx_pgndx,
                                                prev_objndx_spndx);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_page_index_check() failed: %d\n", ret);
                      return ret;
                    }

                  ret = spiffs_page_move(fs, fobj->objid, (FAR uint8_t *)objndx,
                                         fobj->objid, 0, cur_objndx_pgndx,
                                         &new_objndx_pgndx);

                  finfo("Store previous modified objndx page, %04x:%04x, "
                        "nwritten=%d\n",
                        new_objndx_pgndx, objndx->phdr.spndx, nwritten);

                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_page_move() failed: %d\n", ret);
                      return ret;
                    }

                  spiffs_fobj_event(fs,
                                      (FAR struct spiffs_page_objndx_s *)objndx,
                                      SPIFFS_EV_NDXUPD, fobj->objid,
                                      objndx->phdr.spndx, new_objndx_pgndx,
                                      0);
                }
            }

          /* Load next object index page */

          if (cur_objndx_spndx == 0)
            {
              /* Load object index header page, must exist */

              finfo("Load objndxhdr page %04x:%04x\n",
                    cur_objndx_pgndx, cur_objndx_spndx);

              ret = spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_READ,
                                      fobj->objid,
                                      SPIFFS_PAGE_TO_PADDR(fs, cur_objndx_pgndx),
                                      SPIFFS_GEO_PAGE_SIZE(fs), fs->work);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
                  return ret;
                }

              ret = spiffs_validate_objndx(&objhdr->phdr, fobj->objid,
                                           cur_objndx_spndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_validate_objndx() failed: %d\n", ret);
                  return ret;
                }
            }
          else
            {
              int16_t pgndx;

              /* Load existing object index page on first pass */

              finfo("Find objndx spndx=%04x\n", cur_objndx_spndx);

              if (fobj->objndx_spndx == cur_objndx_spndx)
                {
                  pgndx = fobj->objndx_pgndx;
                }
              else
                {
                  ret =
                    spiffs_objlu_find_id_and_span(fs,
                                                  fobj->objid | SPIFFS_OBJID_NDXFLAG,
                                                  cur_objndx_spndx, 0,
                                                  &pgndx);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n",
                           ret);
                      return ret;
                    }
                }

              finfo("Found object index at page=%04x\n", pgndx);

              ret = spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_READ,
                                      fobj->objid,
                                      SPIFFS_PAGE_TO_PADDR(fs, pgndx),
                                      SPIFFS_GEO_PAGE_SIZE(fs), fs->work);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
                  return ret;
                }

              ret = spiffs_validate_objndx(&objhdr->phdr, fobj->objid,
                                           cur_objndx_spndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_validate_objndx() failed: %d\n", ret);
                  return ret;
                }

              cur_objndx_pgndx = pgndx;
            }

          fobj->objndx_pgndx = cur_objndx_pgndx;
          fobj->objndx_spndx = cur_objndx_spndx;
          fobj->offset       = offset + nwritten;
          prev_objndx_spndx  = cur_objndx_spndx;
        }

      /* Write partial data */

      to_write = MIN(len - nwritten, SPIFFS_DATA_PAGE_SIZE(fs) - page_offs);

      if (cur_objndx_spndx == 0)
        {
          /* Get data page from object index header page */

          orig_data_pgndx =
            ((FAR int16_t *)((FAR uint8_t *)objhdr +
              sizeof(struct spiffs_pgobj_ndxheader_s)))[data_spndx];
        }
      else
        {
          /* Get data page from object index page */

          orig_data_pgndx =
            ((FAR int16_t *)((FAR uint8_t *)objndx +
              sizeof(struct spiffs_page_objndx_s)))
                [SPIFFS_OBJNDX_ENTRY(fs, data_spndx)];
        }

      phdr.objid = fobj->objid & ~SPIFFS_OBJID_NDXFLAG;
      phdr.spndx = data_spndx;
      phdr.flags = 0xff;

      if (page_offs == 0 && to_write == SPIFFS_DATA_PAGE_SIZE(fs))
        {
          /* A full page, allocate and write a new page of data */

          ret = spiffs_page_allocate_data(fs,
                                          fobj->objid & ~SPIFFS_OBJID_NDXFLAG,
                                          &phdr, &data[nwritten], to_write,
                                          page_offs, 1, &data_pgndx);
          finfo("Store new data page, %04x:%04x offset=%d, len=%d, nwritten=%d\n",
                data_pgndx, data_spndx, page_offs, to_write, nwritten);
        }
      else
        {
          /* Write to existing page, allocate new and copy unmodified data */

          ret = spiffs_page_data_check(fs, fobj, orig_data_pgndx, data_spndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_page_data_check() failed: %d\n", ret);
              return ret;
            }

          ret = spiffs_page_allocate_data(fs,
                                          fobj->objid & ~SPIFFS_OBJID_NDXFLAG,
                                          &phdr, 0, 0, 0, 0, &data_pgndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_page_allocate_data() failed: %d\n", ret);
              break;
            }

          /* Copy unmodified data */

          if (page_offs > 0)
            {
              /* Before modification */

              ret = spiffs_phys_cpy(fs, fobj->objid,
                                    SPIFFS_PAGE_TO_PADDR(fs, data_pgndx) +
                                    sizeof(struct spiffs_page_header_s),
                                    SPIFFS_PAGE_TO_PADDR(fs, orig_data_pgndx) +
                                    sizeof(struct spiffs_page_header_s), page_offs);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_phys_cpy() failed: %d\n", ret);
                  break;
                }
            }

          if (page_offs + to_write < SPIFFS_DATA_PAGE_SIZE(fs))
            {
              /* After modification */

              ret =
                spiffs_phys_cpy(fs, fobj->objid,
                                SPIFFS_PAGE_TO_PADDR(fs, data_pgndx) +
                                sizeof(struct spiffs_page_header_s) + page_offs +
                                to_write,
                                SPIFFS_PAGE_TO_PADDR(fs, orig_data_pgndx)
                                + sizeof(struct spiffs_page_header_s) + page_offs +
                                to_write,
                                SPIFFS_DATA_PAGE_SIZE(fs) -
                                (page_offs + to_write));
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_phys_cpy() failed: %d\n", ret);
                  break;
                }
            }

          ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                                  fobj->objid, SPIFFS_PAGE_TO_PADDR(fs, data_pgndx) +
                                  sizeof(struct spiffs_page_header_s) + page_offs,
                                  to_write, &data[nwritten]);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
              break;
            }

          phdr.flags &= ~SPIFFS_PH_FLAG_FINAL;
          ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                                  fobj->objid,
                                  SPIFFS_PAGE_TO_PADDR(fs, data_pgndx) +
                                  offsetof(struct spiffs_page_header_s, flags),
                                  sizeof(uint8_t), (FAR uint8_t *)&phdr.flags);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
              break;
            }

          finfo("Store to existing data page, src=%04x, dest=%04x:%04x "
                "offset=%d, len=%d, nwritten=%d\n",
                orig_data_pgndx, data_pgndx, data_spndx, page_offs,
                to_write, nwritten);
        }

      /* Delete original data page */

      ret = spiffs_page_delete(fs, orig_data_pgndx);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_page_delete() failed: %d\n", ret);
          break;
        }

      /* Update memory representation of object index page with new data page */

      if (cur_objndx_spndx == 0)
        {
          /* Update object index header page */

          ((FAR int16_t *)((FAR uint8_t *)objhdr +
            sizeof(struct spiffs_pgobj_ndxheader_s)))[data_spndx] = data_pgndx;

          finfo("Wrote page %04x to objhdr entry=%04x in mem\n",
                data_pgndx, data_spndx);
        }
      else
        {
          /* Update object index page */

          ((FAR int16_t *)((FAR uint8_t *)objndx +
            sizeof(struct spiffs_page_objndx_s)))[SPIFFS_OBJNDX_ENTRY(fs, data_spndx)] =
              data_pgndx;

          finfo("Wrote page %04x to objndx entry %04x in mem\n",
                data_pgndx, (int16_t)SPIFFS_OBJNDX_ENTRY(fs, data_spndx));
        }

      /* Update internals */

      page_offs = 0;
      data_spndx++;
      nwritten  += to_write;
    }

  fobj->offset       = offset + nwritten;
  fobj->objndx_pgndx = cur_objndx_pgndx;
  fobj->objndx_spndx = cur_objndx_spndx;

  /* Finalize updated object indices */

  ret2 = OK;
  if (cur_objndx_spndx != 0)
    {
      int16_t new_objndx_pgndx;

      /* Wrote beyond object index header page.  Write last modified object
       * index page.  Move and update page
       */

      ret2 = spiffs_page_index_check(fs, fobj, cur_objndx_pgndx,
                                     cur_objndx_spndx);
      if (ret2 < 0)
        {
          ferr("ERROR: spiffs_page_index_check() failed: %d\n", ret2);
          return ret2;
        }

      ret2 = spiffs_page_move(fs, fobj->objid, (uint8_t *) objndx,
                              fobj->objid, 0, cur_objndx_pgndx,
                              &new_objndx_pgndx);

      finfo("Store modified objndx page, %04x:%04x, nwritten=%d\n",
            new_objndx_pgndx, cur_objndx_spndx, nwritten);

      fobj->objndx_pgndx = new_objndx_pgndx;
      fobj->objndx_spndx = cur_objndx_spndx;

      if (ret2 < 0)
        {
          ferr("ERROR: spiffs_page_move() failed: %d\n", ret2);
          return ret2;
        }

      spiffs_fobj_event(fs, (FAR struct spiffs_page_objndx_s *)objndx,
                        SPIFFS_EV_NDXUPD, fobj->objid, objndx->phdr.spndx,
                        new_objndx_pgndx, 0);
    }
  else
    {
      /* Wrote within object index header page */

      ret2 = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                       fobj->objhdr_pgndx, fs->work, 0, 0,
                                       &new_objhdr_pgndx);

      finfo("Store modified objhdr page, %04x:%04x, nwritten=%d\n",
            new_objhdr_pgndx, 0, nwritten);

      if (ret2 < 0)
        {
          ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
               ret2);
          return ret2;
        }
    }

  return nwritten;
}

/****************************************************************************
 * Name: spiffs_find_objhdr_pgndx
 *
 * Description:
 *   Finds object index header page by name
 *
 ****************************************************************************/

int spiffs_find_objhdr_pgndx(FAR struct spiffs_s *fs,
                             const uint8_t name[CONFIG_SPIFFS_NAME_MAX],
                             FAR int16_t *pgndx)
{
  int16_t blkndx;
  int entry;
  int ret;

  ret = spiffs_foreach_objlu(fs, fs->lu_blkndx, fs->lu_entry,
                             0, 0, spiffs_find_objhdr_pgndx_callback,
                             name, 0, &blkndx, &entry);

  if (ret == SPIFFS_VIS_END)
    {
      ret = -ENOENT;
    }
  else if (ret < 0)
    {
      ferr("ERROR: spiffs_foreach_objlu() failed: %d\n", ret);
      return ret;
    }

  if (pgndx != NULL)
    {
      *pgndx = SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, entry);
    }

  fs->lu_blkndx = blkndx;
  fs->lu_entry  = entry;

  return ret;
}

/****************************************************************************
 * Name: spiffs_fobj_truncate
 *
 * Description:
 *  Truncates object to new size. If new size is NULL, object may be removed
 *  totally
 *
 ****************************************************************************/

int spiffs_fobj_truncate(FAR struct spiffs_s *fs,
                         FAR struct spiffs_file_s *fobj, off_t new_size,
                         bool remove_full)
{
  FAR struct spiffs_pgobj_ndxheader_s *objhdr;
  FAR struct spiffs_page_objndx_s *objndx;
  uint32_t cur_size;
  int16_t objndx_pgndx;
  int16_t data_spndx;
  int16_t cur_objndx_spndx;
  int16_t prev_objndx_spndx;
  int16_t data_pgndx;
  int16_t new_objhdr_pgndx;
  int ret = OK;

  /* If the file has zero (or undefined) length and we were not asked to
   * remove the file, then there is nothing to do.
   */

  if ((fobj->size == SPIFFS_UNDEFINED_LEN || fobj->size == 0) && !remove_full)
    {
      /* Do nothing */

      return ret;
    }

  /* Need 2 pages if not removing: object index page + possibly chopped data
   * page
   */

  if (!remove_full)
    {
      ret = spiffs_gc_check(fs, SPIFFS_DATA_PAGE_SIZE(fs) * 2);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_gc_check() failed: %d\n", ret);
          return ret;
        }
    }

  objhdr            = (FAR struct spiffs_pgobj_ndxheader_s *)fs->work;
  objndx            = (FAR struct spiffs_page_objndx_s *)fs->work;
  objndx_pgndx      = fobj->objhdr_pgndx;
  cur_objndx_spndx  = 0;
  prev_objndx_spndx = (int16_t)-1;

  data_spndx        = 0;
  if (fobj->size > 0)
    {
      data_spndx    = (fobj->size - 1) / SPIFFS_DATA_PAGE_SIZE(fs);
    }

  cur_size          = 0;
  if (fobj->size != (uint32_t)SPIFFS_UNDEFINED_LEN)
    {
      cur_size      = fobj->size;
    }

  /* Before truncating, check if object is to be fully removed and mark this */

  if (remove_full && new_size == 0)
    {
      uint8_t flags = ~(SPIFFS_PH_FLAG_USED | SPIFFS_PH_FLAG_INDEX |
                        SPIFFS_PH_FLAG_FINAL | SPIFFS_PH_FLAG_NDXDELE);

      ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_UPDT,
                               fobj->objid,
                               SPIFFS_PAGE_TO_PADDR(fs, fobj->objhdr_pgndx) +
                               offsetof(struct spiffs_page_header_s, flags),
                               sizeof(uint8_t), (FAR uint8_t *)&flags);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
          return ret;
        }
    }

  /* Delete from end of object until desired len is reached */

  while (cur_size > new_size)
    {
      cur_objndx_spndx = SPIFFS_OBJNDX_ENTRY_SPNDX(fs, data_spndx);

      /* Put object index for current data span index in work buffer */

      if (prev_objndx_spndx != cur_objndx_spndx)
        {
          if (prev_objndx_spndx != (int16_t)- 1)
            {
              /* Remove previous object index page */

              finfo("Delete objndx page %04x:%04x\n",
                    objndx_pgndx, prev_objndx_spndx);

              ret = spiffs_page_index_check(fs, fobj, objndx_pgndx,
                                            prev_objndx_spndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_page_index_check() failed: %d\n",
                       ret);
                  return ret;
                }

              ret = spiffs_page_delete(fs, objndx_pgndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_page_delete() failed: %d\n", ret);
                  return ret;
                }

              spiffs_fobj_event(fs, NULL, SPIFFS_EV_NDXDEL, fobj->objid,
                                objndx->phdr.spndx, objndx_pgndx, 0);
              if (prev_objndx_spndx > 0)
                {
                  /* Update object index header page, unless we totally want
                   * to remove the file.
                   *
                   * If fully removing, we're not keeping consistency as
                   * good as when storing the header between chunks,
                   * would we be aborted. But when removing full files, a
                   * crammed system may otherwise report ERR_FULL a la
                   * windows.  We cannot have that.  Hence, take the risk -
                   * if aborted, a file check would free the lost pages and
                   * mend things as the file is marked as fully deleted in
                   * the beginning.
                   */

                  if (!remove_full)
                    {
                      finfo("Update objndx hdr page %04x:%04x to size=%d\n",
                            fobj->objhdr_pgndx, prev_objndx_spndx, cur_size);

                      ret = spiffs_fobj_update_ndxhdr(fs, fobj,
                                                      fobj->objid,
                                                      fobj->objhdr_pgndx,
                                                      0, 0, cur_size,
                                                      &new_objhdr_pgndx);
                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
                               ret);
                          return ret;
                        }
                    }

                  fobj->size = cur_size;
                }
            }

          /* Load current object index (header) page */

          if (cur_objndx_spndx == 0)
            {
              objndx_pgndx = fobj->objhdr_pgndx;
            }
          else
            {
              ret = spiffs_objlu_find_id_and_span(fs,
                                                  fobj->objid | SPIFFS_OBJID_NDXFLAG,
                                                  cur_objndx_spndx, 0,
                                                  &objndx_pgndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n",
                       ret);
                  return ret;
                }
            }

          finfo("Load objndx page %04x:%04x for data spndx=%04x\n",
                objndx_pgndx, cur_objndx_spndx, data_spndx);

          ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_READ,
                                  fobj->objid,
                                  SPIFFS_PAGE_TO_PADDR(fs, objndx_pgndx),
                                  SPIFFS_GEO_PAGE_SIZE(fs), fs->work);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
              return ret;
            }

          ret = spiffs_validate_objndx(&objhdr->phdr, fobj->objid,
                                       cur_objndx_spndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_validate_objndx() failed: %d\n", ret);
              return ret;
            }

          fobj->objndx_pgndx = objndx_pgndx;
          fobj->objndx_spndx = cur_objndx_spndx;
          fobj->offset       = cur_size;

          prev_objndx_spndx  = cur_objndx_spndx;
        }

      if (cur_objndx_spndx == 0)
        {
          /* Get data page from object index header page */

          data_pgndx =
            ((FAR int16_t *)((FAR uint8_t *) objhdr +
              sizeof(struct spiffs_pgobj_ndxheader_s)))[data_spndx];

          ((FAR int16_t *)((FAR uint8_t *)objhdr +
              sizeof(struct spiffs_pgobj_ndxheader_s)))[data_spndx] =
                SPIFFS_OBJID_FREE;
        }
      else
        {
          /* Get data page from object index page */

          data_pgndx =
            ((FAR int16_t *)((FAR uint8_t *)objndx +
              sizeof(struct spiffs_page_objndx_s)))
                [SPIFFS_OBJNDX_ENTRY(fs, data_spndx)];

          ((FAR int16_t *)((FAR uint8_t *)objndx +
            sizeof(struct spiffs_page_objndx_s)))
              [SPIFFS_OBJNDX_ENTRY(fs, data_spndx)] = SPIFFS_OBJID_FREE;
        }

      finfo("Got data pgndx %04x\n", data_pgndx);

      if (new_size == 0 || remove_full ||
          cur_size - new_size >= SPIFFS_DATA_PAGE_SIZE(fs))
        {
          /* Delete full data page */

          ret = spiffs_page_data_check(fs, fobj, data_pgndx, data_spndx);
          if (ret != SPIFFS_ERR_DELETED && ret < 0 &&
              ret != SPIFFS_ERR_INDEX_REF_FREE)
            {
              ferr("ERROR: Failed to validate data pgndx=%d\n", ret);
              break;
            }

          if (ret >= 0)
            {
              ret = spiffs_page_delete(fs, data_pgndx);
              if (ret < 0)
                {
                  ferr("ERROR: Failed to delete data pgndx=%d\n", ret);
                  break;
                }
            }
          else if (ret == SPIFFS_ERR_DELETED ||
                   ret == SPIFFS_ERR_INDEX_REF_FREE)
            {
              ret = OK;
            }

          /* Update current size */

          if (cur_size % SPIFFS_DATA_PAGE_SIZE(fs) == 0)
            {
              cur_size -= SPIFFS_DATA_PAGE_SIZE(fs);
            }
          else
            {
              cur_size -= cur_size % SPIFFS_DATA_PAGE_SIZE(fs);
            }

          fobj->size   = cur_size;
          fobj->offset = cur_size;

          finfo("Delete data page %04x for data spndx=%04x, cur_size=%d\n",
                data_pgndx, data_spndx, cur_size);
        }
      else
        {
          struct spiffs_page_header_s phdr;
          int16_t new_data_pgndx;
          uint32_t bytes_to_remove;

          /* Delete last page, partially */

          bytes_to_remove =
            SPIFFS_DATA_PAGE_SIZE(fs) - (new_size % SPIFFS_DATA_PAGE_SIZE(fs));

          finfo("Delete %d bytes from data page=%04x for data spndx=%04x, "
                "cur_size=%d\n",
                bytes_to_remove, data_pgndx, data_spndx, cur_size);

          ret = spiffs_page_data_check(fs, fobj, data_pgndx, data_spndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_page_data_check() failed: %d\n", ret);
              break;
            }

          phdr.objid = fobj->objid & ~SPIFFS_OBJID_NDXFLAG;
          phdr.spndx = data_spndx;
          phdr.flags = 0xff;

          /* Allocate new page and copy unmodified data */

          ret = spiffs_page_allocate_data(fs,
                                          fobj->objid & ~SPIFFS_OBJID_NDXFLAG,
                                          &phdr, 0, 0, 0, 0,
                                          &new_data_pgndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_page_allocate_data() failed: %d\n", ret);
              break;
            }

          ret = spiffs_phys_cpy(fs, 0,
                                SPIFFS_PAGE_TO_PADDR(fs, new_data_pgndx) +
                                sizeof(struct spiffs_page_header_s),
                                SPIFFS_PAGE_TO_PADDR(fs, data_pgndx) +
                                sizeof(struct spiffs_page_header_s),
                                SPIFFS_DATA_PAGE_SIZE(fs) - bytes_to_remove);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_phys_cpy() failed: %d\n", ret);
              break;
            }

          /* Delete original data page */

          ret = spiffs_page_delete(fs, data_pgndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_page_delete() failed: %d\n", ret);
              break;
            }

          phdr.flags &= ~SPIFFS_PH_FLAG_FINAL;
          ret = spiffs_cache_write(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_UPDT,
                                  fobj->objid,
                                  SPIFFS_PAGE_TO_PADDR(fs, new_data_pgndx) +
                                  offsetof(struct spiffs_page_header_s, flags),
                                  sizeof(uint8_t), (FAR uint8_t *)&phdr.flags);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_cache_write() failed: %d\n", ret);
              break;
            }

          /* Update memory representation of object index page with new data
           * page
           */

          if (cur_objndx_spndx == 0)
            {
              /* Update object index header page */

              ((FAR int16_t *)((FAR uint8_t *) objhdr +
                sizeof(struct spiffs_pgobj_ndxheader_s)))[data_spndx] =
                  new_data_pgndx;

              finfo("Wrote page=%04x to objhdr entry %04x in mem\n",
                    new_data_pgndx,
                    (int16_t)SPIFFS_OBJNDX_ENTRY(fs, data_spndx));
            }
          else
            {
              /* Update object index page */

              ((FAR int16_t *)((FAR uint8_t *)objndx +
                sizeof(struct spiffs_page_objndx_s)))
                  [SPIFFS_OBJNDX_ENTRY(fs, data_spndx)] = new_data_pgndx;

              finfo("Wrote page %04x to objndx entry=%04x in mem\n",
                    new_data_pgndx,
                    (int16_t)SPIFFS_OBJNDX_ENTRY(fs, data_spndx));
            }

          cur_size     = new_size;
          fobj->size   = new_size;
          fobj->offset = cur_size;
          break;
        }

      data_spndx--;
    }

  /* Update object indices */

  if (cur_objndx_spndx == 0)
    {
      /* Update object index header page */

      if (cur_size == 0)
        {
          if (remove_full)
            {
              /* Femove object altogether */

              finfo("Femove object index header page=%04x\n", objndx_pgndx);

              ret = spiffs_page_index_check(fs, fobj, objndx_pgndx, 0);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_page_index_check() failed: %d\n",
                       ret);
                  return ret;
                }

              ret = spiffs_page_delete(fs, objndx_pgndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_page_delete() failed: %d\n", ret);
                  return ret;
                }

              spiffs_fobj_event(fs, NULL, SPIFFS_EV_NDXDEL, fobj->objid,
                                0, objndx_pgndx, 0);
            }
          else
            {
              /* Make uninitialized object */

              finfo("Reset objhdr page=%04x\n", objndx_pgndx);

              memset(fs->work + sizeof(struct spiffs_pgobj_ndxheader_s),
                     0xff,
                     SPIFFS_GEO_PAGE_SIZE(fs) -
                     sizeof(struct spiffs_pgobj_ndxheader_s));

              ret = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                              objndx_pgndx, fs->work,
                                              0, SPIFFS_UNDEFINED_LEN,
                                              &new_objhdr_pgndx);

                {
                  ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
                       ret);
                  return ret;
                }
            }
        }
      else
        {
          /* Update object index header page */

          finfo("Update object index header page with indices and size\n");

          ret = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                          objndx_pgndx, fs->work, 0,
                                          cur_size, &new_objhdr_pgndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
                   ret);
              return ret;
            }
        }
    }
  else
    {
      int16_t new_objndx_pgndx;

      /* Update both current object index page and object index header page */

      ret = spiffs_page_index_check(fs, fobj, objndx_pgndx,
                                    cur_objndx_spndx);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_page_index_check() failed: %d\n", ret);
          return ret;
        }

      /* Move and update object index page */

      ret = spiffs_page_move(fs, fobj->objid, (FAR uint8_t *)objhdr,
                             fobj->objid, 0, objndx_pgndx,
                             &new_objndx_pgndx);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_page_move() failed: %d\n", ret);
          return ret;
        }

      spiffs_fobj_event(fs, (FAR struct spiffs_page_objndx_s *)objhdr,
                        SPIFFS_EV_NDXUPD, fobj->objid, objndx->phdr.spndx,
                        new_objndx_pgndx, 0);

      finfo("Store modified objndx page, %04x:%04x\n",
            new_objndx_pgndx, cur_objndx_spndx);

      fobj->objndx_pgndx = new_objndx_pgndx;
      fobj->objndx_spndx = cur_objndx_spndx;
      fobj->offset       = cur_size;

      /* Update object index header page with new size */

      ret = spiffs_fobj_update_ndxhdr(fs, fobj, fobj->objid,
                                           fobj->objhdr_pgndx, 0, 0,
                                           cur_size, &new_objhdr_pgndx);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n", ret);
          return ret;
        }
    }

  fobj->size = cur_size;
  return ret;
}

/****************************************************************************
 * Name: spiffs_object_read
 *
 * Description:
 *
 ****************************************************************************/

ssize_t spiffs_object_read(FAR struct spiffs_s *fs,
                           FAR struct spiffs_file_s *fobj, off_t offset,
                           size_t len, FAR uint8_t *dest)
{
  FAR struct spiffs_pgobj_ndxheader_s *objhdr;
  FAR struct spiffs_page_objndx_s *objndx ;
  uint32_t cur_offset;
  int16_t objndx_pgndx;
  int16_t data_pgndx;
  int16_t data_spndx;
  int16_t cur_objndx_spndx;
  int16_t prev_objndx_spndx;
  int ret = OK;

  objhdr            = (FAR struct spiffs_pgobj_ndxheader_s *)fs->work;
  objndx            = (FAR struct spiffs_page_objndx_s *)fs->work;

  data_spndx        = offset / SPIFFS_DATA_PAGE_SIZE(fs);
  cur_offset        = offset;
  prev_objndx_spndx = (int16_t)-1;

  while (cur_offset < offset + len)
    {
      uint32_t len_to_read;

      cur_objndx_spndx = SPIFFS_OBJNDX_ENTRY_SPNDX(fs, data_spndx);
      if (prev_objndx_spndx != cur_objndx_spndx)
        {
          /* Load current object index (header) page */

          if (cur_objndx_spndx == 0)
            {
              objndx_pgndx = fobj->objhdr_pgndx;
            }
          else
            {
              finfo("Find objndx %04x:%04x\n",
                     fobj->objid, cur_objndx_spndx);

              if (fobj->objndx_spndx == cur_objndx_spndx)
                {
                  objndx_pgndx = fobj->objndx_pgndx;
                }
              else
                {
                  ret =
                    spiffs_objlu_find_id_and_span(fs,
                                                  fobj->objid | SPIFFS_OBJID_NDXFLAG,
                                                  cur_objndx_spndx, 0,
                                                  &objndx_pgndx);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_objlu_find_id_and_span() failed: %d\n",
                           ret);
                      return ret;
                    }
                }
            }

          finfo("Load objndx page %d:%d for data spndx=%d\n",
                objndx_pgndx, cur_objndx_spndx, data_spndx);

          ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJNDX | SPIFFS_OP_C_READ,
                                  fobj->objid,
                                  SPIFFS_PAGE_TO_PADDR(fs, objndx_pgndx),
                                  SPIFFS_GEO_PAGE_SIZE(fs), fs->work);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
              return ret;
            }

          ret = spiffs_validate_objndx(&objndx->phdr, fobj->objid,
                                       cur_objndx_spndx);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_validate_objndx() failed: %d\n", ret);
              return ret;
            }

          fobj->offset       = cur_offset;
          fobj->objndx_pgndx = objndx_pgndx;
          fobj->objndx_spndx = cur_objndx_spndx;
          prev_objndx_spndx  = cur_objndx_spndx;
        }

      if (cur_objndx_spndx == 0)
        {
          /* Get data page from object index header page */

          data_pgndx =
            ((FAR int16_t *)((FAR uint8_t *)objhdr +
               sizeof(struct spiffs_pgobj_ndxheader_s)))[data_spndx];
        }
      else
        {
          /* Get data page from object index page */

          data_pgndx =
            ((FAR int16_t *)((FAR uint8_t *) objndx +
             sizeof(struct spiffs_page_objndx_s)))
               [SPIFFS_OBJNDX_ENTRY(fs, data_spndx)];
        }

      /* All remaining data */

      len_to_read = offset + len - cur_offset;

      /* Remaining data in page */

      len_to_read = MIN(len_to_read,
                        SPIFFS_DATA_PAGE_SIZE(fs) -
                        (cur_offset % SPIFFS_DATA_PAGE_SIZE(fs)));

      /* Remaining data in file */

      len_to_read = MIN(len_to_read, fobj->size);

      finfo("Read offset=%d rd=%d data spndx=%d is data_pgndx=%d addr=%p\n",
            cur_offset, len_to_read, data_spndx, data_pgndx,
            (SPIFFS_PAGE_TO_PADDR(fs, data_pgndx) +
             sizeof(struct spiffs_page_header_s) +
             (cur_offset % SPIFFS_DATA_PAGE_SIZE(fs))));

      if (len_to_read <= 0)
        {
          len = cur_offset - offset;
          break;
        }

      ret = spiffs_page_data_check(fs, fobj, data_pgndx, data_spndx);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_page_data_check() failed: %d\n", ret);
          return ret;
        }

      ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_DA | SPIFFS_OP_C_READ,
                              fobj->objid,
                              SPIFFS_PAGE_TO_PADDR(fs, data_pgndx) +
                              sizeof(struct spiffs_page_header_s) +
                              (cur_offset % SPIFFS_DATA_PAGE_SIZE(fs)),
                              len_to_read, dest);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
          return ret;
        }

      dest         += len_to_read;
      cur_offset   += len_to_read;
      fobj->offset = cur_offset;
      data_spndx++;
    }

  return len;
}

/****************************************************************************
 * Name: spiffs_objlu_find_free_objid
 *
 * Description:
 *   Scans through all object lookup for object index header pages. If total
 *   possible number of object ids cannot fit into a work buffer, these are
 *   grouped. When a group containing free object ids is found, the object
 *   lu is again scanned for object ids within group and bitmasked.  Finally,
 *   the bitmask is searched for a free objid
 *
 ****************************************************************************/

int spiffs_objlu_find_free_objid(FAR struct spiffs_s *fs, int16_t *objid,
                                  FAR const uint8_t *conflicting_name)
{
  uint32_t max_objects;
  struct spiffs_free_objid_s state;
  int16_t free_objid;
  int ret = OK;

  max_objects = (SPIFFS_GEO_BLOCK_COUNT(fs) * SPIFFS_OBJ_LOOKUP_MAX_ENTRIES(fs)) / 2;
  free_objid = SPIFFS_OBJID_FREE;

  state.min_objid = 1;
  state.max_objid = max_objects + 1;

  if ((state.max_objid & SPIFFS_OBJID_NDXFLAG) != 0)
    {
      state.max_objid = ((int16_t) - 1) & ~SPIFFS_OBJID_NDXFLAG;
    }

  state.compaction = 0;
  state.conflicting_name = conflicting_name;

  while (ret >= 0 && free_objid == SPIFFS_OBJID_FREE)
    {
      if (state.max_objid - state.min_objid <=
          (int16_t)SPIFFS_GEO_PAGE_SIZE(fs) * 8)
        {
          uint32_t i;
          uint32_t j;

          /* Possible to represent in bitmap */

          finfo("BITM min=%04x max=%04x\n",
                state.min_objid, state.max_objid);

          memset(fs->work, 0, SPIFFS_GEO_PAGE_SIZE(fs));
          ret = spiffs_foreach_objlu(fs, 0, 0, 0, 0,
                                     spiffs_objlu_find_free_objid_bitmap_callback,
                                     conflicting_name, &state.min_objid,
                                     0, 0);
          if (ret == SPIFFS_VIS_END)
            {
              ret = OK;
            }
          else if (ret < 0)
            {
              ferr("ERROR: spiffs_foreach_objlu() failed: %d\n", ret);
              return ret;
            }

          /* Traverse bitmask until found free objid */

          for (i = 0; i < SPIFFS_GEO_PAGE_SIZE(fs); i++)
            {
              uint8_t mask = fs->work[i];

              if (mask == 0xff)
                {
                  continue;
                }

              for (j = 0; j < 8; j++)
                {
                  if ((mask & (1 << j)) == 0)
                    {
                      *objid = (i << 3) + j + state.min_objid;
                      return OK;
                    }
                }
            }

          return -ENOSPC;
        }
      else
        {
          /* Not possible to represent all ids in range in a bitmap, compact
           * and count
           */

          if (state.compaction != 0)
            {
              uint32_t i;
              uint32_t min_i;
              uint8_t *map;
              uint8_t min_count;

              /* Select element in compacted table, decrease range and
               * recompact
               */

              min_i     = 0;
              map       = (uint8_t *)fs->work;
              min_count = 0xff;

              for (i = 0;
                   i < SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(uint8_t);
                   i++)
                {
                  if (map[i] < min_count)
                    {
                      min_count = map[i];
                      min_i = i;
                      if (min_count == 0)
                        {
                          break;
                        }
                    }
                }

              if (min_count == state.compaction)
                {
                  /* There are no free objids! */

                  fwarn("WARNING: Compacted table is full\n");
                  return -ENOSPC;
                }

              finfo("COMP select index=%d min_count=%d min=%04x max=%04x "
                    "compact:%d\n",
                    min_i, min_count, state.min_objid, state.max_objid,
                    state.compaction);

              if (min_count == 0)
                {
                  /* No objid in this range, skip compacting and use directly */

                  *objid = min_i * state.compaction + state.min_objid;
                  return OK;
                }
              else
                {
                  finfo("COMP SEL chunk=%d min=%04x -> %04x\n",
                        state.compaction, state.min_objid,
                        state.min_objid + min_i * state.compaction);

                  state.min_objid += min_i * state.compaction;
                  state.max_objid  = state.min_objid + state.compaction;

                  /* Decrease compaction */
                }

              if ((state.max_objid - state.min_objid <=
                   (int16_t)SPIFFS_GEO_PAGE_SIZE(fs) * 8))
                {
                  /* No need for compacting, use bitmap */

                  continue;
                }
            }

          /* In a work memory of <page_size> bytes, we may fit in
           * <page_size> IDs. NOTE: Currently page size is equivalent to
           * block size.
           *
           * TODO: What if compaction is > 255 - then we cannot fit it in a
           * byte
           */

          state.compaction =
            (state.max_objid -
             state.min_objid) / ((SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(uint8_t)));

          finfo("COMP min=%04x max=%04x compact=%d\n",
                state.min_objid, state.max_objid, state.compaction);

          memset(fs->work, 0, SPIFFS_GEO_PAGE_SIZE(fs));
          ret = spiffs_foreach_objlu(fs, 0, 0, 0, 0,
                                     spiffs_objlu_find_free_objid_compact_callback,
                                     &state, 0, 0, 0);
          if (ret == SPIFFS_VIS_END)
            {
              ret = OK;
            }
          else if (ret < 0)
            {
              ferr("ERROR: spiffs_foreach_objlu() failed: %d\n", ret);
              return ret;
            }

          state.conflicting_name = 0;   /* Searched for conflicting name once,
                                         * no need to do it again */
        }
    }

  return ret;
}
