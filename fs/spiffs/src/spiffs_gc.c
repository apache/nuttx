/****************************************************************************
 * fs/spiffs.h/spiffs_gc.c
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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>

#include "spiffs.h"
#include "spiffs_core.h"
#include "spiffs_cache.h"
#include "spiffs_gc.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum spiffs_gc_clean_state_e
{
  FIND_OBJ_DATA,
  MOVE_OBJ_DATA,
  MOVE_OBJ_NDX,
  FINISHED
};

struct spiffs_gc_s
{
  enum spiffs_gc_clean_state_e state;
  int16_t cur_objid;
  int16_t cur_objndx_spndx;
  int16_t cur_objndx_pgndx;
  int16_t cur_data_pgndx;
  int stored_scan_entry_index;
  bool objid_found;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_gc_erase_block
 *
 * Description:
 *   Erases a logical block and updates the erase counter.
 *   If cache is enabled, all pages that might be cached in this block
 *   is dropped.
 *
 * Input Parameters:
 *   fs     - A reference to the SPIFFS volume object instance
 *   blkndx - The block index to erase
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.
 *
 ****************************************************************************/

static int spiffs_gc_erase_block(FAR struct spiffs_s *fs, int16_t blkndx)
{
  int ret;
  int i;

  spiffs_gcinfo("Erase block=%04x\n", blkndx);

  /* Erase the block */

  ret = spiffs_erase_block(fs, blkndx);
  if (ret < 0)
    {
      ferr("ERROR: spiffs_erase_block() blkndx=%d failed: %d\n",
           blkndx, ret);
    }

  /* Then remove the pages from the cache. */

  for (i = 0; i < SPIFFS_GEO_PAGES_PER_BLOCK(fs); i++)
    {
      spiffs_cache_drop_page(fs, SPIFFS_PAGE_FOR_BLOCK(fs, blkndx) + i);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_gc_epage_stats
 *
 * Description:
 *   Checks if garbage collecting is necessary. If so a candidate block is
 *   found, cleansed and erased
 *
 *   This function will  try to make room for given amount of bytes in the
 *   filesystem by moving pages and erasing blocks.
 *
 *   If it is physically impossible, err_no will be set to -ENOSPC. If
 *   there already is this amount (or more) of free space, SPIFFS_gc will
 *   silently return. It is recommended to call statfs() before invoking
 *   this method in order to determine what amount of bytes to give.
 *
 *   NB: the garbage collector is automatically called when spiffs needs free
 *   pages. The reason for this function is to give possibility to do
 *   background tidying when user knows the system is idle.
 *
 * Input Parameters:
 *   fs            the file system struct
 *   len           amount of data that should be freed
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  -ENODATA is returned if no blocks were deleted.
 *
 ****************************************************************************/

/* Updates page statistics for a block that is about to be erased */

static int spiffs_gc_epage_stats(FAR struct spiffs_s *fs, int16_t blkndx)
{
  FAR int16_t *objlu_buf = (int16_t *) fs->lu_work;
  uint32_t dele          = 0;
  uint32_t allo          = 0;
  int entries_per_page   = (SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(int16_t));
  int cur_entry          = 0;
  int obj_lookup_page    = 0;
  int ret                = OK;

  /* Check each object lookup page */

  while (ret >= 0 && obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
    {
      int entry_offset = obj_lookup_page * entries_per_page;
      ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ, 0,
                              blkndx * SPIFFS_GEO_BLOCK_SIZE(fs) +
                              SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                              SPIFFS_GEO_PAGE_SIZE(fs), fs->lu_work);

      /* Check each entry */

      while (ret >= 0 &&
             cur_entry - entry_offset < entries_per_page &&
             cur_entry <
             (int)(SPIFFS_GEO_PAGES_PER_BLOCK(fs) -
                   SPIFFS_OBJ_LOOKUP_PAGES(fs)))
        {
          int16_t id = objlu_buf[cur_entry - entry_offset];

          if (id == SPIFFS_OBJID_FREE)
            {
            }
          else if (id == SPIFFS_OBJID_DELETED)
            {
              dele++;
            }
          else
            {
              allo++;
            }

          cur_entry++;
        }

      obj_lookup_page++;
    }

  spiffs_gcinfo("Wipe pallo=%d pdele=%d\n", allo,  dele);

  fs->alloc_pages   -= allo;
  fs->deleted_pages -= dele;
  return ret;
}

/****************************************************************************
 * Name: spiffs_gc_find_candidate
 *
 * Description:
 *   Finds block candidates to erase
 *
 * Input Parameters:
 *   fs               - A reference to the SPIFFS volume object instance
 *   block_candidates - Memory to return an array of candidates
 *   candidate_count  - Memory to return the number of candidates
 *   fs_crammed       - True: Filesystem is full
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  -ENODATA is returned if no blocks were deleted.
 *
 ****************************************************************************/

static int spiffs_gc_find_candidate(FAR struct spiffs_s *fs,
                                    FAR int16_t **block_candidates,
                                    FAR int *candidate_count,
                                    bool fs_crammed)
{
  FAR int32_t *cand_scores;
  FAR int16_t *objlu_buf  = (FAR int16_t *)fs->lu_work;
  FAR int16_t *cand_blocks;
  uint32_t blocks         = SPIFFS_GEO_BLOCK_COUNT(fs);
  int16_t cur_block       = 0;
  uint32_t cur_block_addr = 0;
  int entries_per_page;
  int max_candidates;
  int cur_entry           = 0;
  int ret                 = OK;

  /* Using fs->work area as sorted candidate memory,
   * (int16_t)cand_blkndx/(int32_t)score
   */

  max_candidates =
    MIN(SPIFFS_GEO_BLOCK_COUNT(fs),
        (SPIFFS_GEO_PAGE_SIZE(fs) - 8) /
         (sizeof(int16_t) + sizeof(int32_t)));
  *candidate_count = 0;
  memset(fs->work, 0xff, SPIFFS_GEO_PAGE_SIZE(fs));

  /* Divide up work area into block indices and scores */

  cand_blocks = (FAR int16_t *)fs->work;
  cand_scores =
    (FAR int32_t *)(fs->work + max_candidates * sizeof(int16_t));

  /* Align cand_scores on int32_t boundary */

  cand_scores =
    (FAR int32_t *)(((intptr_t)cand_scores + sizeof(intptr_t) - 1) &
                    ~(sizeof(intptr_t) - 1));

  *block_candidates = cand_blocks;

  entries_per_page = (SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(int16_t));

  /* Check each block */

  while (ret >= 0 && blocks--)
    {
      uint16_t deleted_pages_in_block = 0;
      uint16_t used_pages_in_block    = 0;
      int obj_lookup_page             = 0;

      /* check each object lookup page */

      while (ret >= 0 &&
             obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
          int entry_offset = obj_lookup_page * entries_per_page;
          ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                                  0, cur_block_addr +
                                  SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                                  SPIFFS_GEO_PAGE_SIZE(fs), fs->lu_work);

          /* Check each entry */

          while (ret >= 0 &&
                 cur_entry - entry_offset < entries_per_page &&
                 cur_entry <
                 (int)(SPIFFS_GEO_PAGES_PER_BLOCK(fs) -
                       SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
              int16_t id = objlu_buf[cur_entry - entry_offset];

              if (id == SPIFFS_OBJID_FREE)
                {
                  /* When a free entry is encountered, scan logic ensures
                   * that all following entries are free also
                   */

                  ret = 1;      /* Kill object lu loop */
                  break;
                }
              else if (id == SPIFFS_OBJID_DELETED)
                {
                  deleted_pages_in_block++;
                }
              else
                {
                  used_pages_in_block++;
                }

              cur_entry++;
            }

          obj_lookup_page++;
        }

      if (ret == 1)
        {
          ret = OK;
        }

      /* Calculate score and insert into candidate table stoneage sort, but
       * probably not so many blocks
       */

      if (ret >= 0 /* && deleted_pages_in_block > 0 */)
        {
          int32_t score;
          int16_t erase_count;
          int16_t erase_age;
          int candndx;

          /* Read erase count */

          ret = spiffs_cache_read(fs, SPIFFS_OP_C_READ | SPIFFS_OP_T_OBJ_LU2,
                                  0, SPIFFS_ERASE_COUNT_PADDR(fs, cur_block),
                                  sizeof(int16_t), (uint8_t *)&erase_count);
          if (ret < 0)
            {
              ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
              return ret;
            }

          /* Calculate the erase age */

          if (fs->max_erase_count > erase_count)
            {
              erase_age = fs->max_erase_count - erase_count;
            }
          else
            {
              erase_age =
                SPIFFS_OBJID_FREE - (erase_count - fs->max_erase_count);
            }

          score =
            deleted_pages_in_block * CONFIG_SPIFFS_GC_DELWGT +
            used_pages_in_block * CONFIG_SPIFFS_GC_USEDWGT +
            erase_age * (fs_crammed ? 0 : CONFIG_SPIFFS_GC_ERASEAGEWGT);

          candndx = 0;

          spiffs_gcinfo("blkndx=%04x del=%d use=%d score=%d\n",
                        cur_block, deleted_pages_in_block,
                        used_pages_in_block, score);

          while (candndx < max_candidates)
            {
              if (cand_blocks[candndx] == (int16_t) - 1)
                {
                  cand_blocks[candndx] = cur_block;
                  cand_scores[candndx] = score;
                  break;
                }
              else if (cand_scores[candndx] < score)
                {
                  int reorder_candndx = max_candidates - 2;
                  while (reorder_candndx >= candndx)
                    {
                      cand_blocks[reorder_candndx + 1] =
                        cand_blocks[reorder_candndx];
                      cand_scores[reorder_candndx + 1] =
                        cand_scores[reorder_candndx];
                      reorder_candndx--;
                    }

                  cand_blocks[candndx] = cur_block;
                  cand_scores[candndx] = score;
                  break;
                }

              candndx++;
            }

          (*candidate_count)++;
        }

      cur_entry = 0;
      cur_block++;
      cur_block_addr += SPIFFS_GEO_BLOCK_SIZE(fs);
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_gc_clean
 *
 * Description:
 *   Empties given block by moving all data into free pages of another block
 *   Strategy:
 *     loop:
 *     - Scan object lookup for object data pages.
 *     - For first found id, check spndx and load corresponding object index
 *       page to memory.
 *     - Push object scan lookup entry index.
 *         Rescan object lookup, find data pages with same id and referenced
 *           by same object index.
 *         Move data page, update object index in memory.
 *         When reached end of lookup, store updated object index.
 *     - Pop object scan lookup entry index.
 *     - Repeat loop until end of object lookup.
 *     - Scan object lookup again for remaining object index pages, move to
 *       new page in other block.
 *
 * Input Parameters:
 *   fs            the file system struct
 *   len           amount of data that should be freed
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  -ENODATA is returned if no blocks were deleted.
 *
 ****************************************************************************/

static int spiffs_gc_clean(FAR struct spiffs_s *fs, int16_t blkndx)
{
  const int entries_per_page = (SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(int16_t));
  int ret = OK;

  /* This is the global localizer being pushed and popped */

  struct spiffs_gc_s gc;                 /* Our stack frame/state */
  FAR int16_t *objlu_buf;
  FAR struct spiffs_pgobj_ndxheader_s *objhdr;
  FAR struct spiffs_page_objndx_s *objndx;
  int16_t cur_pgndx      = 0;
  int cur_entry          = 0;

  spiffs_gcinfo("Cleaning block %04x\n", blkndx);

  memset(&gc, 0, sizeof(struct spiffs_gc_s));
  gc.state  = FIND_OBJ_DATA;

  objlu_buf = (FAR int16_t *)fs->lu_work;
  objhdr    = (FAR struct spiffs_pgobj_ndxheader_s *)fs->work;
  objndx    = (struct spiffs_page_objndx_s *)fs->work;

  if (fs->free_blkndx == blkndx)
    {
      /* Move free cursor to next block, cannot use free pages from the block
       * we want to clean
       */

      fs->free_blkndx = (blkndx + 1) % SPIFFS_GEO_BLOCK_COUNT(fs);
      fs->free_entry = 0;
      spiffs_gcinfo("Move free cursor to block=%04x\n", fs->free_blkndx);
    }

  while (ret >= 0 && gc.state != FINISHED)
    {
      int obj_lookup_page;
      uint8_t scan;

      spiffs_gcinfo("state=%d entry=%d\n", gc.state, cur_entry);

      gc.objid_found  = false;  /* Reset (to no found data page) */

      /* Scan through lookup pages */

      obj_lookup_page = cur_entry / entries_per_page;
      scan            = 1;

      /* Check each object lookup page */

      while (scan && ret >= 0 &&
             obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
          int entry_offset = obj_lookup_page * entries_per_page;

          ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                                  0, blkndx * SPIFFS_GEO_BLOCK_SIZE(fs) +
                                  SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                                  SPIFFS_GEO_PAGE_SIZE(fs), fs->lu_work);

          /* Check each object lookup entry */

          while (scan && ret >= 0 &&
                 cur_entry - entry_offset < entries_per_page &&
                 cur_entry <
                 (int)(SPIFFS_GEO_PAGES_PER_BLOCK(fs) -
                       SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
              int16_t id = objlu_buf[cur_entry - entry_offset];

              cur_pgndx =
                SPIFFS_OBJ_LOOKUP_ENTRY_TO_PGNDX(fs, blkndx, cur_entry);

              /* Act upon object id depending on gc state */

              switch (gc.state)
                {
                case FIND_OBJ_DATA:

                  /* Find a data page. */

                  if (id != SPIFFS_OBJID_DELETED &&
                      id != SPIFFS_OBJID_FREE &&
                      ((id & SPIFFS_OBJID_NDXFLAG) == 0))
                    {
                      /* Found a data page, stop scanning and handle in
                       * switch case below
                       */

                      spiffs_gcinfo(
                        "Found data page, state=%d, objid=%04x\n",
                        gc.state, id);

                      gc.objid_found    = true;
                      gc.cur_objid      = id;
                      gc.cur_data_pgndx = cur_pgndx;
                      scan              = 0;
                    }
                  break;

                case MOVE_OBJ_DATA:

                  /* Evacuate found data pages for corresponding object index
                   * we have in memory, update memory representation
                   */

                  if (id == gc.cur_objid)
                    {
                      struct spiffs_page_header_s phdr;

                      ret = spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                      0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                      sizeof(struct spiffs_page_header_s),
                                      (FAR uint8_t *)&phdr);
                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_cache_read() failed: %d\n",
                               ret);
                          return ret;
                        }

                      spiffs_gcinfo("Found data page %04x:%04x @%04x\n",
                                    gc.cur_objid, phdr.spndx, cur_pgndx);

                      if (SPIFFS_OBJNDX_ENTRY_SPNDX(fs, phdr.spndx) !=
                          gc.cur_objndx_spndx)
                        {
                          spiffs_gcinfo(
                            "No objndx spndx match, take in another run\n");
                        }
                      else
                        {
                          int16_t new_data_pgndx;
                          if (phdr.flags & SPIFFS_PH_FLAG_DELET)
                            {
                              /* Move page */

                              ret = spiffs_page_move(fs, 0, 0, id, &phdr,
                                                 cur_pgndx, &new_data_pgndx);

                              spiffs_gcinfo(
                                "Move objndx=%04x:%04x page=%04x to %04x\n",
                                gc.cur_objid, phdr.spndx,
                                cur_pgndx, new_data_pgndx);

                              if (ret < 0)
                                {
                                  ferr("ERROR: spiffs_page_move(): %d\n",
                                       ret);
                                  return ret;
                                }

                              /* Move wipes obj_lu, reload it */

                              ret =
                                spiffs_cache_read(fs,
                                   SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                                   0,
                                   blkndx * SPIFFS_GEO_BLOCK_SIZE(fs) +
                                   SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                                   SPIFFS_GEO_PAGE_SIZE(fs),
                                   fs->lu_work);
                              if (ret < 0)
                                {
                                  ferr("ERROR: spiffs_cache_read(): %d\n",
                                       ret);
                                  return ret;
                                }
                            }
                          else
                            {
                              /* Page is deleted but not deleted in lookup.
                               * Scrap it - might seem unnecessary as we will
                               * erase this block, but we might get aborted
                               */

                              spiffs_gcinfo(
                                "Wipe objndx=%04x:%04x page=%04x\n",
                                id, phdr.spndx, cur_pgndx);

                              ret = spiffs_page_delete(fs, cur_pgndx);
                              if (ret < 0)
                                {
                                  ferr("ERROR: spiffs_page_delete(): %d\n",
                                       ret);
                                  return ret;
                                }

                              new_data_pgndx = SPIFFS_OBJID_FREE;
                            }

                          /* Update memory representation of object index
                           * page with new data page
                           */

                          if (gc.cur_objndx_spndx == 0)
                            {
                              /* Update object index header page */

                              ((FAR int16_t *)((FAR uint8_t *)objhdr +
                                 sizeof(struct spiffs_pgobj_ndxheader_s)))
                                 [phdr.spndx] = new_data_pgndx;

                              spiffs_gcinfo(
                                "Wrote page=%04x to objhdr entry=%04x\n",
                                new_data_pgndx,
                                SPIFFS_OBJNDX_ENTRY(fs, phdr.spndx));
                            }
                          else
                            {
                              /* Update object index page */

                              ((FAR int16_t *)((FAR uint8_t *)objndx +
                                sizeof(struct spiffs_page_objndx_s)))
                                  [SPIFFS_OBJNDX_ENTRY(fs, phdr.spndx)] =
                                    new_data_pgndx;

                              spiffs_gcinfo(
                                "Wrote page=%04x to objndx entry=%04x\n",
                                new_data_pgndx,
                                SPIFFS_OBJNDX_ENTRY(fs, phdr.spndx));
                            }
                        }
                    }
                  break;

                case MOVE_OBJ_NDX:

                  /* Find and evacuate object index pages */

                  if (id != SPIFFS_OBJID_DELETED &&
                      id != SPIFFS_OBJID_FREE &&
                      (id & SPIFFS_OBJID_NDXFLAG) != 0)
                    {
                      /* Found an index object id */

                      struct spiffs_page_header_s phdr;
                      int16_t new_pgndx;

                      /* Load header */

                      ret = spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                      0,
                                      SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                      sizeof(struct spiffs_page_header_s),
                                      (FAR uint8_t *)&phdr);
                      if (ret < 0)
                        {
                          ferr("ERROR: spiffs_cache_read() failed: %d\n",
                               ret);
                          return ret;
                        }

                      if (phdr.flags & SPIFFS_PH_FLAG_DELET)
                        {
                          /* Move page */

                          ret = spiffs_page_move(fs, 0, 0, id, &phdr,
                                                 cur_pgndx, &new_pgndx);

                          spiffs_gcinfo(
                            "Move objndx=%04x:%04x page=%04x to %04x\n",
                            id, phdr.spndx, cur_pgndx, new_pgndx);

                          if (ret < 0)
                            {
                              ferr("ERROR: spiffs_page_move() failed: %d\n",
                                   ret);
                              return ret;
                            }

                          spiffs_fobj_event(fs,
                                    (FAR struct spiffs_page_objndx_s *)&phdr,
                                    SPIFFS_EV_NDXMOV, id,
                                    phdr.spndx, new_pgndx, 0);

                          /* Move wipes obj_lu, reload it */

                          ret =
                            spiffs_cache_read(fs,
                                   SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                                   0,
                                   blkndx * SPIFFS_GEO_BLOCK_SIZE(fs) +
                                   SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                                   SPIFFS_GEO_PAGE_SIZE(fs), fs->lu_work);
                          if (ret < 0)
                            {
                              ferr("ERROR: spiffs_cache_read() failed: %d\n",
                                   ret);
                              return ret;
                            }
                        }
                      else
                        {
                          /* Page is deleted but not deleted in lookup, scrap
                           * it - might seem unnecessary as we will erase
                           * this block, but we might get aborted
                           */

                          spiffs_gcinfo("Wipe objndx=%04x:%04x page=%04x\n",
                                        id, phdr.spndx, cur_pgndx);

                          ret = spiffs_page_delete(fs, cur_pgndx);
                          if (ret < 0)
                            {
                              ferr("ERROR: spiffs_fobj_event() failed: %d\n",
                                   ret);
                              return ret;
                            }

                          spiffs_fobj_event(fs, NULL,
                                            SPIFFS_EV_NDXDEL, id,
                                            phdr.spndx, cur_pgndx, 0);
                        }
                    }
                  break;

                default:
                  scan = 0;
                  break;
                }

              cur_entry++;
            }

          obj_lookup_page++;    /* No need to check scan variable here,
                                 * obj_lookup_page is set in start of loop
                                 */
        }

      if (ret < 0)
        {
          break;
        }

      /* State finalization and switch */

      switch (gc.state)
        {
        case FIND_OBJ_DATA:
          if (gc.objid_found)
            {
              struct spiffs_page_header_s phdr;
              int16_t objndx_pgndx;

              /* Handle found data page.  Find out corresponding objndx page
               * and load it to memory
               */

              gc.stored_scan_entry_index = cur_entry; /* Push cursor */
              cur_entry                  = 0;         /* Sestart scan from start */
              gc.state                   = MOVE_OBJ_DATA;

              ret = spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                      0, SPIFFS_PAGE_TO_PADDR(fs, cur_pgndx),
                                      sizeof(struct spiffs_page_header_s),
                                      (FAR uint8_t *)&phdr);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_fobj_event() failed: %d\n", ret);
                  return ret;
                }

              gc.cur_objndx_spndx =
                SPIFFS_OBJNDX_ENTRY_SPNDX(fs, phdr.spndx);

              spiffs_gcinfo("Find objndx spndx=%04x\n", gc.cur_objndx_spndx);

              ret = spiffs_objlu_find_id_and_span(fs,
                                         gc.cur_objid | SPIFFS_OBJID_NDXFLAG,
                                         gc.cur_objndx_spndx, 0,
                                         &objndx_pgndx);
              if (ret == -ENOENT)
                {
                  /* On borked systems we might get an ERR_NOT_FOUND here -
                   * this is handled by simply deleting the page as it is not
                   * referenced from anywhere
                   */

                  spiffs_gcinfo("objndx not found! Wipe page %04x\n",
                                gc.cur_data_pgndx);

                  ret = spiffs_page_delete(fs, gc.cur_data_pgndx);
                  if (ret < 0)
                    {
                      ferr("ERROR: spiffs_page_delete() failed: %d\n", ret);
                      return ret;
                    }

                  /* Then we restore states and continue scanning for data
                   * pages
                   */

                  cur_entry = gc.stored_scan_entry_index;  /* pop cursor */
                  gc.state  = FIND_OBJ_DATA;
                  break;
                }

              if (ret < 0)
                {
                  return ret;
                }

              spiffs_gcinfo("Found object index at page=%04x\n",
                            objndx_pgndx);

              ret = spiffs_cache_read(fs,
                                      SPIFFS_OP_T_OBJ_LU2 | SPIFFS_OP_C_READ,
                                      0,
                                      SPIFFS_PAGE_TO_PADDR(fs, objndx_pgndx),
                                      SPIFFS_GEO_PAGE_SIZE(fs), fs->work);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_cache_read() failed: %d\n", ret);
                  return ret;
                }

              /* Cannot allow a gc if the presumed index in fact is no
               * index, a check must run or lot of data may be lost
               */

              ret = spiffs_validate_objndx(&objndx->phdr,
                                         gc.cur_objid | SPIFFS_OBJID_NDXFLAG,
                                         gc.cur_objndx_spndx);
              if (ret < 0)
                {
                  ferr("ERROR: spiffs_validate_objndx() failed: %d\n", ret);
                  return ret;
                }

              gc.cur_objndx_pgndx = objndx_pgndx;
            }
          else
            {
              /* No more data pages found, passed through all block, start
               * evacuating object indices
               */

              gc.state  = MOVE_OBJ_NDX;
              cur_entry = 0;    /* Restart entry scan index */
            }
          break;

        case MOVE_OBJ_DATA:
          {
            int16_t new_objndx_pgndx;

            /* Store modified objndx (hdr) page residing in memory now that
             * all data pages belonging to this object index and residing in
             * the block we want to evacuate
             */

            gc.state  = FIND_OBJ_DATA;
            cur_entry = gc.stored_scan_entry_index;     /* pop cursor */

            if (gc.cur_objndx_spndx == 0)
              {
                /* Store object index header page */

                ret = spiffs_fobj_update_ndxhdr(fs, 0,
                                         gc.cur_objid | SPIFFS_OBJID_NDXFLAG,
                                         gc.cur_objndx_pgndx, fs->work, 0,
                                         0, &new_objndx_pgndx);

                spiffs_gcinfo("Store modified objhdr page=%04x:%04x\n",
                              new_objndx_pgndx, 0);

                if (ret < 0)
                  {
                    ferr("ERROR: spiffs_fobj_update_ndxhdr() failed: %d\n",
                         ret);
                    return ret;
                  }
              }
            else
              {
                /* Store object index page */

                ret =  spiffs_page_move(fs, 0, fs->work,
                                        gc.cur_objid | SPIFFS_OBJID_NDXFLAG,
                                        0, gc.cur_objndx_pgndx,
                                        &new_objndx_pgndx);

                spiffs_gcinfo("Store modified objndx page=%04x:%04x\n",
                              new_objndx_pgndx, objndx->phdr.spndx);

                if (ret < 0)
                  {
                    ferr("ERROR: spiffs_page_move() failed: %d\n", ret);
                    return ret;
                  }

                spiffs_fobj_event(fs,
                                 (FAR struct spiffs_page_objndx_s *)fs->work,
                                 SPIFFS_EV_NDXUPD, gc.cur_objid,
                                 objndx->phdr.spndx,
                                 new_objndx_pgndx, 0);
              }
          }
          break;

        case MOVE_OBJ_NDX:
          /* Scanned through all blocks, no more object indices found - our
           * work here is done
           */

          gc.state = FINISHED;
          break;

        default:
          cur_entry = 0;
          break;
        }

      spiffs_gcinfo("state=%d\n", gc.state);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spiffs_gc_quick
 *
 * Description:
 *   Searches for blocks where all entries are deleted - if one is found,
 *   the block is erased. Compared to the non-quick gc, the quick one ensures
 *   that no updates are needed on existing objects on pages that are erased.
 *
 * Input Parameters:
 *   fs             - A reference to the SPIFFS volume object instance
 *   max_free_pages - The maximum pages to free
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  -ENODATA is returned if no blocks were deleted.
 *
 ****************************************************************************/

int spiffs_gc_quick(FAR struct spiffs_s *fs, uint16_t max_free_pages)
{
  FAR int16_t *objlu_buf  = (int16_t *) fs->lu_work;
  uint32_t cur_block_addr = 0;
  uint32_t blocks         = SPIFFS_GEO_BLOCK_COUNT(fs);
  int16_t cur_block       = 0;
  int entries_per_page    = (SPIFFS_GEO_PAGE_SIZE(fs) / sizeof(int16_t));
  int cur_entry           = 0;
  int ret                 = OK;

  spiffs_gcinfo("max_free_pages=%u\n", max_free_pages);
#ifdef CONFIG_SPIFFS_GCDBG
  fs->stats_gc_runs++;
#endif

  /* Find fully deleted blocks */

  while (ret >= 0 && blocks--)
    {
      uint16_t deleted_pages_in_block = 0;
      uint16_t free_pages_in_block = 0;
      int obj_lookup_page = 0;

      /* Check each object lookup page */

      while (ret >= 0 &&
             obj_lookup_page < (int)SPIFFS_OBJ_LOOKUP_PAGES(fs))
        {
          int entry_offset = obj_lookup_page * entries_per_page;

          ret = spiffs_cache_read(fs, SPIFFS_OP_T_OBJ_LU | SPIFFS_OP_C_READ,
                                  0, cur_block_addr +
                                  SPIFFS_PAGE_TO_PADDR(fs, obj_lookup_page),
                                  SPIFFS_GEO_PAGE_SIZE(fs), fs->lu_work);

          /* Check each entry */

          while (ret >= 0 &&
                 cur_entry - entry_offset < entries_per_page &&
                 cur_entry <
                 (int)(SPIFFS_GEO_PAGES_PER_BLOCK(fs) -
                       SPIFFS_OBJ_LOOKUP_PAGES(fs)))
            {
              int16_t id = objlu_buf[cur_entry - entry_offset];

              if (id == SPIFFS_OBJID_DELETED)
                {
                  deleted_pages_in_block++;
                }
              else if (id == SPIFFS_OBJID_FREE)
                {
                  /* Kill scan, go for next block */

                  free_pages_in_block++;
                  if (free_pages_in_block > max_free_pages)
                    {
                      obj_lookup_page = SPIFFS_OBJ_LOOKUP_PAGES(fs);
                      ret = 1;  /* Kill object lu loop */
                      break;
                    }
                }
              else
                {
                  /* Kill scan, go for next block */

                  obj_lookup_page = SPIFFS_OBJ_LOOKUP_PAGES(fs);
                  ret = 1;      /* Kill object lu loop */
                  break;
                }

              cur_entry++;
            }

          obj_lookup_page++;
        }

      if (ret == 1)
        {
          ret = OK;
        }

      if (ret >= 0 &&
          deleted_pages_in_block + free_pages_in_block ==
          SPIFFS_GEO_PAGES_PER_BLOCK(fs) - SPIFFS_OBJ_LOOKUP_PAGES(fs) &&
          free_pages_in_block <= max_free_pages)
        {
          /* Found a fully deleted block */

          fs->deleted_pages -= deleted_pages_in_block;
          ret = spiffs_gc_erase_block(fs, cur_block);
          return ret;
        }

      cur_entry = 0;
      cur_block++;
      cur_block_addr += SPIFFS_GEO_BLOCK_SIZE(fs);
    }

  if (ret >= 0)
    {
      /* No deleted blocks */

      ret = -ENODATA;
    }

  return ret;
}

/****************************************************************************
 * Name: spiffs_gc_check
 *
 * Description:
 *   Checks if garbage collecting is necessary. If so a candidate block is
 *   found, cleansed and erased
 *
 *   This function will  try to make room for given amount of bytes in the
 *   filesystem by moving pages and erasing blocks.
 *
 *   If it is physically impossible, err_no will be set to -ENOSPC. If
 *   there already is this amount (or more) of free space, SPIFFS_gc will
 *   silently return. It is recommended to call statfs() before invoking
 *   this method in order to determine what amount of bytes to give.
 *
 *   NB: the garbage collector is automatically called when spiffs needs free
 *   pages. The reason for this function is to give possibility to do
 *   background tidying when user knows the system is idle.
 *
 * Input Parameters:
 *   fs            the file system struct
 *   len           amount of data that should be freed
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned on
 *   any failure.  -ENODATA is returned if no blocks were deleted.
 *
 ****************************************************************************/

int spiffs_gc_check(FAR struct spiffs_s *fs, off_t len)
{
  int32_t free_pages;
  uint32_t needed_pages;
  int tries = 0;
  int ret;

  /* Get the number of free pages */

  free_pages = (SPIFFS_GEO_PAGES_PER_BLOCK(fs) -
                SPIFFS_OBJ_LOOKUP_PAGES(fs)) *
                (SPIFFS_GEO_BLOCK_COUNT(fs) - 2) -
                fs->alloc_pages - fs->deleted_pages;

  spiffs_gcinfo("len=%ld free_blocks=%lu free_pages=%ld\n",
                (long)len, (unsigned long)fs->free_blocks,
                (long)free_pages);

  if (fs->free_blocks > 3 &&
      (int32_t)len < free_pages * (int32_t)SPIFFS_DATA_PAGE_SIZE(fs))
    {
      spiffs_gcinfo("Sufficient free space is available  Do nothing.\n");
      return OK;
    }

  /* Get the number of pages needed */

  needed_pages = (len + SPIFFS_DATA_PAGE_SIZE(fs) - 1) /
                 SPIFFS_DATA_PAGE_SIZE(fs);

#if 0
  if (fs->free_blocks <= 2 && (int32_t)needed_pages > free_pages)
    {
      spiffs_gcinfo("Full freeblk=%d needed=%d free=%d dele=%d\n",
                    fs->free_blocks, needed_pages, free_pages,
                    fs->deleted_pages);
      return -ENOSPC;
    }
#endif

  if ((int32_t)needed_pages > (int32_t)(free_pages + fs->deleted_pages))
    {
      spiffs_gcinfo("Full freeblk=%d needed=%d free=%d dele=%d\n",
                    fs->free_blocks, needed_pages, free_pages,
                    fs->deleted_pages);
      return -ENOSPC;
    }

  do
    {
      FAR int16_t *cands;
      int count;
      int16_t cand;
      int32_t prev_free_pages = free_pages;

      spiffs_gcinfo("#%d: run gc free_blocks=%d pfree=%d pallo=%d pdele=%d "
                    "[%d] len=%d of %d\n",
                    tries, fs->free_blocks, free_pages,
                    fs->alloc_pages, fs->deleted_pages,
                    (free_pages + fs->alloc_pages + fs->deleted_pages),
                    len, (uint32_t)(free_pages * SPIFFS_DATA_PAGE_SIZE(fs)));

      /* If the fs is crammed, ignore block age when selecting candidate -
       * kind of a bad state
       */

      ret = spiffs_gc_find_candidate(fs, &cands, &count, free_pages <= 0);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_gc_find_candidate() failed: %d\n", ret);
          return ret;
        }

      if (count == 0)
        {
          spiffs_gcinfo("No candidates, return\n");
          return (int32_t) needed_pages < free_pages ? OK : -ENOSPC;
        }

#ifdef CONFIG_SPIFFS_GCDBG
      fs->stats_gc_runs++;
#endif
      cand = cands[0];

      ret = spiffs_gc_clean(fs, cand);

      spiffs_gcinfo("Cleaning block %d, result=%d\n", cand, ret);

      if (ret < 0)
        {
          ferr("ERROR: spiffs_gc_clean() failed: %d\n", ret);
          return ret;
        }

      ret = spiffs_gc_epage_stats(fs, cand);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_gc_epage_stats() failed: %d\n", ret);
          return ret;
        }

      ret = spiffs_gc_erase_block(fs, cand);
      if (ret < 0)
        {
          ferr("ERROR: spiffs_gc_erase_block() failed: %d\n", ret);
          return ret;
        }

      free_pages = (SPIFFS_GEO_PAGES_PER_BLOCK(fs) -
                    SPIFFS_OBJ_LOOKUP_PAGES(fs)) *
                    (SPIFFS_GEO_BLOCK_COUNT(fs) - 2) -
                    fs->alloc_pages - fs->deleted_pages;

      if (prev_free_pages <= 0 && prev_free_pages == free_pages)
        {
          /* Abort early to reduce wear, at least tried once */

          spiffs_gcinfo("Early abort, no result on gc when fs crammed\n");
          break;
        }
    }
  while (++tries < CONFIG_SPIFFS_GC_MAXRUNS &&
         (fs->free_blocks <= 2 ||
          (int32_t) len > free_pages * (int32_t) SPIFFS_DATA_PAGE_SIZE(fs)));

  /* Re-calculate the number of free pages */

  free_pages = (SPIFFS_GEO_PAGES_PER_BLOCK(fs) -
                SPIFFS_OBJ_LOOKUP_PAGES(fs)) *
                (SPIFFS_GEO_BLOCK_COUNT(fs) - 2) -
                fs->alloc_pages - fs->deleted_pages;

  if ((int32_t) len > free_pages * (int32_t)SPIFFS_DATA_PAGE_SIZE(fs))
    {
      ret = -ENOSPC;
    }

  spiffs_gcinfo("Finished, %d dirty, blocks, %d free, %d pages free, "
                "%d tries, ret=%d\n",
                fs->alloc_pages + fs->deleted_pages,
                fs->free_blocks, free_pages, tries, ret);

  return ret;
}
