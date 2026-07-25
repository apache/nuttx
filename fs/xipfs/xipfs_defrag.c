/****************************************************************************
 * fs/xipfs/xipfs_defrag.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
 *
 * Because files are written once at a known size and never grow, a file can
 * never fragment internally.  The only thing that fragments is free space,
 * via the holes deletes leave behind.  So this is not a continuous
 * compactor: it is an occasional coalescing pass, invoked explicitly.
 *
 * It is structured as a loop of atomic relocations.  Each relocation copies
 * one closed, unpinned extent into free space, commits a new metadata
 * generation naming the new location, and only then erases the blocks it
 * vacated.  Every iteration boundary is therefore a fully consistent
 * layout, merely a less compact one, and every way of stopping -- a time
 * budget, a pinned extent in the way, a media error, a power cut -- lands
 * on one of those boundaries.  Nothing is ever left half moved.
 *
 * The pass is best effort by design.  It reports what it achieved and why
 * it stopped rather than succeeding or failing, because the caller asked
 * for it after an allocation did not fit and what it actually needs to know
 * is whether retrying that allocation can now work.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <nuttx/clock.h>

#include "xipfs.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_defrag_findlowestfree
 *
 * Description:
 *   Find the lowest addressed free run in the data region.
 *
 * Returned Value:
 *   true if a free run exists, with its start and length reported.
 *
 ****************************************************************************/

static bool xipfs_defrag_findlowestfree(FAR struct xipfs_mount_s *fs,
                                        FAR uint32_t *start,
                                        FAR uint32_t *len)
{
  FAR struct xipfs_extent_s *ext;
  uint32_t block;
  uint32_t run;
  bool used;

  for (block = fs->data_start;
       block < fs->data_start + fs->data_nblocks;
       block++)
    {
      used = false;

      for (ext = fs->extents; ext != NULL; ext = ext->flink)
        {
          if (block >= ext->start_block &&
              block < ext->start_block + ext->nblocks)
            {
              used = true;
              break;
            }
        }

      if (used)
        {
          continue;
        }

      /* Measure the run starting here */

      run = 0;
      while (block + run < fs->data_start + fs->data_nblocks)
        {
          bool inuse = false;

          for (ext = fs->extents; ext != NULL; ext = ext->flink)
            {
              if (block + run >= ext->start_block &&
                  block + run < ext->start_block + ext->nblocks)
                {
                  inuse = true;
                  break;
                }
            }

          if (inuse)
            {
              break;
            }

          run++;
        }

      *start = block;
      *len   = run;
      return true;
    }

  return false;
}

/****************************************************************************
 * Name: xipfs_defrag_pickvictim
 *
 * Description:
 *   Choose the unpinned extent that starts immediately above the given free
 *   run and fits inside it.  Moving that one down is what merges the hole
 *   with whatever free space follows the extent.
 *
 * Returned Value:
 *   The extent to relocate, or NULL if none qualifies.  'blocked' is set
 *   when the only candidates were rejected because they are pinned, which
 *   is the difference between "nothing left to do" and "cannot proceed
 *   until something is unloaded".
 *
 ****************************************************************************/

static FAR struct xipfs_extent_s *
xipfs_defrag_pickvictim(FAR struct xipfs_mount_s *fs, uint32_t free_start,
                        uint32_t free_len, FAR bool *pin_blocked,
                        FAR bool *open_blocked,
                        FAR uint32_t *pinned_blocks)
{
  FAR struct xipfs_extent_s *best = NULL;
  FAR struct xipfs_extent_s *ext;

  *pin_blocked  = false;
  *open_blocked = false;

  for (ext = fs->extents; ext != NULL; ext = ext->flink)
    {
      if (ext->isdir)
        {
          /* A directory is a metadata record with no blocks to move */

          continue;
        }

      if (ext->start_block <= free_start)
        {
          continue;
        }

      if (ext->nblocks > free_len)
        {
          continue;
        }

      if (ext->pincount > 0)
        {
          /* Pinned extents sitting mid-flash split the free space and bound
           * how much can ever be reclaimed.  That is inherent to executing
           * in place, not a defect: the module is running from those very
           * blocks.
           */

          *pin_blocked = true;
          *pinned_blocks += ext->nblocks;
          continue;
        }

      if (ext->writing || ext->openrefs > 0)
        {
          /* Merely open, not mapped.  Still not movable -- a reader holds a
           * file position against it -- but the caller resolves this by
           * closing a descriptor rather than by unloading a module, so it
           * is reported separately.
           */

          *open_blocked = true;
          continue;
        }

      /* Prefer the extent closest to the hole */

      if (best == NULL || ext->start_block < best->start_block)
        {
          best = ext;
        }
    }

  return best;
}

/****************************************************************************
 * Name: xipfs_defrag_relocate
 *
 * Description:
 *   Perform one atomic relocation of 'ext' to 'dest'.
 *
 *   The order is load bearing: copy the data in full, commit the metadata
 *   that names the new location, and only then erase the old blocks.  A
 *   power loss before the commit leaves the old generation live and the old
 *   data intact, so the copy is simply forgotten.  A power loss after it
 *   leaves the new location live and the old blocks merely stale, which the
 *   next mount reclaims because nothing references them.
 *
 ****************************************************************************/

static int xipfs_defrag_relocate(FAR struct xipfs_mount_s *fs,
                                 FAR struct xipfs_extent_s *ext,
                                 uint32_t dest)
{
  uint32_t old_start = ext->start_block;
  uint32_t nblocks = ext->nblocks;
  uint32_t pages_per_block;
  uint32_t block;
  uint32_t page;
  int ret;

  pages_per_block = fs->geo.erasesize / fs->geo.blocksize;

  /* The destination must already be erased.  Erasing it here rather than
   * assuming keeps the invariant local and cheap: it is free space, so
   * nothing can be lost by erasing it.
   */

  ret = xipfs_flash_erase(fs, dest, nblocks);
  if (ret < 0)
    {
      return ret;
    }

  /* Copy a page at a time.  Block granular allocation means no source block
   * ever holds a mix of live and dead data, so the staging buffer is one
   * program page rather than a whole erase block.
   */

  for (block = 0; block < nblocks; block++)
    {
      for (page = 0; page < pages_per_block; page++)
        {
          uint32_t offset = page * fs->geo.blocksize;

          ret = xipfs_flash_read(fs, old_start + block, offset, fs->stage,
                                 fs->geo.blocksize);
          if (ret < 0)
            {
              return ret;
            }

          ret = xipfs_flash_write(fs, dest + block, offset, fs->stage,
                                  fs->geo.blocksize);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  /* Commit point.  Until this succeeds the extent still officially lives at
   * its old location and everything written above is invisible.
   */

  ext->start_block = dest;

  ret = xipfs_meta_commit(fs);
  if (ret < 0)
    {
      ext->start_block = old_start;
      return ret;
    }

  /* The move is now durable.  Erasing the vacated blocks is pure hygiene:
   * if it fails or is interrupted, the layout is still correct and the
   * blocks are simply unreferenced until something reuses them.
   */

  xipfs_alloc_rebuild(fs);
  xipfs_flash_erase(fs, old_start, nblocks);

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_defrag
 *
 * Description:
 *   Run one best-effort compaction pass.  Called with the mount unlocked.
 *
 * Returned Value:
 *   OK on any clean stop, including one that made no progress at all.  A
 *   negative errno only for a failure that prevented the pass from running.
 *
 ****************************************************************************/

int xipfs_defrag(FAR struct xipfs_mount_s *fs, uint32_t max_ms,
                 FAR struct xipfs_defrag_result_s *result)
{
  FAR struct xipfs_extent_s *victim;
  clock_t start_tick;
  uint32_t free_start;
  uint32_t free_len;
  uint32_t pinned_blocks;
  bool pin_blocked;
  bool open_blocked;
  int ret;

  memset(result, 0, sizeof(*result));
  result->reason = XIPFS_DEFRAG_DONE;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  start_tick = clock_systime_ticks();

  for (; ; )
    {
      /* Check the caller's budget between moves, never during one */

      if (max_ms > 0 &&
          TICK2MSEC(clock_systime_ticks() - start_tick) >= max_ms)
        {
          result->reason = XIPFS_DEFRAG_TIME_BUDGET;
          break;
        }

      if (!xipfs_defrag_findlowestfree(fs, &free_start, &free_len))
        {
          /* No free space at all; nothing to compact into */

          break;
        }

      pinned_blocks = 0;
      victim = xipfs_defrag_pickvictim(fs, free_start, free_len,
                                       &pin_blocked, &open_blocked,
                                       &pinned_blocks);
      if (victim == NULL)
        {
          /* A live mapping is the more serious obstruction, so it wins the
           * report when both are present.
           */

          if (pin_blocked)
            {
              result->reason        = XIPFS_DEFRAG_BLOCKED_PINS;
              result->blocks_pinned = pinned_blocks;
            }
          else if (open_blocked)
            {
              result->reason = XIPFS_DEFRAG_BLOCKED_OPEN;
            }

          break;
        }

      ret = xipfs_defrag_relocate(fs, victim, free_start);
      if (ret < 0)
        {
          /* Stopped at a valid intermediate layout.  Nothing is partially
           * applied, so this is a clean stop rather than an error the
           * caller has to recover from.
           */

          result->reason = (ret == -ENOMEM) ? XIPFS_DEFRAG_BLOCKED_RAM
                                            : XIPFS_DEFRAG_ERROR;
          break;
        }

      result->extents_moved++;
      result->blocks_reclaimed += victim->nblocks;
    }

  result->largest_free_run = (size_t)xipfs_alloc_largestrun(fs) *
                             fs->geo.erasesize;

  finfo("xipfs: defrag moved %" PRIu32 " extents, largest run %zu, "
        "reason %d\n", result->extents_moved, result->largest_free_run,
        result->reason);

  xipfs_unlock(fs);
  return OK;
}

/****************************************************************************
 * Name: xipfs_listpinned
 *
 * Description:
 *   Report the extents currently holding XIP pins.  Without this, a caller
 *   told that defrag is blocked by pins has no way to find out which module
 *   to unload, and the report is a dead end.
 *
 ****************************************************************************/

int xipfs_listpinned(FAR struct xipfs_mount_s *fs,
                     FAR struct xipfs_pinned_entry_s *entries,
                     size_t nentries, FAR size_t *count)
{
  FAR struct xipfs_extent_s *ext;
  size_t n = 0;
  int ret;

  ret = xipfs_lock(fs);
  if (ret < 0)
    {
      return ret;
    }

  for (ext = fs->extents; ext != NULL; ext = ext->flink)
    {
      if (ext->pincount == 0)
        {
          continue;
        }

      if (n < nentries)
        {
          xipfs_meta_path(fs, ext, entries[n].path,
                          sizeof(entries[n].path));
          entries[n].start_block = ext->start_block;
          entries[n].nblocks     = ext->nblocks;
          entries[n].pincount    = ext->pincount;
        }

      n++;
    }

  /* Report the true total even when the caller's array was too small, so a
   * short buffer is detectable rather than silently truncating.
   */

  *count = n;

  xipfs_unlock(fs);
  return n > nentries ? -ENOSPC : OK;
}
