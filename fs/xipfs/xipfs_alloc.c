/****************************************************************************
 * fs/xipfs/xipfs_alloc.c
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
 * Allocation is in whole erase blocks and always contiguous.  Because a
 * file's full size is known when it is created, the exact extent is
 * reserved up front and never grows, so a file can never become internally
 * fragmented.  The only source of fragmentation is the holes left behind by
 * deletes, which is what the defragmenter coalesces.
 *
 * The free map is a bitmap over the data region, derived entirely from the
 * committed directory: it is state that can always be rebuilt, never state
 * that has to be recovered.
 *
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>

#include <nuttx/bits.h>
#include <nuttx/kmalloc.h>

#include "xipfs.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xipfs_alloc_init
 *
 * Description:
 *   Allocate the free bitmap.  Sized for the data region only.
 *
 ****************************************************************************/

int xipfs_alloc_init(FAR struct xipfs_mount_s *fs)
{
  fs->bitmapsize = BITS_TO_LONGS(fs->data_nblocks) * sizeof(unsigned long);
  fs->bitmap     = kmm_zalloc(fs->bitmapsize);

  if (fs->bitmap == NULL)
    {
      return -ENOMEM;
    }

  return OK;
}

/****************************************************************************
 * Name: xipfs_alloc_rebuild
 *
 * Description:
 *   Recompute the free bitmap from the extent list.  Called after mount and
 *   after any change that moves extents.
 *
 ****************************************************************************/

void xipfs_alloc_rebuild(FAR struct xipfs_mount_s *fs)
{
  FAR struct xipfs_extent_s *ext;
  uint32_t index;

  memset(fs->bitmap, 0, fs->bitmapsize);

  for (ext = fs->extents; ext != NULL; ext = ext->flink)
    {
      /* A directory record owns no blocks, and its start_block lies below
       * the data region, so the index must not be formed for one.
       */

      if (ext->nblocks == 0)
        {
          continue;
        }

      index = ext->start_block - fs->data_start;

      DEBUGASSERT(index + ext->nblocks <= fs->data_nblocks);
      bitmap_set(fs->bitmap, index, ext->nblocks);
    }
}

/****************************************************************************
 * Name: xipfs_alloc
 *
 * Description:
 *   Reserve a contiguous run of 'nblocks' erase blocks using best fit.
 *
 *   This deliberately does NOT invoke the defragmenter on failure.  A
 *   caller that hits -ENOSPC gets to decide whether compacting is worth it
 *   right now, and the defragmenter therefore always runs from a known
 *   clean state rather than from inside a half-finished allocation.
 *
 * Returned Value:
 *   OK with *start_block set, or -ENOSPC if no single contiguous run of
 *   the requested size exists.
 *
 ****************************************************************************/

int xipfs_alloc(FAR struct xipfs_mount_s *fs, uint32_t nblocks,
                FAR uint32_t *start_block)
{
  uint32_t best_start = 0;
  uint32_t best_len = 0;
  uint32_t run_start = 0;
  uint32_t run_len = 0;
  uint32_t i;
  int ret;

  if (nblocks == 0)
    {
      return -EINVAL;
    }

  for (i = 0; i <= fs->data_nblocks; i++)
    {
      bool used = (i == fs->data_nblocks) || test_bit(i, fs->bitmap) != 0;

      if (!used)
        {
          if (run_len == 0)
            {
              run_start = i;
            }

          run_len++;
          continue;
        }

      /* End of a free run.  Keep it if it is the tightest fit so far. */

      if (run_len >= nblocks && (best_len == 0 || run_len < best_len))
        {
          best_len   = run_len;
          best_start = run_start;
        }

      run_len = 0;
    }

  if (best_len == 0)
    {
      return -ENOSPC;
    }

  /* The scan already proved this run free under the mount lock, so -EBUSY
   * is not a full volume: the bitmap and the extent list disagree.
   */

  ret = bitmap_allocate_region(fs->bitmap, best_start, nblocks);
  if (ret < 0)
    {
      ferr("ERROR: Free run %" PRIu32 "+%" PRIu32 " is already allocated\n",
           best_start, nblocks);
      return ret;
    }

  *start_block = fs->data_start + best_start;
  return OK;
}

/****************************************************************************
 * Name: xipfs_free
 *
 * Description:
 *   Release a previously allocated run.  Coalescing is implicit: the run
 *   simply becomes part of whatever free neighbours surround it.
 *
 ****************************************************************************/

void xipfs_free(FAR struct xipfs_mount_s *fs, uint32_t start_block,
                uint32_t nblocks)
{
  /* Freeing nothing is a no-op.  This is not a corner case to tolerate but
   * the correct answer for an extent whose reservation never landed: a
   * create cut short before xipfs_reserve succeeds leaves start_block at 0,
   * which is below the data region, and the cleanup path still hands that
   * extent here.  A live extent always has nblocks > 0, so guarding on the
   * length is exactly equivalent to guarding on "was this ever reserved".
   */

  if (nblocks == 0)
    {
      return;
    }

  DEBUGASSERT(start_block >= fs->data_start);
  DEBUGASSERT(start_block + nblocks <= fs->data_start + fs->data_nblocks);

  bitmap_clear(fs->bitmap, start_block - fs->data_start, nblocks);
}

/****************************************************************************
 * Name: xipfs_alloc_largestrun
 *
 * Description:
 *   Length in blocks of the largest contiguous free run.  This is what
 *   tells a caller whose allocation just failed whether a retry after
 *   defragmenting can possibly succeed.
 *
 ****************************************************************************/

uint32_t xipfs_alloc_largestrun(FAR struct xipfs_mount_s *fs)
{
  uint32_t best = 0;
  uint32_t run = 0;
  uint32_t i;

  for (i = 0; i < fs->data_nblocks; i++)
    {
      if (test_bit(i, fs->bitmap))
        {
          run = 0;
          continue;
        }

      run++;
      if (run > best)
        {
          best = run;
        }
    }

  return best;
}

/****************************************************************************
 * Name: xipfs_alloc_freeblocks
 ****************************************************************************/

uint32_t xipfs_alloc_freeblocks(FAR struct xipfs_mount_s *fs)
{
  uint32_t count = 0;
  uint32_t i;

  for (i = 0; i < fs->data_nblocks; i++)
    {
      if (!test_bit(i, fs->bitmap))
        {
          count++;
        }
    }

  return count;
}
