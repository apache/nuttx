/****************************************************************************
 * fs/mnemofs/mnemofs_blkalloc.c
 *
 * SPDX-License-Identifier: Apache-2.0 or BSD-3-Clause
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
 * Alternatively, the contents of this file may be used under the terms of
 * the BSD-3-Clause license:
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2024 Saurav Pal
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the author nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 ****************************************************************************/

/* mnemofs block allocator takes some inspiration from littlefs's block
 * allocator.
 *
 * It has two primary jobs...provide a block and ensure wear levelling. The
 * block allocator of mnemofs tries to provide a block that will more or less
 * ensure wear levelling. We'll call the block allocator as BA.
 *
 * The block allocator starts at a random block in the device and starts a
 * circular allocation from there, ie. it allocated sequentially till it
 * reaches the end, at which point it cycles back to the beginning and then
 * continues allocating sequentially. If a page is requested it will check if
 * the page has been written to (being used). If a page is being written to
 * but all the pages in a block are ready to be erased, then the block is
 * erased and page is allocated. If none of these two conditions match, it
 * moves on to check the next page and so on. If the block that contains the
 * page is a bad block, the BA skips all the pages in the entire block.
 *
 * The BA can also grant a request for an entire block. If the BA is
 * currently in the middle of a block, it will skip the remaining pages till
 * it reaches the start of the next block. These pages won't be reflected as
 * being used, and can be allocated the next time the BA cycles back to these
 * pages. Even though skipped pages will be eventually utilized later anyway,
 * block allocation requests are made by very few critical data structures
 * in mnemofs, and they all do it in bulk, and thus skipped pages are
 * minimal.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <math.h>
#include <nuttx/kmalloc.h>
#include <stdbool.h>
#include <stdlib.h>

#include "mnemofs.h"
#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMAP_GET(bmap, idx, off) (((bmap)[(idx)] & (1 << (off))) != 0)
#define BMAP_SET(bmap, idx, off) ((bmap)[(idx)] |= (1 << (off)))
#define DEL_ARR_BLK(sb, blk)     (MFS_BA((sb)).k_del[(blk) * sizeof(size_t)])
#define DEL_ARR_PG(sb, pg)       (DEL_ARR_BLK(sb, MFS_PG2BLK((sb), (pg))))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static inline void pg2bmap(mfs_t pg, FAR mfs_t *idx, FAR uint8_t *off);
static int         is_pg_writeable(FAR struct mfs_sb_s * const sb, mfs_t pg,
                                   FAR mfs_t *idx, FAR uint8_t *off);
static int         is_blk_writeable(FAR struct mfs_sb_s * const sb,
                                    const mfs_t blk);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pg2bmap
 *
 * Description:
 *   Gets the bitmap location of a page. The page in the bitmap will be in
 *   bmap[idx] byte at (1 << off) position in the byte.
 *
 * Input Parameters:
 *   pg  - Page number to check.
 *   idx - Populated later with the index of page in MFS_BA(sb).bmap_upgs
 *   off - Populated later with the offset of page in MFS_BA(sb).bmap_upgs
 *
 * Assumptions/Limitations:
 *   Does not check validity of the index.
 *
 ****************************************************************************/

static inline void pg2bmap(mfs_t pg, FAR mfs_t *idx, FAR uint8_t *off)
{
  /* The compiler should automatically use shift operation for division. */

  *idx = pg / 8;
  *off = pg % 8;
}

/****************************************************************************
 * Name: is_pg_writeable
 *
 * Description:
 *   Checks if a page is writeable by checking if the page is either free, or
 *   it's being used but the entire block is ready for erase.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   pg  - Page number to check.
 *   idx - Populated later with the index of page in MFS_BA(sb).bmap_upgs
 *   off - Populated later with the offset of page in MFS_BA(sb).bmap_upgs
 *
 * Returned Value:
 *   MFS_BLK_BAD      - If the block of the page is a bad block.
 *   MFS_PG_USED      - If the page is being used.
 *   MFS_BLK_ERASABLE - If page can be allocated, but block needs erase.
 *   MFS_PG_FREE      - If the page is free.
 *   -ENOSYS          - Not supported.
 *
 * Assumptions/Limitations:
 *   Assumes this is run in a locked environment.
 *
 ****************************************************************************/

static int is_pg_writeable(FAR struct mfs_sb_s * const sb, mfs_t pg,
                           FAR mfs_t *idx, FAR uint8_t *off)
{
  int blkbad_status;

  /* Bad block check. */

  blkbad_status = mfs_isbadblk(sb, MFS_PG2BLK(sb, pg));
  if (predict_false(blkbad_status == -ENOSYS))
    {
      return blkbad_status;
    }

  if (predict_false(blkbad_status < 0) || blkbad_status == 1)
    {
      return MFS_BLK_BAD;
    }

  pg2bmap(MFS_BA(sb).c_pg, idx, off);

  if (BMAP_GET(MFS_BA(sb).bmap_upgs, *idx, *off))
    {
      if (DEL_ARR_PG(sb, MFS_BA(sb).c_pg) == MFS_PGINBLK(sb))
        {
          return MFS_BLK_ERASABLE;
        }
      else
        {
          return MFS_PG_USED;
        }
    }
  else
    {
      return MFS_PG_FREE;
    }
}

/****************************************************************************
 * Name: is_blk_writeable
 *
 * Description:
 *   Checks if an entire block is allocatable, either because none of the
 *   pages in it have been allocated, or because the entire block can be
 *   erased.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   pg  - Page number to check.
 *   idx - Populated later with the index of page in MFS_BA(sb).bmap_upgs
 *   off - Populated later with the offset of page in MFS_BA(sb).bmap_upgs
 *
 * Returned Value:
 *   MFS_BLK_BAD      - If the block is a bad block.
 *   MFS_BLK_USED     - If the block is being used.
 *   MFS_BLK_ERASABLE - If block can be allocated, but block needs erase.
 *   MFS_BLK_FREE     - If the block is free.
 *
 * Assumptions/Limitations:
 *   Assumes this is run in a locked environment.
 *
 ****************************************************************************/

static int is_blk_writeable(FAR struct mfs_sb_s * const sb, const mfs_t blk)
{
  int     blkbad_status;
  mfs_t   i;
  mfs_t   pg            = MFS_BLK2PG(sb, blk);
  mfs_t   idx;
  uint8_t off;

  /* Bad block check. */

  blkbad_status = mfs_isbadblk(sb, blk);
  if (predict_false(blkbad_status == -ENOSYS))
    {
      return blkbad_status;
    }

  if (predict_false(blkbad_status < 0) || blkbad_status == 1)
    {
      return MFS_BLK_BAD;
    }

  for (i = 0; i < MFS_PGINBLK(sb); i++)
    {
      pg2bmap(pg + i, &idx, &off);

      if (BMAP_GET(MFS_BA(sb).bmap_upgs, idx, off))
        {
          if (DEL_ARR_PG(sb, MFS_BA(sb).c_pg) == MFS_PGINBLK(sb))
            {
              return MFS_BLK_ERASABLE;
            }
          else
            {
              return MFS_BLK_USED;
            }
        }
    }

  return MFS_BLK_FREE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mfs_ba_fmt(FAR struct mfs_sb_s * const sb)
{
  int     ret  = OK;

  /* We need at least 5 blocks, as one is occupied by superblock, at least
   * one for the journal, 2 for journal's master blocks, and at least one for
   * actual data.
   */

  if (MFS_NBLKS(sb) < 5)
    {
      ret = -ENOSPC;
      goto errout;
    }

  memset(&MFS_BA(sb), 0, sizeof(MFS_BA(sb)));

  MFS_BA(sb).s_blk = rand() % MFS_NBLKS(sb);
  if (MFS_PG2BLK(sb, MFS_BA(sb).s_blk) == sb->sb_blk)
    {
      MFS_BA(sb).s_blk++;
      MFS_BA(sb).s_blk %= MFS_NBLKS(sb);
    }

  MFS_BA(sb).c_pg = MFS_BLK2PG(sb, MFS_BA(sb).s_blk);

  /* MFS_BA(sb).k_del_elemsz = ((log + 7) & (-8)) / 8; */

  MFS_BA(sb).k_del = fs_heap_zalloc(sizeof(size_t) * MFS_NBLKS(sb));
  if (predict_false(MFS_BA(sb).k_del == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  MFS_BA(sb).n_bmap_upgs = MFS_UPPER8(MFS_NPGS(sb));

  MFS_BA(sb).bmap_upgs = fs_heap_zalloc(MFS_BA(sb).n_bmap_upgs);
  if (predict_false(MFS_BA(sb).bmap_upgs == NULL))
    {
      ret = -ENOMEM;
      goto errout_with_k_del;
    }

  /* TODO: Do not start from journal blocks. */

  finfo("mnemofs: Block Allocator initialized, starting at page %d.\n",
        MFS_BLK2PG(sb, MFS_BA(sb).s_blk));
  return ret;

errout_with_k_del:
  fs_heap_free(MFS_BA(sb).k_del);

errout:
  return ret;
}

int mfs_ba_init(FAR struct mfs_sb_s * const sb)
{
  /* TODO: Ensure journal and master node are initialized before this. */

  int ret = OK;

  ret = mfs_ba_fmt(sb);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  /* Traverse the FS tree. */

  ret = mfs_pitr_traversefs(sb, MFS_MN(sb).root_ctz, MFS_ISDIR);
  if (predict_false(ret < 0))
    {
      goto errout_with_ba;
    }

  return ret;

errout_with_ba:
  mfs_ba_free(sb);

errout:
  return ret;
}

void mfs_ba_free(FAR struct mfs_sb_s * const sb)
{
  fs_heap_free(MFS_BA(sb).k_del);
  fs_heap_free(MFS_BA(sb).bmap_upgs);

  finfo("Block Allocator Freed.");
}

mfs_t mfs_ba_getpg(FAR struct mfs_sb_s * const sb)
{
  bool    inc   = true;
  bool    found = false;
  mfs_t   i     = MFS_BA(sb).c_pg;
  mfs_t   pg    = 0;
  mfs_t   idx;
  mfs_t   tpgs  = MFS_NBLKS(sb) * MFS_PGINBLK(sb);
  uint8_t off;

  for (; i != tpgs; i++)
    {
      switch (is_pg_writeable(sb, MFS_BA(sb).c_pg, &idx, &off))
        {
          case MFS_PG_USED:
            finfo("Used %d\n", MFS_BA(sb).c_pg);
            break;

          case MFS_PG_FREE:
            finfo("Free %d\n", MFS_BA(sb).c_pg);
            pg = MFS_BA(sb).c_pg;
            mfs_ba_markusedpg(sb, pg);
            found = true;
            break;

          case MFS_BLK_BAD:
            finfo("Bad %d\n", MFS_BA(sb).c_pg);

            /* Skip pages to next block. */

            MFS_BA(sb).c_pg = MFS_BLK2PG(sb,
                                (MFS_PG2BLK(sb, MFS_BA(sb).c_pg) + 1) %
                                MFS_NBLKS(sb));
            inc = false;
            break;

          case MFS_BLK_ERASABLE:
            finfo("Erasable %d\n", MFS_BA(sb).c_pg);
            pg = MFS_BA(sb).c_pg;
            mfs_erase_blk(sb, MFS_PG2BLK(sb, MFS_BA(sb).c_pg));
            DEL_ARR_PG(sb, MFS_BA(sb).c_pg) = 0;
            mfs_ba_markusedpg(sb, pg);
            found = true;
            break;

          case -ENOSYS:

            /* TODO: Manually check for bad blocks. */

            return 0;
        }

      if (inc)
        {
          MFS_BA(sb).c_pg++;
          MFS_BA(sb).c_pg %= tpgs;
        }
      else
        {
          i--;
          inc = true;
        }

      if (found)
        {
          break;
        }
    }

  if (!found)
    {
      DEBUGASSERT(pg == 0);
      finfo("No more pages found. Page: %u.", pg);
    }

  return pg;
}

mfs_t mfs_ba_getblk(FAR struct mfs_sb_s * const sb)
{
  bool  found = false;
  mfs_t i     = 0;
  mfs_t blk;
  mfs_t ret   = 0;

  blk = MFS_PG2BLK(sb, MFS_BA(sb).c_pg);
  if (MFS_BA(sb).c_pg % MFS_PGINBLK(sb))
    {
      /* Skipped pages are not updated in used. */

      blk++;
      blk %= MFS_NBLKS(sb);
      i++;
    }

  for (; i < MFS_NBLKS(sb); i++)
    {
      switch (is_blk_writeable(sb, blk))
        {
          case MFS_BLK_BAD:
            break;

          case MFS_BLK_USED:
            break;

          case MFS_BLK_ERASABLE:
            mfs_ba_blkmarkdel(sb, blk);
            mfs_ba_markusedblk(sb, blk);
            found = true;
            break;

          case MFS_BLK_FREE:
            mfs_ba_markusedblk(sb, blk);
            found = true;
            break;

          case -ENOSYS:

            /* TODO: Manually check for bad blocks. */

            return 0;
        }

      if (found)
        {
          break;
        }

      blk++;
      blk %= MFS_NBLKS(sb);
    }

  if (found)
    {
      ret = blk;
      MFS_BA(sb).c_pg = MFS_BLK2PG(sb, (++blk) % MFS_NBLKS(sb));
    }

  finfo("Block number: %u. Found: %d.", ret, found);

  return ret;
}

void mfs_ba_pgmarkdel(FAR struct mfs_sb_s * const sb, mfs_t pg)
{
  DEL_ARR_PG(sb, MFS_BA(sb).c_pg)++;
}

void mfs_ba_blkmarkdel(FAR struct mfs_sb_s * const sb, mfs_t blk)
{
  DEL_ARR_BLK(sb, blk) = MFS_PGINBLK(sb);
}

int mfs_ba_delmarked(FAR struct mfs_sb_s * const sb)
{
  int   ret = OK;
  mfs_t i;

  for (i = 1; i < MFS_NBLKS(sb); i++)
    {
      if (DEL_ARR_BLK(sb, i) == MFS_PGINBLK(sb))
        {
          ret = mfs_erase_blk(sb, i);

          if (ret != OK)
            {
              return ret;
            }
        }
    }

  return ret;
}

/* Mark a page as being used. Used by master node during initial format and
 */

void mfs_ba_markusedpg(FAR struct mfs_sb_s * const sb, mfs_t pg)
{
  mfs_t   idx;
  uint8_t off;

  pg2bmap(pg, &idx, &off);
  BMAP_SET(MFS_BA(sb).bmap_upgs, idx, off); /* Set as used */
}

void mfs_ba_markusedblk(FAR struct mfs_sb_s * const sb, mfs_t blk)
{
  mfs_t i  = 0;
  mfs_t pg = MFS_BLK2PG(sb, blk);

  for (i = 0; i < MFS_PGINBLK(sb); i++)
    {
      mfs_ba_markusedpg(sb, pg + i);
    }
}

mfs_t mfs_ba_getavailpgs(FAR const struct mfs_sb_s * const sb)
{
  /* TODO */

  return 0;
}
