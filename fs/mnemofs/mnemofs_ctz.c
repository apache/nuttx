/****************************************************************************
 * fs/mnemofs/mnemofs_ctz.c
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

/****************************************************************************
 * In mnemofs, the files and directories use the CTZ skip list data structure
 * defined by littlefs. These are reverse skip lists with a specific number
 * of pointers for each block. The number of pointers for a block at index
 * `x` is `ctz(x) + 1`. There are no pointers if the index is 0.
 *
 * The pointers all point to some CTZ block other than the CTZ block they are
 * part of. The `k`th pointer of a CTZ block at index `x` points to the
 * CTZ block at index `x - 2^k`.
 *
 * For example, CTZ block at index 2 has 2 pointers, and they point to the
 * block at index 1, and index 0 respectively.
 *
 *                                                     File/Dir Ptr
 *                                                            |
 *                                                            V
 * +------+   +------+   +------+   +------+   +------+   +------+
 * |      |<--|      |---|      |---|      |---|      |   |      |
 * | Node |<--| Node |---| Node |<--| Node |---| Node |   | Node |
 * |  0   |<--|  1   |<--|  2   |<--|  3   |<--|  4   |<--|  5   |
 * +------+   +------+   +------+   +------+   +------+   +------+
 *
 * In mnemofs, each CTZ block is stored in a page on the flash. All code in
 * this entire file will call CTZ blocks as blocks to honour the original
 * naming, and will specify wherever it deviates from this assumption.
 *
 * Littlefs's design documentation lists all the benefits that this data
 * structure brings to the table when it comes to storing large pieces of
 * data that will be modified considerably frequently, while being in a
 * Copy On Write (CoW) environment.
 *
 * In mnemofs, the CTZ methods only interface with the underlying R/W methods
 * , journal on the lower side and on the upper side, the LRU, and ensures
 * that whatever data it provides considers both the on-flash data, as well
 * the journal logs.
 *
 * The pointers are stored such that the first pointer, which points to
 * (x - 2^0), is stored at the very end of the CTZ block. The second pointer
 * is stored second last, and so on.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>
#include <fcntl.h>
#include <nuttx/kmalloc.h>
#include <math.h>
#include <sys/param.h>
#include <sys/stat.h>

#include "mnemofs.h"
#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MFS_CTZ_PTRSZ (sizeof(mfs_t))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static mfs_t  ctz_idx_nptrs(const mfs_t idx);
static void   ctz_off2loc(FAR const struct mfs_sb_s * const sb, mfs_t off,
                          FAR mfs_t *idx, FAR mfs_t *pgoff);
static mfs_t  ctz_blkdatasz(FAR const struct mfs_sb_s * const sb,
                            const mfs_t idx);
static void   ctz_copyidxptrs(FAR const struct mfs_sb_s * const sb,
                              struct mfs_ctz_s ctz, const mfs_t idx,
                              FAR char *buf);

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
 * Name: ctz_idx_nptrs
 *
 * Description:
 *   Gives the numbers of pointers that a CTZ block of given index should
 *   have.
 *
 * Input Parameters:
 *   idx - Index of the ctz block.
 *
 * Returned Value:
 *   The number of pointers in the CTZ block.
 *
 ****************************************************************************/

static mfs_t ctz_idx_nptrs(const mfs_t idx)
{
  mfs_t ret;

  ret = (idx == 0) ? 0 : mfs_ctz(idx) + 1;
  finfo("Number of pointers for %u index is %u.", idx, ret);
  return ret;
}

/****************************************************************************
 * Name: ctz_off2loc
 *
 * Description:
 *   Converts ctz offset (which is the offset of the data stored in the ctz
 *   list, which is unaware of the presence of pointers) into the CTZ
 *   block index and the offset in that CTZ block.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   off   - Offset of the data stored in the CTZ list.
 *   idx   - Indes of the CTZ block, to be populated.
 *   pgoff - Offset inside the CTZ block, to be populated.
 *
 ****************************************************************************/

static void ctz_off2loc(FAR const struct mfs_sb_s * const sb, mfs_t off,
                        FAR mfs_t *idx, FAR mfs_t *pgoff)
{
  const mfs_t wb  = sizeof(mfs_t);
  const mfs_t den = MFS_PGSZ(sb) - 2 * wb;

  if (off < den)
    {
      *idx = 0;
      *pgoff = off;
      return;
    }

  if (idx != NULL)
    {
      *idx   = (off - wb * (__builtin_popcount((off / den) - 1) + 2)) / den;
    }

  if (pgoff != NULL)
    {
      *pgoff = off - den * (*idx) - wb * __builtin_popcount(*idx)
              - (ctz_idx_nptrs(*idx) * wb);
    }

  finfo("Offset %u. Calculated index %u and page offset %u.", off, *idx,
        *pgoff);
}

/****************************************************************************
 * Name: ctz_blkdatasz
 *
 * Description:
 *   The size of data in B that can be fit inside a CTZ block at index `idx`.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   idx - Index of the ctz block.
 *
 * Returned Value:
 *   The size of data in the CTZ block.
 *
 ****************************************************************************/

static mfs_t ctz_blkdatasz(FAR const struct mfs_sb_s * const sb,
                           const mfs_t idx)
{
  mfs_t ret;

  ret = MFS_PGSZ(sb) - (ctz_idx_nptrs(idx) * MFS_LOGPGSZ(sb));
  finfo("Block data size for index %u is %u.", idx, ret);
  return ret;
}

/****************************************************************************
 * Name: ctz_copyidxptrs
 *
 * Description:
 *   This is used for cases when you want to expand a CTZ list from any point
 *   in the list. If we want to expand the CTZ list from a particular index,
 *   say `start_idx`, while keeping all indexes before it untouched, we
 *   would need to first allocate new blocks on the flash, and then copy
 *   the pointers to the location.
 *
 *   Usage of this function is, the caller needs to first allocate a CTZ
 *   block (a page on flash), allocate buffer which is the size of a CTZ
 *   block (a page on flash), and use this method to copy the pointers to the
 *   buffer, then write the data to the flash.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   ctz - CTZ list to use as a reference.
 *   idx - Index of the block who's supposed pointers are to be copied.
 *   buf - Buffer representing the entire CTZ block where pointers are
 *         copied to.
 *
 * Assumptions/Limitations:
 *   This assumes `idx` is not more than `ctz->idx_e + 1`.
 *
 ****************************************************************************/

static void ctz_copyidxptrs(FAR const struct mfs_sb_s * const sb,
                            struct mfs_ctz_s ctz, const mfs_t idx,
                            FAR char *buf)
{
  mfs_t i;
  mfs_t n_ptrs;
  mfs_t prev_pg;
  mfs_t prev_idx;

  if (idx == 0)
    {
      /* No pointers for first block. */

      return;
    }

  n_ptrs = ctz_idx_nptrs(idx);

  if (idx != ctz.idx_e + 1)
    {
      /* We travel to the second last "known" CTZ block. */

      ctz.pg_e  = mfs_ctz_travel(sb, ctz.idx_e, ctz.pg_e, idx - 1);
      ctz.idx_e = idx - 1;
    }

  buf += MFS_PGSZ(sb); /* Go to buf + pg_sz */

  DEBUGASSERT(idx == ctz.idx_e + 1);

  finfo("Copying %u pointers for CTZ (%u, %u) at index %u.", n_ptrs,
        ctz.idx_e, ctz.pg_e, idx);

  for (i = 0; i < n_ptrs; i++)
    {
      if (predict_false(i == 0))
        {
          prev_idx = ctz.idx_e;
          prev_pg  = ctz.pg_e;
        }
      else
        {
          prev_pg  = mfs_ctz_travel(sb, prev_idx, prev_pg, prev_idx - 1);
          prev_idx--;
        }

      ctz.idx_e = prev_idx;

      /* Do buf + pg_sz - (idx * sizeof(mfs_t)) iteratively. */

      buf -= MFS_CTZ_PTRSZ;
      mfs_ser_mfs(prev_pg, buf);

      finfo("Copied %u page number to %uth pointer.", prev_pg, i);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mfs_ctz_rdfromoff(FAR const struct mfs_sb_s * const sb,
                      const struct mfs_ctz_s ctz, mfs_t data_off,
                      mfs_t len, FAR char * buf)
{
  int   ret       = OK;
  mfs_t i;
  mfs_t cur_pg;
  mfs_t cur_idx;
  mfs_t cur_pgoff;
  mfs_t end_idx;
  mfs_t end_pgoff;
  mfs_t pg_rd_sz;

  finfo("Reading (%u, %u) CTZ from %u offset for %u bytes.", ctz.idx_e,
        ctz.pg_e, data_off, len);

  if (ctz.idx_e == 0 && ctz.pg_e == 0)
    {
      goto errout;
    }

  ctz_off2loc(sb, data_off + len, &cur_idx, &cur_pgoff);
  ctz_off2loc(sb, data_off, &end_idx, &end_pgoff);

  DEBUGASSERT(ctz.idx_e < cur_idx); /* TODO: Need to consider this. For now, there is a temporary fix in read(). */
  if (ctz.idx_e < end_idx)
    {
      goto errout;
    }

  cur_pg   = mfs_ctz_travel(sb, ctz.idx_e, ctz.pg_e, cur_idx);

  if (predict_false(cur_pg == 0))
    {
      goto errout;
    }

  /* O(n) read by reading in reverse. */

  finfo("Started reading. Current Idx: %u, End Idx: %u.", cur_idx, end_idx);

  if (cur_idx != end_idx)
    {
      for (i = cur_idx; i >= end_idx; i--)
        {
          finfo("Current index %u, Current Page %u.", i, cur_pg);

          if (predict_false(i == cur_idx))
            {
              pg_rd_sz  = cur_pgoff;
              ret       = mfs_read_page(sb, buf - pg_rd_sz, pg_rd_sz, cur_pg,
                                        0);
              cur_pgoff = 0;
            }
          else if (predict_false(i == end_idx))
            {
              pg_rd_sz = ctz_blkdatasz(sb, i) - end_pgoff;
              ret      = mfs_read_page(sb, buf - pg_rd_sz, pg_rd_sz, cur_pg,
                                       end_pgoff);
            }
          else
            {
              pg_rd_sz = ctz_blkdatasz(sb, i);
              ret      = mfs_read_page(sb, buf - pg_rd_sz, pg_rd_sz, cur_pg,
                                      0);
            }

          if (predict_false(ret == 0))
            {
              ret = -EINVAL;
              goto errout;
            }

          buf -= pg_rd_sz;
        }

      cur_pg = mfs_ctz_travel(sb, cur_idx, cur_pg, cur_idx - 1);
      if (predict_false(cur_pg == 0))
        {
          ret = -EINVAL;
          goto errout;
        }
    }
  else
    {
      ret = mfs_read_page(sb, buf, len, cur_pg, end_pgoff);
      if (predict_false(ret < 0))
        {
          goto errout;
        }

      ret = OK;
    }

  finfo("Reading finished.");

errout:
  return ret;
}

int mfs_ctz_wrtnode(FAR struct mfs_sb_s * const sb,
                    FAR const struct mfs_node_s * const node,
                    FAR struct mfs_ctz_s *new_loc)
{
  int                    ret        = OK;
  bool                   written    = false;
  mfs_t                  prev;
  mfs_t                  rem_sz;
  mfs_t                  new_pg;
  mfs_t                  cur_pg;
  mfs_t                  cur_idx;
  mfs_t                  cur_pgoff;
  mfs_t                  lower;
  mfs_t                  upper;
  mfs_t                  upper_og;
  mfs_t                  lower_upd;
  mfs_t                  upper_upd;
  mfs_t                  del_bytes;
  FAR char               *buf       = NULL;
  FAR char               *tmp       = NULL;
  struct mfs_ctz_s       ctz;
  FAR struct mfs_delta_s *delta;

  finfo("Write LRU node %p at depth %u.", node, node->depth);

  /* Traverse common CTZ blocks. */

  ctz_off2loc(sb, node->range_min, &cur_idx, &cur_pgoff);
  ctz    = node->path[node->depth - 1].ctz;
  cur_pg = mfs_ctz_travel(sb, ctz.idx_e, ctz.pg_e, cur_idx);

  /* So, till cur_idx - 1, the CTZ blocks are common. */

  buf = fs_heap_zalloc(MFS_PGSZ(sb));
  if (predict_false(buf == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Initially, there might be some offset in cur_idx CTZ blocks that is
   * unmodified as well.
   */

  finfo("Initial read.");
  tmp = buf;
  mfs_read_page(sb, tmp, cur_pgoff, cur_pg, 0);
  tmp += cur_pgoff;

  /* Modifications. */

  prev      = 0;
  rem_sz    = node->sz;
  lower     = node->range_min;
  del_bytes = 0;

  /* [lower, upper) range. Two pointer approach. Window gets narrower
   * for every delete falling inside it.
   */

  while (rem_sz > 0)
    {
      upper    = MIN(prev + lower + ctz_blkdatasz(sb, cur_idx), rem_sz);
      upper_og = upper;

      finfo("Remaining Size %" PRIu32 ". Lower %" PRIu32 ", Upper %" PRIu32
            ", Current Offset %zd.", rem_sz, lower, upper, tmp - buf);

      /* Retrieving original data. */

      ret = mfs_ctz_rdfromoff(sb, ctz, lower + del_bytes, upper - lower,
                              tmp);
      if (predict_false(ret < 0))
        {
          goto errout_with_buf;
        }

      list_for_every_entry(&node->delta, delta, struct mfs_delta_s, list)
        {
          finfo("Checking delta %p in node %p. Offset %" PRIu32 ", bytes %"
                PRIu32, delta, node, delta->off, delta->n_b);

          lower_upd = MAX(lower, delta->off);
          upper_upd = MIN(upper, delta->off + delta->n_b);

          if (lower_upd >= upper_upd)
            {
              /* Skip this delta. */

              continue;
            }

          if (delta->upd == NULL)
            {
              finfo("Node type: Delete");

              /* Delete */

              del_bytes += upper_upd - lower_upd;
              memmove(tmp + (lower_upd - lower), tmp + (upper_upd - lower),
                      upper - upper_upd);
              upper -= upper_upd;
            }
          else
            {
              finfo("Node type: Update");

              /* Update */

              memcpy(tmp + (lower_upd - lower),
                     delta->upd + (lower_upd - delta->off),
                     upper_upd - lower_upd);
            }
        }

      /* rem_sz check for final write. */

      if (upper == upper_og || rem_sz == upper - lower)
        {
          prev = 0;

          /* Time to write a page for new CTZ list. */

          new_pg = mfs_ba_getpg(sb);
          if (predict_false(new_pg == 0))
            {
              ret = -ENOSPC;
              goto errout_with_buf;
            }

          ctz_copyidxptrs(sb, ctz, cur_idx, buf);

          ret = mfs_write_page(sb, buf, MFS_PGSZ(sb), new_pg, 0);
          if (predict_false(ret == 0))
            {
              ret = -EINVAL;
              goto errout_with_buf;
            }

          memset(buf, 0, MFS_PGSZ(sb));
          tmp = buf;
          ctz.idx_e = cur_idx;
          ctz.pg_e  = new_pg;
          cur_idx++;

          written = true;

          finfo("Written data to page %" PRIu32, new_pg);
        }
      else
        {
          tmp += upper - lower;

          written = false;
        }

      prev    = upper - lower;
      rem_sz -= upper - lower;
      lower   = upper;
    }

  DEBUGASSERT(written);

  /* TODO: Need to verify for cases where the delete extends outside, etc. */

  /* Write log. Assumes journal has enough space due to the limit. */

  finfo("Writing log.");
  *new_loc = ctz;
  ret = mfs_jrnl_wrlog(sb, node, ctz, node->sz);
  if (predict_false(ret < 0))
    {
      goto errout_with_buf;
    }

errout_with_buf:
  fs_heap_free(buf);

errout:
  return ret;
}

mfs_t mfs_ctz_travel(FAR const struct mfs_sb_s * const sb,
                     mfs_t idx_src, mfs_t pg_src, mfs_t idx_dest)
{
  char  buf[4];
  mfs_t pg;
  mfs_t idx;
  mfs_t pow;
  mfs_t diff;
  mfs_t max_pow;

  /* Rising phase. */

  max_pow = (sizeof(mfs_t) * 8) - mfs_clz(idx_src ^ idx_dest);
  idx     = idx_src;
  pow     = 1;
  pg      = pg_src;

  for (pow = mfs_ctz(idx); pow < max_pow - 1; pow = mfs_ctz(idx))
    {
      mfs_read_page(sb, buf, 4, pg, MFS_PGSZ(sb) - (4 * pow));
      mfs_deser_mfs(buf, &pg);
      idx -= (1 << pow);

      if (pg == 0)
        {
          return 0;
        }
    }

  if (idx == idx_dest)
    {
      return pg;
    }

  /* Falling phase. */

  diff = idx - idx_dest;

  for (pow = mfs_set_msb(diff); diff != 0; pow = mfs_set_msb(diff))
    {
      mfs_read_page(sb, buf, 4, pg, MFS_PGSZ(sb) - (4 * pow));
      mfs_deser_mfs(buf, &pg);
      idx  -= (1 << pow);
      diff -= (1 << pow);

      if (pg == 0)
        {
          return 0;
        }
    }

  finfo("Travel from index %" PRIu32 " at page %" PRIu32 " to index %" PRIu32
        " at page %" PRIu32 ".", idx_src, pg_src, idx_dest, pg);

  return pg;
}
