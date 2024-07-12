/****************************************************************************
 * fs/mnemofs/mnemofs_ctz.c
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
static mfs_t  ctz_travel(FAR const struct mfs_sb_s * const sb, mfs_t idx_src,
                         mfs_t pg_src, mfs_t idx_dest);
static void   ctz_copyidxptrs(FAR const struct mfs_sb_s * const sb,
                              FAR struct mfs_ctz_s ctz, const mfs_t idx,
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
  return idx == 0 ? 0 : mfs_ctz(idx) + 1;
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
  const mfs_t den = MFS_PGSZ(sb) - 8;
  if (off < MFS_PGSZ(sb))
    {
      *idx = 0;
      *pgoff = off;
      return;
    }

  *idx   = floor((off - 4 * (__builtin_popcount((off / den) - 1) + 2))
           / den);
  *pgoff = off - den * (*idx) - 4 * __builtin_popcount(*idx);
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
  return MFS_PGSZ(sb) - (ctz_idx_nptrs(idx) * MFS_LOGPGSZ(sb));
}

/****************************************************************************
 * Name: ctz_travel
 *
 * Description:
 *   From CTZ block at page `pg_src` and index `idx_src`, give the page
 *   number of index `idx_dest`.
 *
 *   The source is preferably the last CTZ block in the CTZ list, but it can
 *   realistically be any CTZ block in the CTZ list whos position is known.
 *   However, `idx_dest <= idx_src` has to be followed. Takes O(log(n))
 *   complexity to travel.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   idx_src  - Index of the source ctz block.
 *   pg_src   - Page number of the source ctz block.
 *   idx_dest - Index of the destination ctz block.
 *
 * Returned Value:
 *   The page number corresponding to `idx_dest`.
 *
 * Assumptions/Limitations:
 *   `idx_dest <= idx_src`.
 *
 ****************************************************************************/

static mfs_t ctz_travel(FAR const struct mfs_sb_s * const sb, mfs_t idx_src,
                        mfs_t pg_src, mfs_t idx_dest)
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
    }

  return pg;
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
 *   sb    - Superblock instance of the device.
 *   ctz   - CTZ list to use as a reference.
 *   idx   - Index of the block who's supposed pointers are to be copied.
 *   buf   - Buffer representing the entire CTZ block where pointers are
 *           copied to.
 *
 * Assumptions/Limitations:
 *   This assumes `idx` is not more than `ctz->idx_e + 1`.
 *
 ****************************************************************************/

static void ctz_copyidxptrs(FAR const struct mfs_sb_s * const sb,
                            FAR struct mfs_ctz_s ctz, const mfs_t idx,
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
      ctz.pg_e  = ctz_travel(sb, ctz.idx_e, ctz.pg_e, idx - 1);
      ctz.idx_e = idx - 1;
    }

  buf += MFS_PGSZ(sb); /* Go to buf + pg_sz */

  for (i = 0; i < n_ptrs; i++)
    {
      if (predict_false(i == 0))
        {
          prev_idx = ctz.idx_e;
          prev_pg  = ctz.pg_e;
        }
      else
        {
          prev_pg  = ctz_travel(sb, prev_idx, prev_pg, prev_idx - 1);
          prev_idx--;
        }

      ctz.idx_e = prev_idx;

      /* Do buf + pg_sz - (idx * sizeof(mfs_t)) iteratively. */

      buf -= MFS_CTZ_PTRSZ;
      mfs_ser_mfs(prev_pg, buf);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mfs_ctz_rdfromoff(FAR struct mfs_sb_s * const sb, mfs_t data_off,
                      FAR struct mfs_path_s * const path, const mfs_t depth,
                      FAR char *buf, mfs_t buflen)
{
  int              ret  = OK;
  mfs_t            sz;
  mfs_t            pg;
  mfs_t            idx;
  mfs_t            off;
  struct mfs_ctz_s ctz;

  /* Get updated location from the journal */

  DEBUGASSERT(depth > 0);
  ctz = path[depth - 1].ctz;

  ctz_off2loc(sb, data_off, &idx, &off);

  /* TODO: Make the traversal in reverse direction. It would cause
   * a lot less traversals.
   */

  while (idx <= ctz.idx_e && off - data_off < buflen)
    {
      sz = ctz_blkdatasz(sb, idx);
      pg = ctz_travel(sb, ctz.idx_e, ctz.pg_e, idx);

      ret = mfs_read_page(sb, buf, sz, pg, off);
      if (predict_false(ret < 0))
        {
          return ret;
        }

      off = 0;
      idx++;
      buf += sz;
    }

  return ret;
}

int mfs_ctz_nwrtooff(FAR struct mfs_sb_s * const sb,
                     FAR struct mfs_node_s *node,
                     FAR struct mfs_path_s * const path, const mfs_t depth,
                     const mfs_t ctz_sz, FAR struct mfs_ctz_s *new_ctz)
{
  int                    ret       = OK;
  bool                   del;
  mfs_t                  pg;
  mfs_t                  sz;
  mfs_t                  inc;
  mfs_t                  idx;
  mfs_t                  bytes;
  mfs_t                  pgoff;
  mfs_t                  itr_min;
  mfs_t                  itr_max;
  mfs_t                  neg_off; /* Negative offset due to delete. */
  FAR char               *buf      = NULL;
  const mfs_t            range_min = node->range_min;
  const mfs_t            range_max = node->range_max;
  struct mfs_ctz_s       ctz;
  FAR struct mfs_delta_s *delta    = NULL;

  buf = kmm_zalloc(MFS_PGSZ(sb));
  if (predict_false(buf == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* CoW of items in block before range_min */

  ctz_off2loc(sb, range_min, &idx, &pgoff);
  DEBUGASSERT(depth > 0);
  pg = ctz_travel(sb, path[depth - 1].ctz.idx_e, path[depth - 1].ctz.pg_e,
                  idx);
  if (predict_false(pg == 0))
    {
      ret = -EINVAL;
      goto errout_with_buf;
    }

  ctz.idx_e = idx - 1;
  ctz.pg_e  = pg;
  mfs_read_page(sb, buf, pgoff, pg, pgoff);

  /* Updates */

  bytes   = range_min;
  itr_max = range_min;
  neg_off = 0;
  del     = false;

  while (bytes < range_max)
    {
      if (!del)
        {
          memset(buf, 0, MFS_PGSZ(sb));
          sz = ctz_blkdatasz(sb, ctz.idx_e + 1);
          itr_min = itr_max;
          itr_max += sz;
          ctz_copyidxptrs(sb, ctz, ctz.idx_e + 1, buf);
        }
      else
        {
          inc = itr_max - itr_min;
          itr_min = itr_max;
          itr_min = sz - inc;
        }

      del = false;
      mfs_ctz_rdfromoff(sb, itr_min + neg_off, path, depth, buf + pgoff, sz);

      list_for_every_entry(&node->delta, delta, struct mfs_delta_s, list)
        {
          if (delta->off + delta->n_b <= itr_min || itr_max <= delta->off)
            {
              /* Out of range */

              continue;
            }

          inc = MIN(itr_max, delta->off + delta->n_b) - delta->off;

          if (delta->upd != NULL)
            {
              /* Update */

              memcpy(buf + pgoff + (delta->off - itr_min), delta->upd, inc);
            }
          else
            {
              /* Delete */

              memmove(buf + pgoff + (delta->off - itr_min),
                      buf + pgoff + (delta->off - itr_min) + inc,
                      itr_max - (delta->off + inc));
              itr_max -= inc;
              neg_off += inc;
              del     = true;
            }
        }

      bytes += itr_max - itr_min;

      if (!del)
        {
          pgoff    = 0;
          pg = mfs_ba_getpg(sb);
          if (pg == 0)
            {
              ret = -ENOSPC;
              goto errout_with_buf;
            }

          mfs_write_page(sb, buf, MFS_PGSZ(sb), pg, 0);
          ctz.pg_e = pg;
          ctz.idx_e++;
        }
      else
        {
          pgoff += itr_max - itr_min;
        }
    }

  /* Copy rest of the file. */

  if (del)
    {
      mfs_ctz_rdfromoff(sb, itr_max + neg_off, path, depth, buf + pgoff,
                        sz - pgoff);
      pg = mfs_ba_getpg(sb);
      if (pg == 0)
        {
          ret = -ENOSPC;
          goto errout_with_buf;
        }

      mfs_write_page(sb, buf, MFS_PGSZ(sb), pg, 0);
      ctz.pg_e = pg;
      ctz.idx_e++;
      itr_max += sz - pgoff;
      pgoff   = 0;
    }

  while (bytes < ctz_sz)
    {
      sz = ctz_blkdatasz(sb, ctz.idx_e + 1);
      mfs_ctz_rdfromoff(sb, itr_max + neg_off, path, depth, buf,
                        MIN(MFS_PGSZ(sb), ctz_sz - bytes));
      pg = mfs_ba_getpg(sb);
      if (pg == 0)
        {
          ret = -ENOSPC;
          goto errout_with_buf;
        }

      mfs_write_page(sb, buf, MFS_PGSZ(sb), pg, 0);
      bytes += MIN(MFS_PGSZ(sb), ctz_sz - bytes);
    }

  /* TODO: Check for cases where delete, but no further data at the end.
   * , which might cause an infinite loop.
   */

errout_with_buf:
  kmm_free(buf);

errout:
  return ret;
}

int mfs_ctz_wrtooff(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
                    mfs_t o_bytes, const mfs_t n_bytes,
                    mfs_t o_ctz_sz, FAR struct mfs_path_s * const path,
                    const mfs_t depth, FAR const char *buf,
                    FAR struct mfs_ctz_s *ctz)
{
  int              ret            = OK;
  bool             partial;
  mfs_t            partial_bytes; /* Bytes partially filled. */
  mfs_t            pg;
  mfs_t            off;
  mfs_t            idx;
  mfs_t            bytes;
  mfs_t            o_pg;
  mfs_t            o_off;
  mfs_t            o_idx;
  mfs_t            o_rembytes;
  mfs_t            o_data_off;
  mfs_t            n_pg;
  mfs_t            n_data_off;
  mfs_t            ctz_blk_datasz;
  FAR char         *data;
  struct mfs_ctz_s o_ctz;
  struct mfs_ctz_s n_ctz;

  data = kmm_zalloc(MFS_PGSZ(sb));
  if (predict_false(data == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Get updated location from the journal. */

  DEBUGASSERT(depth > 0);
  o_ctz = path[depth - 1].ctz;

  /* TODO: Make the traversal in reverse direction. It would cause
   * a lot less traversals.
   */

  ctz_off2loc(sb, data_off, &idx, &off);

  /* Reach the common part. */

  if (idx != 0 && off != 0)
    {
      pg = ctz_travel(sb, o_ctz.idx_e, o_ctz.pg_e, idx - 1);
      n_ctz.idx_e = idx - 1;
      n_ctz.pg_e  = pg;
    }
  else
    {
      pg = o_ctz.pg_e;
      n_ctz.idx_e = 0;
      n_ctz.pg_e  = pg;
      finfo("CTZ: %u %u %u", pg, n_ctz.idx_e, n_ctz.pg_e);
    }

  /* Add new data from buf. */

  partial    = false;
  o_rembytes = o_ctz_sz - (data_off + o_bytes);
  bytes      = data_off;
  if (n_bytes != 0)
    {
      while (bytes < data_off + n_bytes)
        {
          n_pg = mfs_ba_getpg(sb);
          if (n_pg)
            {
              ret = -ENOSPC;
              goto errout_with_data;
            }

          ctz_copyidxptrs(sb, n_ctz, idx, data); /* Handles idx == 0. */
          ctz_blk_datasz = ctz_blkdatasz(sb, idx);

          if (predict_false(off != 0))
            {
              /* This happens at max for the first iteration of the loop. */

              mfs_read_page(sb, data, off,
                            ctz_travel(sb, o_ctz.idx_e, o_ctz.pg_e, idx), 0);
              bytes += off;
            }

          memcpy(data + off, buf + bytes,
                MIN(ctz_blk_datasz - off, n_bytes - bytes));
          if (n_bytes - bytes < ctz_blk_datasz - off && o_rembytes != 0)
            {
              partial = true;
              partial_bytes = n_bytes - bytes;
            }
          else
            {
              bytes += ctz_blk_datasz - off;
            }

          if (predict_false(off != 0))
            {
              /* This happens at max for the first iteration of the loop. */

              off = 0;
            }

          if (predict_true(!partial))
            {
              mfs_write_page(sb, data, MFS_PGSZ(sb), n_pg, 0);
              n_ctz.idx_e = idx;
              n_ctz.pg_e  = pg;
            }
          else
            {
              /* In this case, we have a situation where there is still
               * some empty area left in the page, and there is some data
               * that is waiting to be fit in there, so we won't write it
               * to the flash JUST yet. We'll fill in the rest of the data
               * and THEN write it.
               *
               * It's okay to leave the loop as this case will happen, if at
               * all, on the last iteration of the loop.
               */

              n_ctz.idx_e = idx;
              n_ctz.pg_e  = pg;
              break;
            }

          idx++;

          memset(data, 0, MFS_PGSZ(sb));
        }
    }

  o_data_off = data_off + o_bytes;
  n_data_off = data_off + n_bytes;

  /* Completing partially filled data, if present. */

  if (partial)
    {
      ctz_off2loc(sb, o_data_off, &o_idx, &o_off);
      o_pg = ctz_travel(sb, o_ctz.idx_e, o_ctz.pg_e, o_idx);
      mfs_read_page(sb, data + partial_bytes,
                    ctz_blkdatasz(sb, n_ctz.idx_e) - partial_bytes, o_pg,
                    o_off);
      mfs_write_page(sb, data, MFS_PGSZ(sb), n_ctz.pg_e, 0);

      o_data_off    += partial_bytes;
      n_data_off    += partial_bytes;
      partial_bytes = 0;
      partial       = false;
    }

  /* Adding old bytes in. */

  while (o_data_off < o_ctz_sz)
    {
      memset(data, 0, MFS_PGSZ(sb));

      pg = mfs_ba_getpg(sb);
      if (predict_false(pg == 0))
        {
          ret = -ENOSPC;
          goto errout_with_data;
        }

      ctz_blk_datasz = ctz_blkdatasz(sb, n_ctz.idx_e + 1);
      ctz_copyidxptrs(sb, n_ctz, n_ctz.idx_e + 1, data);

      mfs_ctz_rdfromoff(sb, o_data_off, path, depth, data, ctz_blk_datasz);
      mfs_write_page(sb, data, MFS_PGSZ(sb), pg, 0);

      n_ctz.idx_e++;
      n_ctz.pg_e = pg;

      o_data_off += ctz_blk_datasz;
    }

  /* path is not updated to point to the new ctz. This is upto the caller. */

  *ctz = n_ctz;

errout_with_data:
  kmm_free(data);

errout:
  return ret;
}
