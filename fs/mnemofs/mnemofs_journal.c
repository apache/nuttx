/****************************************************************************
 * fs/mnemofs/mnemofs_journal.c
 * Journal of mnemofs.
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
 * In mnemofs, the journal stores the path, depth and the new location of the
 * CTZ file called logs, and also the location of the master block. The first
 * n blocks of the journal store the logs, while the last two blocks contain
 * master nodes, and the blocks are called as master blocks. The two master
 * blocks are identical copies for backup.
 *
 * Due to LRU, and the structure of mnemofs, the first n blocks of the
 * journal get filled up much faster than the master blocks, and move more.
 * There will be certain point where the entire journal (the n+2 blocks)
 * move, but mostly, its the first n blocks that move.
 *
 * The first block starts with an 8 byte magic sequence, a 2 bytes long
 * number denoting number of blocks in the journal, and then follows up
 * with an array containing the block numbers of all blocks in the journal
 * including the first block. Then the logs start.
 *
 * The logs take up size in multiples of pages. There might be unitilzed
 * space at the end of a log.
 *
 * All logs are followed by a byte-long hash of the log.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <endian.h>
#include <nuttx/kmalloc.h>
#include <nuttx/list.h>
#include <sys/param.h>

#include "mnemofs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* NOTE: Even if higher level functions use path as struct mfs_path_s,
 * journal only uses struct mfs_ctz_s to avoid problems during write and
 * read of logs. The offsets in struct mfs_path_s will be applied by the
 * search methods of higher functions.
 */

/* There is a byte-long hash that follows this on-flash. */

struct jrnl_log_s
{
  mfs_t depth;
  struct mfs_ctz_s new;
  FAR struct mfs_ctz_s path[];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int   jrnl_wrlog(FAR struct mfs_sb_s * const sb,
                        FAR const struct mfs_path_s * const path,
                        const mfs_t depth, const struct mfs_ctz_s new_ctz);
static int   jrnl_rdlog(FAR const struct mfs_sb_s * const sb, mfs_t pg,
                        mfs_t idx, FAR mfs_t *new_pg, FAR mfs_t *new_idx,
                        FAR struct jrnl_log_s **log);
static void  jrnl_log_free(FAR struct jrnl_log_s *log);

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
 * Name: jrnl_wrlog
 *
 * Description:
 *   Appends a log to the journal.
 *
 * Input Parameters:
 *   sb      - Superblock instance of the device.
 *   path    - CTZ representation of the relpath.
 *   depth   - Length of path.
 *   new_ctz - The updated location.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   Assumes the CTZ list to be updated is `path[depth - 1].ctz`.
 *
 ****************************************************************************/

static int jrnl_wrlog(FAR struct mfs_sb_s * const sb,
                      FAR const struct mfs_path_s * const path,
                      const mfs_t depth, const struct mfs_ctz_s new_ctz)
{
  int      ret    = OK;
  mfs_t    i;
  mfs_t    pg;
  mfs_t    idx;
  mfs_t    sz;
  mfs_t    blk;
  mfs_t    n_pgs;
  mfs_t    rem_sz;
  FAR char *tmp;
  FAR char *buf   = NULL;

  /* FUTURE TODO: Group logs together. Common hash and a prefix for number
   * of logs together.
   */

  /* Serialize. */

  sz  = sizeof(struct jrnl_log_s) + depth * sizeof(struct mfs_ctz_s) + 1;
  buf = kmm_zalloc(sz);
  if (predict_false(buf == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  tmp = buf;
  tmp = mfs_ser_mfs(depth, tmp);
  tmp = mfs_ser_ctz(&new_ctz, tmp);
  for (i = 0; i < depth; i++)
    {
      tmp = mfs_ser_ctz(&path[i].ctz, tmp);
    }

  tmp = mfs_ser_8(mfs_arrhash(buf, sz - 1), tmp); /* Hash */

  /* Write. */

  pg    = MFS_JRNL(sb).log_cpg;
  idx   = MFS_JRNL(sb).log_cblkidx;
  blk   = MFS_PG2BLK(sb, pg);
  n_pgs = (sz + (MFS_PGSZ(sb) - 1)) / MFS_PGSZ(sb); /* Ceil */
  tmp   = buf;

  for (i = 0; i < n_pgs; i++)
    {
      rem_sz = MIN(MFS_PGSZ(sb), buf + sz - tmp);
      ret    = mfs_write_page(sb, tmp, rem_sz, pg, 0);
      if (predict_false(ret < 0))
        {
          goto errout_with_buf;
        }

      pg++;

      if (pg - MFS_BLK2PG(sb, blk) >= MFS_PGINBLK(sb))
        {
          idx++;
          if (idx >= MFS_JRNL(sb).n_blks)
            {
              /* TODO: Time for journal flushing and moving. */
            }

          blk = mfs_jrnl_blkidx2blk(sb, idx);
          pg  = MFS_BLK2PG(sb, blk);
        }

      tmp += rem_sz;
    }

  MFS_JRNL(sb).log_cpg     = pg;
  MFS_JRNL(sb).log_cblkidx = idx;
  MFS_JRNL(sb).n_logs      += 1;

errout_with_buf:
  kmm_free(buf);

errout:
  return ret;
}

/****************************************************************************
 * Name: jrnl_rdlog
 *
 * Description:
 *   Read a log to the journal from given location.
 *
 * Input Parameters:
 *   sb         - Superblock instance of the device.
 *   pg         - Page number of the start of the log.
 *   idx        - Index of the start of the log.
 *   new_pg     - Page number of the start of the next log.
 *   new_idx    - Index of the start of the next log.
 *   log        - To populate with the log.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   This assumes that at any possibly attainable locations in the journal,
 *   there will atleast be 4 bytes of data left in that page to read.
 *
 *   Everything stored in the journal blocks is multiple of 4, and any block
 *   size is a power of 2, so any block size greater than 4 should also be a
 *   multiple of 4.
 *
 *   Free the log after use.
 *
 ****************************************************************************/

static int jrnl_rdlog(FAR const struct mfs_sb_s * const sb, mfs_t pg,
                      mfs_t idx, FAR mfs_t *new_pg, FAR mfs_t *new_idx,
                      FAR struct jrnl_log_s **log)
{
  int            ret        = OK;
  char           buftmp[4];
  mfs_t          n_pgs;
  mfs_t          i;
  mfs_t          sz;
  mfs_t          blk;
  mfs_t          depth;
  mfs_t          rem_sz;
  uint8_t        hash;
  FAR char       *buf       = NULL;
  FAR char       *tmp;
  FAR const char *tmp2;

  blk = MFS_PG2BLK(sb, pg);

  /* Read depth. */

  mfs_read_page(sb, buftmp, 5, pg, 0);
  mfs_deser_mfs(buftmp, &depth);

  sz    = sizeof(struct jrnl_log_s) + (depth * sizeof(struct mfs_ctz_s)) + 1;
  buf   = kmm_zalloc(sz);
  if (predict_false(buf == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  *log  = kmm_zalloc(sz - 1);
  if (predict_false(*log == NULL))
    {
      goto errout_with_buf;
    }

  /* Read the log. */

  n_pgs = (sz + (MFS_PGSZ(sb) - 1)) / MFS_PGSZ(sb);
  tmp   = buf;

  for (i = 0; i < n_pgs; i++)
    {
      rem_sz = MIN(MFS_PGSZ(sb), buf + sz - tmp);
      ret    = mfs_read_page(sb, tmp, rem_sz, pg, 0);
      if (predict_false(ret < 0))
        {
          goto errout_with_log;
        }

      pg++;

      if (pg - MFS_BLK2PG(sb, blk) >= MFS_PGINBLK(sb))
        {
          idx++;
          if (idx >= MFS_JRNL(sb).n_blks)
            {
              /* TODO: Time for journal flushing and moving. */
            }

          blk = mfs_jrnl_blkidx2blk(sb, idx);
          pg  = MFS_BLK2PG(sb, blk);
        }

      tmp += rem_sz;
    }

  /* Deserialize */

  tmp2 = buf;
  tmp2 = mfs_deser_ctz(tmp2, &(*log)->new);
  for (i = 0; i < depth; i++)
    {
      tmp2 = mfs_deser_ctz(tmp2, &(*log)->path[i]);
    }

  tmp2 = mfs_deser_8(tmp2, &hash);

  if (hash != mfs_arrhash(buf, sz - 1))
    {
      ret = -EINVAL;
      goto errout_with_log;
    }

  *new_pg  = pg;
  *new_idx = idx;

  kmm_free(buf);
  return ret;

errout_with_log:
  kmm_free(*log);
  *log = NULL;

errout_with_buf:
  kmm_free(buf);

errout:
  return ret;
}

/****************************************************************************
 * Name: jrnl_log_free
 *
 * Description:
 *   Free a log.
 *
 * Input Parameters:
 *   log        - Journal log.
 *
 ****************************************************************************/

static void jrnl_log_free(FAR struct jrnl_log_s *log)
{
  kmm_free(log);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mfs_jrnl_newlog(FAR struct mfs_sb_s * const sb,
                    FAR const struct mfs_path_s * const path,
                    const mfs_t depth, const struct mfs_ctz_s new_ctz)
{
  return jrnl_wrlog(sb, path, depth, new_ctz);
}

int mfs_jrnl_updatepath(FAR const struct mfs_sb_s * const sb,
                        FAR struct mfs_path_s * const path,
                        const mfs_t depth)
{
  int               ret   = OK;
  mfs_t             k;
  mfs_t             pg;
  mfs_t             idx;
  struct jrnl_log_s *log;

  idx   = MFS_JRNL(sb).log_sblkidx;
  pg    = MFS_JRNL(sb).log_spg;

  for (k = 0; k < MFS_JRNL(sb).n_logs; k++)
    {
      ret = jrnl_rdlog(sb, pg, idx, &pg, &idx, &log);
      if (predict_false(ret < 0))
        {
          return ret;
        }

      if (log->depth > depth)
        {
          continue;
        }

      if (depth == 0 ||
          (log->path[log->depth - 1].pg_e == path[log->depth - 1].ctz.pg_e &&
          log->path[log->depth - 1].idx_e == path[log->depth - 1].ctz.idx_e))
        {
          path[log->depth].ctz = log->new;
        }

      jrnl_log_free(log);
    }

  /* TODO: Create an update offset function in ctz.c to update the offsets
   * of path.
   */

  return ret;
}

int mfs_jrnl_init(FAR struct mfs_sb_s * const sb, mfs_t blk)
{
  mfs_t                pg;
  FAR char             *buf       = NULL;
  char                 buftmp[2];
  int                  ret        = OK;
  mfs_t                sz;
  mfs_t                idx;
  FAR struct jrnl_log_s *log      = NULL;

  /* Magic sequence is already used to find the block, so not required. */

  mfs_read_page(sb, buftmp, 2, MFS_BLK2PG(sb, blk), 8);
  mfs_deser_16(buf, &MFS_JRNL(sb).n_blks);

  if (MFS_JRNL(sb).n_blks == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  sz = 8 + 2 + ((CONFIG_MNEMOFS_JOURNAL_NBLKS + 2) * 4);
  pg = (sz + (MFS_PGSZ(sb) - 1)) / MFS_PGSZ(sb);

  MFS_JRNL(sb).log_cpg       = pg;
  MFS_JRNL(sb).log_cblkidx   = 0;
  MFS_JRNL(sb).log_spg       = MFS_JRNL(sb).log_cpg;
  MFS_JRNL(sb).log_sblkidx   = MFS_JRNL(sb).log_cblkidx;
  MFS_JRNL(sb).jrnlarr_pg    = 0;
  MFS_JRNL(sb).jrnlarr_pgoff = 8 + 2;

  /* Number of logs */

  idx                 = 0;
  MFS_JRNL(sb).n_logs = 0;
  while (idx < MFS_JRNL(sb).n_logs)
    {
      jrnl_rdlog(sb, pg, idx, &pg, &idx, &log);

      /* Assumes checking the depth is enough to check if it's empty. */

      if (log->depth == 0)
        {
          break;
        }

      MFS_JRNL(sb).n_logs++;
      jrnl_log_free(log);
    }

  /* Master node */

  MFS_JRNL(sb).mblk1 = mfs_jrnl_blkidx2blk(sb, MFS_JRNL(sb).n_blks);
  MFS_JRNL(sb).mblk2 = mfs_jrnl_blkidx2blk(sb, MFS_JRNL(sb).n_blks + 1);

  /* TODO: Read all pages in master blocks to find the last master node
   * update.
   */

errout:
  return ret;
}

void mfs_jrnl_free(FAR struct mfs_sb_s * const sb)
{
  /* TODO: Flush entire journal here. */

  finfo("Journal Freed.");
}

int mfs_jrnl_fmt(FAR struct mfs_sb_s * const sb, mfs_t blk1, mfs_t blk2)
{
  int      i;
  int      ret        = OK;
  mfs_t    sz;
  mfs_t    pg;
  mfs_t    mn;
  mfs_t    blk;
  mfs_t    n_pgs;
  mfs_t    rem_sz;
  mfs_t    alloc_blk;
  FAR char *tmp;
  FAR char *buf       = NULL;

  /* Write magic sequence, size of jrnlarr, and then the jrnlarr. */

  sz    = 8 + 2 + ((CONFIG_MNEMOFS_JOURNAL_NBLKS + 2) * 4);
  n_pgs = (sz + (MFS_PGSZ(sb) - 1)) / MFS_PGSZ(sb);

  buf   = kmm_zalloc(sz);
  if (predict_false(buf == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  if (blk1 == 0 && blk2 == 0)
    {
      blk1 = mfs_ba_getblk(sb);
      if (predict_false(blk1 == 0))
        {
          ret = -ENOSPC;
          goto errout_with_buf;
        }

      finfo("Allocated Master Block 1: %d.", blk1);

      blk2 = mfs_ba_getblk(sb);
      if (predict_false(blk2 == 0))
        {
          ret = -ENOSPC;
          goto errout_with_buf;
        }

      finfo("Allocated Master Block 1: %d.", blk2);
      finfo("New locations for Master Blocks %d & %d.", blk1, blk2);
    }

  tmp = buf;

  tmp = mfs_ser_str(MFS_JRNL_MAGIC, 8, tmp);
  tmp = mfs_ser_16(CONFIG_MNEMOFS_JOURNAL_NBLKS, tmp);
  for (i = 0; i < CONFIG_MNEMOFS_JOURNAL_NBLKS; i++)
    {
      alloc_blk = mfs_ba_getblk(sb);
      tmp       = mfs_ser_mfs(alloc_blk, tmp);

      if (predict_false(i == 0))
        {
          blk = alloc_blk;
        }

      finfo("Allocated Journal Block %d at Block %d.", i, alloc_blk);
    }

  tmp = mfs_ser_mfs(blk1, tmp);
  tmp = mfs_ser_mfs(blk2, tmp);

  finfo("All Journal Blocks allocated.");

  tmp = buf;
  pg  = MFS_BLK2PG(sb, blk);
  for (i = 0; i < n_pgs; i++)
    {
      rem_sz = MIN(MFS_PGSZ(sb), buf + sz - tmp);
      ret    = mfs_write_page(sb, tmp, rem_sz, pg, 0);
      if (predict_false(ret < 0))
        {
          goto errout_with_buf;
        }

      pg++;

      DEBUGASSERT(pg < MFS_BLK2PG(sb, blk) + MFS_PGINBLK(sb));

      tmp += rem_sz;
    }

  ret = OK; /* We reach here, we OK. */

  finfo("Written magic sequence, size and journal array into the journal.");

  MFS_JRNL(sb).n_logs        = 0;
  MFS_JRNL(sb).log_cpg       = pg;
  MFS_JRNL(sb).log_cblkidx   = 0;
  MFS_JRNL(sb).log_spg       = MFS_JRNL(sb).log_cpg;
  MFS_JRNL(sb).log_sblkidx   = MFS_JRNL(sb).log_cblkidx;
  MFS_JRNL(sb).jrnlarr_pg    = MFS_BLK2PG(sb, blk);
  MFS_JRNL(sb).jrnlarr_pgoff = 8 + 2;
  MFS_JRNL(sb).n_blks        = CONFIG_MNEMOFS_JOURNAL_NBLKS;
  MFS_JRNL(sb).mblk1         = blk1;
  MFS_JRNL(sb).mblk2         = blk2;

  /* Master node */

  mn = mfs_mn_fmt(sb, blk);
  if (predict_false(mn == 0))
    {
      ret = -ENOSPC;
      goto errout_with_buf;
    }

  /* TODO: Write master node's location in blk1, blk2. */

errout_with_buf:
  kmm_free(buf);

errout:
  return ret;
}

mfs_t mfs_jrnl_blkidx2blk(FAR const struct mfs_sb_s * const sb,
                          const mfs_t blk_idx)
{
  int   ret     = OK;
  mfs_t pg;
  mfs_t pgoff;
  mfs_t blk;
  mfs_t idx;
  char  buf[4];

  pg    = MFS_JRNL(sb).jrnlarr_pg;
  pgoff = MFS_JRNL(sb).jrnlarr_pgoff;
  blk   = MFS_PG2BLK(sb, pg);

  pgoff += blk_idx * 4;

  if (pgoff > MFS_PGSZ(sb))
    {
      pg += pgoff / MFS_PGSZ(sb);
      pgoff %= MFS_PGSZ(sb);
    }

  /* No pg overflow. The blocks have to be big enough. */

  DEBUGASSERT(pg < (MFS_BLK2PG(sb, blk) + MFS_PGINBLK(sb)));

  ret = mfs_read_page(sb, buf, 4, pg, pgoff);
  if (predict_false(ret < 0))
    {
      return 0;
    }

  mfs_deser_mfs(buf, &idx);

  /* FUTURE TODO: Make it such that the entire jrnlarr doesn't need to be in
   * a single block.
   */

  return idx;
}
