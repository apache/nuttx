/****************************************************************************
 * fs/mnemofs/mnemofs_journal.c
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
#include "fs_heap.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MFS_JRNL_SUFFIXSZ (8 + 2) /* 8 byte magic sequence + no. of blks */
#define MFS_LOGSZ(depth)  (sizeof(mfs_t) * 2 + sizeof(struct mfs_ctz_s) + \
                           sizeof(struct mfs_path_s) * depth + \
                           sizeof(struct timespec) * 3 + sizeof(uint16_t))

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* NOTE: Even if higher level functions use path as struct mfs_path_s,
 * journal only uses struct mfs_ctz_s to avoid problems during write and
 * read of logs. The offsets in struct mfs_path_s will be applied by the
 * search methods of higher functions.
 */

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

FAR static const char *deser_log(FAR const char * const in,
                                 FAR struct mfs_jrnl_log_s * const x);
FAR static char       *ser_log(FAR const struct mfs_jrnl_log_s * const x,
                               FAR char * const out);

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
 * Name: mfs_jrnl_rdlog
 *
 * Description:
 *   Read a log to the journal from given location, and update location to
 *   point to next log location.
 *
 * Input Parameters:
 *   sb        - Superblock instance of the device.
 *   blkidx    - Journal Block Index of the current block.
 *   pg_in_blk - Page offset in the block.
 *   log       - To populate with the log.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   This function does NOT care about start of the journal, or even, if
 *   the initial requested area is inside the journal. It will malfunction
 *   if not used properly. Usually this is used in an iterative manner, and
 *   hence the first time blkidx and pg_in_blk are initialized, they should
 *   be derived from the values in MFS_JRNL(sb) respectively.
 *
 *   This updates the blkidx and pg_in_blk to point to the next log, and
 *   returns an -ENOSPC when end of journal is reached in traversal.
 *
 *   Free the log after use.
 *
 ****************************************************************************/

int mfs_jrnl_rdlog(FAR const struct mfs_sb_s *const sb,
                      FAR mfs_t *blkidx, FAR mfs_t *pg_in_blk,
                      FAR struct mfs_jrnl_log_s *log)
{
  int       ret       = OK;
  char      tmp[4];
  mfs_t     log_sz;
  mfs_t     jrnl_pg;
  mfs_t     jrnl_blk;
  FAR char *buf       = NULL;

  DEBUGASSERT(*pg_in_blk % MFS_PGSZ(sb) == 0);

  jrnl_blk = mfs_jrnl_blkidx2blk(sb, *blkidx);
  jrnl_pg  = MFS_BLK2PG(sb, jrnl_blk) + *pg_in_blk;

  /* First 4 bytes contain the size of the entire log. */

  ret = mfs_read_page(sb, tmp, 4, jrnl_pg, 0);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  mfs_deser_mfs(tmp, &log_sz);
  if (log_sz == 0)
    {
      ret = -ENOSPC;
      goto errout;
    }

  buf = fs_heap_zalloc(log_sz);
  if (predict_false(buf == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  ret = mfs_read_page(sb, buf, log_sz, jrnl_pg, 4);
  if (predict_false(ret < 0))
    {
      goto errout_with_buf;
    }

  ret = OK;

  if (predict_false(deser_log(buf, log) == NULL))
    {
      ret = -ENOMEM;
      goto errout_with_buf;
    }

  (*pg_in_blk)++;

  if (*pg_in_blk >= MFS_PGINBLK(sb))
    {
      *pg_in_blk = 0;
      (*blkidx)++;
    }

errout_with_buf:
  fs_heap_free(buf);

errout:
  return ret;
}

/****************************************************************************
 * Name: ser_log
 *
 * Description:
 *   Serialize a log.
 *
 * Input Parameters:
 *   n   - Log to serialize
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 * Assumptions/Limitations:
 *   This assumes the out buffer has enough space to hold the inline path.
 *
 *   This doesn't require the hash to be pre-calculated.
 *
 ****************************************************************************/

FAR static char *ser_log(FAR const struct mfs_jrnl_log_s * const x,
                         FAR char * const out)
{
  FAR char *o = out;
  mfs_t i;

  o = mfs_ser_mfs(x->depth, o);
  o = mfs_ser_mfs(x->sz_new, o);
  o = mfs_ser_ctz(&x->loc_new, o);
  o = mfs_ser_timespec(&x->st_mtim_new, o);
  o = mfs_ser_timespec(&x->st_atim_new, o);
  o = mfs_ser_timespec(&x->st_ctim_new, o);

  for (i = 0; i < x->depth; i++)
    {
      o = mfs_ser_path(&x->path[i], o);
    }

  o = mfs_ser_16(mfs_hash(out, o - out), o);
  return o;
}

/****************************************************************************
 * Name: deser_log
 *
 * Description:
 *   Deserialize a log.
 *
 * Input Parameters:
 *   in - Input array from where to deserialize.
 *   n  - Log to deserialize
 *
 * Returned Value:
 *   NULL - Error.
 *   Pointer to byte after the end of serialized value.
 *
 * Assumptions/Limitations:
 *   This allocates space for the path, and the log should freed after use.
 *
 ****************************************************************************/

FAR static const char *deser_log(FAR const char * const in,
                                 FAR struct mfs_jrnl_log_s * const x)
{
  FAR const char *i = in;
  mfs_t k;

  i = mfs_deser_mfs(i, &x->depth);
  i = mfs_deser_mfs(i, &x->sz_new);
  i = mfs_deser_ctz(i, &x->loc_new);
  i = mfs_deser_timespec(i, &x->st_mtim_new);
  i = mfs_deser_timespec(i, &x->st_atim_new);
  i = mfs_deser_timespec(i, &x->st_ctim_new);

  /* Allocates path. Deallocate using mfs_jrnl_log_free. */

  x->path = fs_heap_zalloc(sizeof(struct mfs_jrnl_log_s) * x->depth);
  if (predict_false(x->path == NULL))
    {
      return NULL;
    }

  for (k = 0; k < x->depth; k++)
    {
      i = mfs_deser_path(i, &x->path[k]);
    }

  i = mfs_deser_16(i, &x->hash);
  return i;
}

/****************************************************************************
 * Name: mfs_jrnl_log_free
 *
 * Description:
 *   Free the log.
 *
 * Input Parameters:
 *   log - Log
 *
 * Assumptions/Limitations:
 *   This allocates space for the path, and the log should freed after use.
 *
 ****************************************************************************/

void mfs_jrnl_log_free(FAR const struct mfs_jrnl_log_s * const log)
{
  fs_heap_free(log->path);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mfs_jrnl_init(FAR struct mfs_sb_s * const sb, mfs_t blk)
{
  char              buftmp[2];
  int               ret        = OK;
  mfs_t             sz;
  mfs_t             blkidx;
  mfs_t             pg_in_blk;
  struct mfs_jrnl_log_s log;

  /* Magic sequence is already used to find the block, so not required. */

  mfs_read_page(sb, buftmp, 2, MFS_BLK2PG(sb, blk), 8);
  mfs_deser_16(buftmp, &MFS_JRNL(sb).n_blks);

  if (MFS_JRNL(sb).n_blks == 0)
    {
      ret = -EINVAL;
      goto errout;
    }

  sz = MFS_JRNL_SUFFIXSZ + ((CONFIG_MNEMOFS_JOURNAL_NBLKS + 2) * 4);

  MFS_JRNL(sb).jrnl_start    = blk;
  MFS_JRNL(sb).log_cpg       = (sz + (MFS_PGSZ(sb) - 1)) / MFS_PGSZ(sb);
  MFS_JRNL(sb).log_spg       = MFS_JRNL(sb).log_cpg;
  MFS_JRNL(sb).log_cblkidx   = 0;
  MFS_JRNL(sb).log_sblkidx   = MFS_JRNL(sb).log_cblkidx;
  MFS_JRNL(sb).jrnlarr_pg    = MFS_BLK2PG(sb, blk); /* Assuming pgsz > 10 */
  MFS_JRNL(sb).jrnlarr_pgoff = MFS_JRNL_SUFFIXSZ;

  /* Number of logs */

  MFS_JRNL(sb).n_logs = 0;
  blkidx              = MFS_JRNL(sb).log_sblkidx;
  pg_in_blk           = MFS_JRNL(sb).log_spg % MFS_PGINBLK(sb);

  while (true)
    {
      ret = mfs_jrnl_rdlog(sb, &blkidx, &pg_in_blk, &log);
      if (predict_false(ret < 0 && ret != -ENOSPC))
        {
          goto errout;
        }
      else if (ret == -ENOSPC)
        {
          ret = OK;
          break;
        }

      /* Assumes checking the depth is enough to check if it's empty, as
       * theoretically there are no blocks with depth 0, as root has a
       * depth of 1.
       */

      if (log.depth == 0)
        {
          DEBUGASSERT(log.path == NULL);
          break;
        }

      MFS_JRNL(sb).n_logs++;
      mfs_jrnl_log_free(&log);
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

int mfs_jrnl_fmt(FAR struct mfs_sb_s * const sb, FAR mfs_t *blk1,
                 FAR mfs_t *blk2, FAR mfs_t *jrnl_blk)
{
  int       i;
  int       ret       = OK;
  mfs_t     sz;
  mfs_t     pg;
  mfs_t     blk;
  mfs_t     alloc_blk;
  FAR char *tmp;
  FAR char *buf       = NULL;

  /* TODO: Replace jrnl_blk with MFS_JRNL(sb).jrnl_start if possible. */

  /* Write magic sequence, size of jrnlarr, and then the jrnlarr. */

  sz = MFS_JRNL_SUFFIXSZ + ((CONFIG_MNEMOFS_JOURNAL_NBLKS + 2) * 4);

  buf = fs_heap_zalloc(sz);
  if (predict_false(buf == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  if (*blk1 == 0 && *blk2 == 0)
    {
      *blk1 = mfs_ba_getblk(sb);
      if (predict_false(blk1 == 0))
        {
          ret = -ENOSPC;
          goto errout_with_buf;
        }

      finfo("Allocated Master Block 1: %d.", *blk1);

      *blk2 = mfs_ba_getblk(sb);
      if (predict_false(blk2 == 0))
        {
          ret = -ENOSPC;
          goto errout_with_buf;
        }

      finfo("Allocated Master Block 1: %d.", *blk2);
      finfo("New locations for Master Blocks %d & %d.", *blk1, *blk2);
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
          blk                     = alloc_blk;
          *jrnl_blk               = alloc_blk;
          MFS_JRNL(sb).jrnl_start = alloc_blk;
        }

      finfo("Allocated Journal Block %d at Block %d.", i, alloc_blk);
    }

  tmp = mfs_ser_mfs(*blk1, tmp);
  tmp = mfs_ser_mfs(*blk2, tmp);

  finfo("All Journal Blocks allocated.");

  pg  = MFS_BLK2PG(sb, blk);
  ret = mfs_write_page(sb, buf, sz, pg, 0); /* Assuming array fits in a
                                             * single page.
                                             */
  if (predict_false(ret < 0))
    {
      goto errout_with_buf;
    }

  ret = OK; /* We reach here, we OK. */

  finfo("Written magic sequence, size and journal array into the journal.");

  MFS_JRNL(sb).n_logs        = 0;
  MFS_JRNL(sb).n_blks        = CONFIG_MNEMOFS_JOURNAL_NBLKS;
  MFS_JRNL(sb).log_cpg       = pg + 1; /* Assumes 1 page for jrnl_arr. */
  MFS_JRNL(sb).log_cblkidx   = 0;
  MFS_JRNL(sb).log_spg       = MFS_JRNL(sb).log_cpg;
  MFS_JRNL(sb).log_sblkidx   = MFS_JRNL(sb).log_cblkidx;
  MFS_JRNL(sb).jrnlarr_pg    = MFS_BLK2PG(sb, blk);
  MFS_JRNL(sb).jrnlarr_pgoff = MFS_JRNL_SUFFIXSZ;
  MFS_JRNL(sb).mblk1         = *blk1;
  MFS_JRNL(sb).mblk2         = *blk2;

errout_with_buf:
  fs_heap_free(buf);

errout:
  return ret;
}

void mfs_jrnl_free(FAR struct mfs_sb_s * const sb)
{
  if (!mfs_jrnl_isempty(sb) &&
      MFS_JRNL(sb).log_cblkidx >= MFS_JRNL_LIM(sb))
    {
      mfs_jrnl_flush(sb);
    }

  finfo("Journal Freed.");
}

mfs_t mfs_jrnl_blkidx2blk(FAR const struct mfs_sb_s * const sb,
                          const mfs_t blk_idx)
{
  int   ret = OK;
  mfs_t pg;
  mfs_t idx;
  mfs_t blk;
  mfs_t pgoff;
  char  buf[4];

  pg     = MFS_JRNL(sb).jrnlarr_pg;
  pgoff  = MFS_JRNL(sb).jrnlarr_pgoff;
  blk    = MFS_PG2BLK(sb, pg);
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

int mfs_jrnl_updatedinfo(FAR const struct mfs_sb_s * const sb,
                         FAR struct mfs_path_s * const path,
                         const mfs_t depth)
{
  int               ret         = OK;
  mfs_t             blkidx;
  mfs_t             counter     = 0;
  mfs_t             pg_in_block;
  struct mfs_jrnl_log_s tmplog;

  /* TODO: Allow optional filling of updated timestamps, etc. */

  DEBUGASSERT(depth > 0);

  blkidx      = MFS_JRNL(sb).log_sblkidx;
  pg_in_block = MFS_JRNL(sb).log_spg % MFS_PGINBLK(sb);

  while (blkidx < MFS_JRNL(sb).n_blks && counter < MFS_JRNL(sb).n_logs)
    {
      ret = mfs_jrnl_rdlog(sb, &blkidx, &pg_in_block, &tmplog);
      if (predict_false(ret < 0 && ret != -ENOSPC))
        {
          goto errout;
        }
      else if (ret == -ENOSPC)
        {
          break;
        }

      DEBUGASSERT(tmplog.depth > 0);

      if (tmplog.depth > depth)
        {
          /* Not suitable. */
        }
      else
        {
          DEBUGASSERT(tmplog.depth > 0);

          if (mfs_path_eq(&tmplog.path[tmplog.depth - 1],
                          &path[tmplog.depth - 1]))
            {
              path[tmplog.depth - 1].ctz = tmplog.loc_new;
              path[tmplog.depth - 1].sz  = tmplog.sz_new;
            }
        }

      mfs_jrnl_log_free(&tmplog);
      counter++;
    }

errout:
  return ret;
}

int mfs_jrnl_wrlog(FAR struct mfs_sb_s * const sb,
                   FAR const struct mfs_node_s *node,
                   const struct mfs_ctz_s loc_new, const mfs_t sz_new)
{
  int                    ret      = OK;
  mfs_t                  jrnl_pg;
  FAR char              *buf      = NULL;
  FAR char              *tmp      = NULL;
  const mfs_t            log_sz   = sizeof(mfs_t) + MFS_LOGSZ(node->depth);
  struct mfs_jrnl_log_s  log;

  buf = fs_heap_zalloc(log_sz); /* For size before log. */
  if (predict_false(buf == NULL))
    {
      ret = -ENOMEM;
      goto errout;
    }

  /* Serialize */

  log.depth       = node->depth;
  log.sz_new      = sz_new;
  log.loc_new     = loc_new;
  log.st_mtim_new = node->st_mtim;
  log.st_atim_new = node->st_atim;
  log.st_ctim_new = node->st_ctim;
  log.path        = node->path;    /* Fine as temporarily usage. */

  tmp = buf;
  tmp = mfs_ser_mfs(log_sz - sizeof(mfs_t), tmp); /* First 4 bytes have sz */
  tmp = ser_log(&log, tmp);

  /* Store */

  jrnl_pg  = MFS_JRNL(sb).log_cpg;

  /* TODO: It assumes it takes only one page per log. */

  ret = mfs_write_page(sb, buf, log_sz, jrnl_pg, 0);
  if (predict_false(ret < 0))
    {
      goto errout_with_buf;
    }

  ret = OK;

  jrnl_pg++;

  if (jrnl_pg % MFS_PGINBLK(sb) == 0)
    {
      MFS_JRNL(sb).log_cblkidx++;
    }

  MFS_JRNL(sb).log_cpg = jrnl_pg;
  MFS_JRNL(sb).n_logs++;

errout_with_buf:
  fs_heap_free(buf);

errout:
  return ret;
}

int mfs_jrnl_flush(FAR struct mfs_sb_s * const sb)
{
  /* When a file or a directory is deleted.
   *
   * It will be modified to an entry in the LRU which details the deletion
   * of all bytes from the child... as in, offset 0, deleted bytes is the
   * size of the file.
   *
   * The new "location" can be used as (0, 0) to signify a deletion, even in
   * its journal log.
   *
   * Also ensure if the size gets updated to 0.
   *
   * Then the flush operation problem will be solved for removal of files or
   * directories.
   *
   * Move operation will not empty the child, but only the parent from the
   * old parent.
   */

  /* Time complexity is going to be horrendous. Hint: O(n^2). HOWEVER, as
   * littlefs points out....if n is constant, it's essentially a O(k), or
   * O(1) :D
   */

  /* TODO: Need to consider how the LRU and Journal interact with each other
   * for newly created fs object's entries.
   */

  /* We're using updatectz to update the LRU inside the journal. Think
   * about how that might affect the iteration attempts.
   */

  int                      ret           = OK;
  mfs_t                    blkidx        = MFS_JRNL(sb).log_sblkidx;
  mfs_t                    log_itr       = 0;
  mfs_t                    pg_in_blk     = MFS_JRNL(sb).log_spg \
                                           % MFS_PGINBLK(sb);
  mfs_t                    tmp_blkidx;
  mfs_t                    tmp_pg_in_blk;
  mfs_t                    mn_blk1;
  mfs_t                    mn_blk2;
  mfs_t                    i;
  mfs_t                    jrnl_blk;
  mfs_t                    blk;
  struct mfs_jrnl_log_s    log;
  struct mfs_jrnl_log_s    tmp_log;
  FAR struct mfs_path_s   *path          = NULL;
  struct mfs_jrnl_state_s  j_state;
  struct mfs_mn_s          mn_state;

  while (log_itr < MFS_JRNL(sb).n_logs)
    {
      ret = mfs_jrnl_rdlog(sb, &blkidx, &pg_in_blk, &log);
      if (predict_false(ret < 0))
        {
          DEBUGASSERT(ret != -ENOSPC); /* While condition is sufficient. */
          goto errout;
        }

      if (log.loc_new.pg_e == 0 && log.loc_new.idx_e == 0)
        {
          /* Entry is deleted, do not bother with it. */

          break;
        }

      tmp_blkidx    = blkidx;
      tmp_pg_in_blk = pg_in_blk;

      path = fs_heap_zalloc(log.depth * sizeof(struct mfs_path_s));
      if (predict_false(path == NULL))
        {
          goto errout;
        }

      memcpy(path, log.path, log.depth * sizeof(struct mfs_path_s));
      path[log.depth - 1].ctz = log.loc_new;

      for (; ; )
        {
          ret = mfs_jrnl_rdlog(sb, &tmp_blkidx, &tmp_pg_in_blk, &tmp_log);
          if (ret == -ENOSPC)
            {
              break;
            }
          else if (predict_false(ret < 0))
            {
              mfs_jrnl_log_free(&log);
              mfs_free_patharr(path);
              goto errout;
            }

          if (tmp_log.depth > log.depth)
            {
              mfs_jrnl_log_free(&tmp_log);
              continue;
            }

          if (!mfs_path_eq(&path[tmp_log.depth - 1],
                          &tmp_log.path[tmp_log.depth - 1]))
            {
              mfs_jrnl_log_free(&tmp_log);
              continue;
            }

          path[tmp_log.depth - 1] = tmp_log.path[tmp_log.depth - 1];

          if (tmp_log.loc_new.pg_e == 0 && tmp_log.loc_new.idx_e == 0)
            {
              /* Entry is deleted, do not bother with it. */

              break;
            }
        }

      if (log.depth == 1)
        {
          MFS_MN(sb).root_ctz = path[log.depth - 1].ctz;
          MFS_MN(sb).root_sz  = path[log.depth - 1].sz;

          /* TODO: Other parameters. */
        }
      else
        {
          ret = mfs_lru_updatectz(sb, path, log.depth,
                                  path[log.depth - 1].ctz,
                                  path[log.depth - 1].sz);
          if (predict_false(ret < 0))
            {
              mfs_free_patharr(path);
              mfs_jrnl_log_free(&log);
              goto errout;
            }
        }

      mfs_free_patharr(path);
      mfs_jrnl_log_free(&log);
    }

  if (MFS_MN(sb).mblk_idx == MFS_PGINBLK(sb))
    {
      mn_blk1 = 0;
      mn_blk2 = 0;
    }
  else
    {
      /* FUTURE TODO: Save the two block numbers in master node structure to
       * be faster.
       */

      mn_blk1 = mfs_jrnl_blkidx2blk(sb, MFS_JRNL(sb).n_blks);
      mn_blk2 = mfs_jrnl_blkidx2blk(sb, MFS_JRNL(sb).n_blks + 1);
    }

  /* Reallocate journal. */

  j_state  = MFS_JRNL(sb);
  mn_state = MFS_MN(sb);

  ret = mfs_jrnl_fmt(sb, &mn_blk1, &mn_blk2, &jrnl_blk);
  if (predict_false(ret < 0))
    {
      MFS_JRNL(sb) = j_state;
      goto errout;
    }

  /* Write master node entry. */

  ret = mfs_mn_sync(sb, &path[0], mn_blk1, mn_blk2, jrnl_blk);
  if (predict_false(ret < 0))
    {
      MFS_MN(sb) = mn_state;
      goto errout;
    }

  /* Mark all old blocks of journal (and master blocks) as deletable. */

  for (i = 0; i < MFS_JRNL(sb).n_blks + 2; i++)
    {
      blk = mfs_jrnl_blkidx2blk(sb, i);
      mfs_ba_blkmarkdel(sb, blk);
    }

  /* Delete outdated blocks. */

  ret = mfs_ba_delmarked(sb);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

errout:
  return ret;
}

bool mfs_jrnl_isempty(FAR const struct mfs_sb_s * const sb)
{
  return MFS_JRNL(sb).n_logs == 0;
}
