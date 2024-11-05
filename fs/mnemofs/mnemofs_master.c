/****************************************************************************
 * fs/mnemofs/mnemofs_master.c
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
 * In mnemofs, the master node points to the root of the file system. It
 * contains the information about the root, and when the root is updated,
 * the master node needs to point to the updated location, and thus, needs to
 * update the master node.
 *
 * Master nodes sit at the very end of the journal. The last two blocks of
 * the journal are called master blocks, and they are filled with a new
 * entry for a master node everytime it is updated. They are filled in a
 * sequential manner, and thus, the latest master node can be found easily.
 * The two master blocks contain identical information, and exist to be as a
 * backup.
 *
 * The stored master nodes are basically `struct mfs_mn_s` without the
 * redundant `pg` member.
 *
 * The master node also points to the start of the journal, and thus, when
 * the journal moves, a new master node entry is added.
 *
 * A master node update, when written to the file system, marks the end of
 * an update of the file system tree. Thus, at this point, any obsolete data
 * that can be erased, will be erased by the block allocator. Only after
 * writing the master block is the file system tree updated. Before this,
 * the old file system tree is accessible through the older master node, and
 * can be accessed again during power loss.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/kmalloc.h>
#include <sys/stat.h>

#include "mnemofs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static FAR char       *ser_mn(const struct mfs_mn_s mn,
                              FAR char * const out);
static FAR const char *deser_mn(FAR const char * const in,
                                FAR struct mfs_mn_s *mn, FAR uint16_t *hash);

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
 * Name: ser_mn
 *
 * Description:
 *   Serialize master node.
 *
 * Input Parameters:
 *   mn  - Master node.
 *   out - Out buffer.
 *
 * Returned Value:
 *   Pointer to the end of the serialized data in `out`.
 *
 * Assumptions/Limitations:
 *   Out should contain enough space for `mn` and 1 byte extra for the hash.
 *
 ****************************************************************************/

static FAR char *ser_mn(const struct mfs_mn_s mn, FAR char * const out)
{
  FAR char *tmp = out;

  tmp = mfs_ser_mfs(mn.jrnl_blk, tmp);
  tmp = mfs_ser_mfs(mn.mblk_idx, tmp);
  tmp = mfs_ser_ctz(&mn.root_ctz, tmp);
  tmp = mfs_ser_mfs(mn.root_sz, tmp);
  tmp = mfs_ser_timespec(&mn.ts, tmp);
  tmp = mfs_ser_16(mfs_hash(out, tmp - out), tmp);

  /* TODO: Update this, and the make a macro for size of MN. */

  return tmp;
}

/****************************************************************************
 * Name: ser_mn
 *
 * Description:
 *   Deserialize master node.
 *
 * Input Parameters:
 *   in   - In buffer.
 *   mn   - Master node to populate.
 *   hash - Stored hash (of serialized data) to populate.
 *
 * Returned Value:
 *   Pointer to the end of the serialized data in `in`.
 *
 * Assumptions/Limitations:
 *   In should contain enough space for `mn` and 1 byte extra for the hash.
 *
 ****************************************************************************/

static FAR const char *deser_mn(FAR const char * const in,
                                FAR struct mfs_mn_s *mn, FAR uint16_t *hash)
{
  FAR const char *tmp = in;

  tmp = mfs_deser_mfs(tmp, &mn->jrnl_blk);
  tmp = mfs_deser_mfs(tmp, &mn->mblk_idx);
  tmp = mfs_deser_ctz(tmp, &mn->root_ctz);
  tmp = mfs_deser_mfs(tmp, &mn->root_sz);
  tmp = mfs_deser_timespec(tmp, &mn->ts);
  tmp = mfs_deser_16(tmp, hash);

  /* TODO: Update this, and the make a macro for size of MN. */

  return tmp;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mfs_mn_init(FAR struct mfs_sb_s * const sb, const mfs_t jrnl_blk)
{
  int             ret          = OK;
  bool            found        = false;
  mfs_t           i            = 0;
  mfs_t           mblk1;
  mfs_t           blkidx;
  mfs_t           pg_in_blk;
  mfs_t           jrnl_blk_tmp;
  uint16_t        hash;
  struct mfs_mn_s mn;
  const mfs_t     sz           = sizeof(struct mfs_mn_s) - sizeof(mn.pg);
  char            buftmp[4];
  char            buf[sz + 1];
  struct mfs_jrnl_log_s log;

  mblk1       = mfs_jrnl_blkidx2blk(sb, MFS_JRNL(sb).n_blks);

  mn.jrnl_blk = jrnl_blk;
  mn.mblk_idx = 0;
  mn.pg       = MFS_BLK2PG(sb, mblk1);

  for (i = 0; i < MFS_PGINBLK(sb); i++)
    {
      mfs_read_page(sb, buftmp, 4, mn.pg, 0);
      mfs_deser_mfs(buftmp, &jrnl_blk_tmp);

      if (jrnl_blk_tmp == 0)
        {
          break;
        }

      found = true;
      mn.mblk_idx++;
      mn.pg++;
    }

  if (found == false)
    {
      ret = -EINVAL;
      goto errout;
    }

  if (i == MFS_PGINBLK(sb))
    {
      ret = -ENOSPC;
      goto errout;
    }
  else
    {
      mn.pg--;
    }

  mfs_read_page(sb, buf, sz + 1, mn.pg, 0);
  deser_mn(buf, &mn, &hash);
  if (hash != mfs_hash(buf, sz))
    {
      ret = -EINVAL;
      goto errout;
    }

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

      if (log.depth == 1)
        {
          mn.root_ctz = log.loc_new;
          mn.root_sz  = log.sz_new;
        }

      mfs_jrnl_log_free(&log);
    }

  /* FUTURE TODO: Recovery in case of hash not matching, or page not
   * readable.
   */

  mn.root_mode = 0777 | S_IFDIR;

  MFS_MN(sb) = mn;
errout:
  return ret;
}

int mfs_mn_fmt(FAR struct mfs_sb_s * const sb, const mfs_t mblk1,
               const mfs_t mblk2, const mfs_t jrnl_blk)
{
  int              ret         = OK;
  mfs_t            pg;
  struct mfs_mn_s  mn;
  struct timespec  ts;
  const mfs_t      sz          = sizeof(struct mfs_mn_s) - sizeof(mn.pg);
  char             buf[sz + 1];

  clock_gettime(CLOCK_REALTIME, &ts);

  memset(buf, 0, sz + 1);

  pg = mfs_ba_getpg(sb);
  if (predict_false(pg == 0))
    {
      ret = -ENOSPC;
      goto errout;
    }

  finfo("Root formatted to be at Page %u", pg);

  mn.root_ctz.idx_e = 0;
  mn.root_ctz.pg_e  = pg;
  mn.jrnl_blk       = jrnl_blk;
  mn.mblk_idx       = 0;
  mn.pg             = MFS_BLK2PG(sb, mblk1);
  mn.root_sz        = 0;
  mn.ts             = ts;
  mn.root_st_atim   = ts;
  mn.root_st_ctim   = ts;
  mn.root_st_mtim   = ts;
  mn.root_mode      = 0777 | S_IFDIR;

  /* Serialize. */

  ser_mn(mn, buf);

  ret = mfs_write_page(sb, buf, sz, MFS_BLK2PG(sb, mblk1), 0);
  if (predict_false(ret < 0))
    {
      goto errout;
    }
  else
    {
      ret = OK;
    }

  ret = mfs_write_page(sb, buf, sz, MFS_BLK2PG(sb, mblk2), 0);
  if (predict_false(ret < 0))
    {
      goto errout;
    }
  else
    {
      ret = OK;
    }

  mn.mblk_idx = 1;
  MFS_MN(sb) = mn;
  finfo("Master node written. Now at page %d, timestamp %lld.%.9ld.",
        MFS_MN(sb).pg, (long long)MFS_MN(sb).ts.tv_sec,
        MFS_MN(sb).ts.tv_nsec);

errout:
  return ret;
}

int mfs_mn_move(FAR struct mfs_sb_s * const sb, struct mfs_ctz_s root,
                const mfs_t root_sz)
{
  int             ret          = OK;
  struct mfs_mn_s mn;
  const mfs_t     sz           = sizeof(struct mfs_mn_s) - sizeof(mn.pg);
  char            buf[sz + 1];

  if (MFS_MN(sb).mblk_idx == MFS_PGINBLK(sb) - 1)
    {
      /* TODO: Move journal. Master blocks are full. */
    }

  mn = MFS_MN(sb);

  mn.root_ctz = root;
  mn.root_sz = root_sz;
  mn.mblk_idx++; /* TODO */
  mn.pg++;

  ser_mn(mn, buf);

  ret = mfs_write_page(sb, buf, sz + 1, mn.pg, 0);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  MFS_MN(sb) = mn;

errout:
  return ret;
}

int mfs_mn_sync(FAR struct mfs_sb_s *sb,
                FAR struct mfs_path_s * const new_loc,
                const mfs_t blk1, const mfs_t blk2, const mfs_t jrnl_blk)
{
  int              ret         = OK;
  struct timespec  ts;
  struct mfs_mn_s  mn;
  const mfs_t      sz          = sizeof(struct mfs_mn_s) - sizeof(mn.pg);
  char             buf[sz + 1];

  mn = MFS_MN(sb);

  clock_gettime(CLOCK_REALTIME, &ts);

  if (mn.mblk_idx == MFS_PGINBLK(sb))
    {
      /* New blocks have been already allocated by the journal. */

      mn.mblk_idx = 0;
      mn.pg       = MFS_BLK2PG(sb, blk1);
    }

  mn.ts        = ts;
  mn.root_sz   = new_loc->sz;
  mn.root_ctz  = new_loc->ctz;
  mn.root_mode = 0777 | S_IFDIR;

  /* TODO: Root timestamps. */

  /* Serialize. */

  ser_mn(mn, buf);

  ret = mfs_write_page(sb, buf, sz, MFS_BLK2PG(sb, blk1) + mn.mblk_idx, 0);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  ret = mfs_write_page(sb, buf, sz, MFS_BLK2PG(sb, blk2) + mn.mblk_idx, 0);
  if (predict_false(ret < 0))
    {
      goto errout;
    }

  mn.mblk_idx++;
  MFS_MN(sb) = mn;

errout:
  return ret;
}
