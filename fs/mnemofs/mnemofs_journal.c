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
 ****************************************************************************/

/* Journal on-flash has a structure like:
 * struct mfs_jrnl_node {
 *   union {
 *     struct {
 *       char magic[7];
 *       uint8_t n_blocks;
 *       char index_array[ceil((n_blocks * sizeof(mfs_t))/pg_sz)],
 *       writeable_area[pg_sz - (ceil((n_blocks * sizeof(mfs_t))/pg_sz) + 1)]
 *     }, // First block.
 *     char writeable_area[pg_sz];
 *     char masterblocks[pg_sz]; // Last two blocks.
 *   }
 * };
 *
 */

/* TODO: On-flash offsets for the journal structure (index array, n_blks,
 * etc.)
 */

/* Journal will have `sb->jrnl_nblks + 2` blocks.
 *
 * The first block will start with a 7 byte-long magic number that identifies
 * a journal of mnemofs.
 *
 * Then another byte will be used to write the number of blocks in the
 * journal.
 *
 * This is done so that, if the journal is found during mount, it will be
 * quick to point to the master node as well.
 * NOTE: This count will NOT include the 2 master blocks.
 *
 * Then the next `(sb->jrnl_nblks + 2) * sizeof(mfs_t)` bytes will be used to
 * store an array of block numbers. These block numbers are of each of the
 * blocks of the journal. This is called the journal index array. If the
 * array extends to be partially occupying a page, then the rest of the page
 * is left unused.
 *
 * Then all the rest of the pages that are part of the journal are writeable
 * for the logs except the master blocks.
 * Advantages of journal index array over a singly linked list:
 *   - Multiple block numbers can be stored together. In singly linked list,
 *     there was only one block number in the last page of the block, which
 *     wasted a lot of space.
 *   - Page caching can cache the array as it is used often, and then
 *     traversal would be quicker.
 *   - While mounting, if journal is reached first during scan of the device,
 *     to get the root node, all that is required is to scan the second byte
 *     to get the array size, and then jump to the array, get the master
 *     block locations, find the most recent master block and then we have
 *     the master node. This removes the problem of reaching the master node
 *     through linked list traversal, which would be slower due to less
 *     probability of caching.
 *   - The entire journal moves together, so there is no reason for linked
 *     list except to not deal with the contrainst of having contiguous block
 *     problems like uneven wear which might lead to bad blocks right inside
 *     the journal among other things, etc.
 */

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

struct mfs_jrnl_log
{
  mfs_t size;
  mfs_t depth;
  struct mfs_ctz_store_s new;
  struct mfs_ctz_store_s *path; /* path has depth no. of elements */
  char hash;
  char magic[sizeof(MFS_JRNL_MAGIC)];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* NOTE: Journal assumes no page is less than 16B in size for its
 *        initialization.
 *        And assumes all page sizes are multiples of 4B.
 */

static int jrnl_off2details(FAR const struct mfs_sb_info * const sb,
                            mfs_t off, mfs_t *blk, mfs_t *pg, mfs_t *pgoff);
static mfs_t inline mfs_blkremsz(FAR const struct mfs_sb_info * const sb,
                                  mfs_t pg, mfs_t pgoff);

static char * ser_jrnl_log(const struct mfs_jrnl_log * const log,
                          char * out);
static const char * deser_jrnl_log(const char * const in,
                                  struct mfs_jrnl_log * const log);
static uint8_t jrnl_log_hash(const struct mfs_jrnl_log * const log);
static mfs_t jrnl_info2logsz(const struct mfs_jrnl_info * const info);
struct mfs_jrnl_log jrnl_info2log(const struct mfs_jrnl_info *info);

struct mfs_jrnl_info jrnl_log2info(const struct mfs_jrnl_log *log);

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
 * Name: ser_jrnl_log
 *
 * Description:
 *   Serialize a log to store it on flash.
 *
 * Input Parameters:
 *   log - Journal log.
 *   out - Output buffer to serialize to.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

static char * ser_jrnl_log(const struct mfs_jrnl_log * const log, char * out)
{
  mfs_t i = 0;

  out = mfs_ser_mfs(log->size, out);
  out = mfs_ser_mfs(log->depth, out);
  out = mfs_ser_ctz_store(&log->new, out);
  for (i = 0; i < log->depth; i++)
    {
      out = mfs_ser_ctz_store(&log->path[i], out);
    }

  out = mfs_ser_8(log->hash, out);
  out = mfs_ser_str(log->magic, sizeof(MFS_JRNL_MAGIC), out);
  return out;
}

/****************************************************************************
 * Name: deser_jrnl_log
 *
 * Description:
 *   Serialize a log to store it on flash.
 *
 * Input Parameters:
 *   log - Journal log.
 *   out - Output buffer to serialize to.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 * Assumptions/Limitations:
 *   Does not check checksum (hash) match.
 *
 ****************************************************************************/

static const char * deser_jrnl_log(const char * const in,
                                  struct mfs_jrnl_log * const log)
{
  mfs_t idx = 0;
  const char * i = in;

  i = mfs_deser_mfs(i, &log->size);
  i = mfs_deser_mfs(i, &log->depth);
  i = mfs_deser_ctz_store(i, &log->new);

  for (idx = 0; idx < log->depth; idx++)
    {
      i = mfs_deser_ctz_store(i, &log->path[idx]);
    }

  i = mfs_deser_8(i, &log->hash);
  i = mfs_deser_str(i, log->magic, sizeof(MFS_JRNL_MAGIC));
  return i;
}

/****************************************************************************
 * Name: jrnl_log_hash
 *
 * Description:
 *   Get hash of the members of log representation that come before hash.
 *   Calculation of hash of a log is done by assuming the hash field will
 *   be 0. It is also calculated by XOR of hash of all members in it. For
 *   array members, the hash is computed as the XOR of individual elements
 *   in it.
 *
 * Input Parameters:
 *   log - Journal log.
 *
 * Returned Value:
 *   Hash of the log.
 *
 ****************************************************************************/

static uint8_t jrnl_log_hash(const struct mfs_jrnl_log * const log)
{
  int i = 0;
  uint8_t hash = 0;

  hash ^= mfs_arrhash((char *) &log->size, sizeof(log->size));
  hash ^= mfs_arrhash((char *) &log->depth, sizeof(log->depth));
  hash ^= mfs_arrhash((char *) &log->new, sizeof(log->new));

  for (i = 0; i < log->depth; i++)
    {
      hash ^= mfs_arrhash((char *) &log->path[i], sizeof(log->path[i]));
    }

  hash ^= mfs_arrhash((char *) &log->magic, sizeof(MFS_JRNL_MAGIC));
  return hash;
}

/****************************************************************************
 * Name: jrnl_info2logsz
 *
 * Description:
 *   Get total size of log equivalent from a jouranl info.
 *
 * Input Parameters:
 *   info - Journal info from memory.
 *
 * Returned Value:
 *   Size of the log representation.
 *
 ****************************************************************************/

static mfs_t jrnl_info2logsz(const struct mfs_jrnl_info * const info)
{
  /* TODO */

  return 0;
}

/****************************************************************************
 * Name: jrnl_info2log
 *
 * Description:
 *   Get a log equivalent from a jouranl info.
 *
 * Input Parameters:
 *   info - Journal info from memory.
 *
 * Returned Value:
 *   The log representation.
 *
 ****************************************************************************/

struct mfs_jrnl_log jrnl_info2log(const struct mfs_jrnl_info *info)
{
  struct mfs_jrnl_log log;
  log.size  = jrnl_info2logsz(info);
  log.depth = info->depth;
  log.new   = info->new;
  log.path  = info->path;
  log.hash  = jrnl_log_hash(&log);
  memcpy(log.magic, MFS_JRNL_MAGIC, sizeof(MFS_JRNL_MAGIC));
  return log;
}

/****************************************************************************
 * Name: jrnl_log2info
 *
 * Description:
 *   Get an info equivalent from a retrieved jouranl log.
 *
 * Input Parameters:
 *   log - Retrieved journal log from the flash.
 *
 * Returned Value:
 *   The info representation.
 *
 ****************************************************************************/

struct mfs_jrnl_info jrnl_log2info(const struct mfs_jrnl_log *log)
{
  struct mfs_jrnl_info info;

  info.depth = log->depth;
  info.new = log->new;
  info.path = log->path;
  list_initialize(&info.list);
  return info;
}

/****************************************************************************
 * Name: jrnl_off2details
 *
 * Description:
 *   Given an offset in the journal (from the very start), it will provide
 *   the on-flash details of the offset.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   off   - Journal offset.
 *   blk   - Block number of the on-flash location of the offset.
 *   pg    - Page number of the on-flash location of the offset.
 *   pgoff - Page offset in the page of the on-flash location of the offset.
 *
 * Returned Value:
 *   0        - OK
 *   -EINVAL  - Out of bounds offset.
 *
 ****************************************************************************/

static int jrnl_off2details(FAR const struct mfs_sb_info * const sb,
                            mfs_t off, mfs_t *blk, mfs_t *pg, mfs_t *pgoff)
{
  mfs_t tmp;

  if (sb->blk_sz * sb->j_nblks <= off)
    {
      return -EINVAL;
    }

  tmp     = off / sb->blk_sz;
  *blk    = sb->j_state.idxarr[tmp];
  tmp     = (off % sb->blk_sz) / sb->pg_sz;
  *pg     = MFS_BLK2PG(sb, *blk) + tmp;
  *pgoff  = off % sb->pg_sz;
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Mount */

int mfs_jrnl_fmt(FAR struct mfs_sb_info * const sb)
{
  int ret = OK;
  int i;
  mfs_t tmp;
  mfs_t wr_s_pg;                      /* Start page of writeable area */
  mfs_t *idxarr        = NULL;
  char *buf            = NULL;
  uint8_t idxarr_idx   = 0;
  const uint8_t n_blks = sb->j_nblks; /* Not counting master blocks. */
  const int idxarr_sz  = (n_blks + 2) * sizeof(mfs_t);
  const int buf_sz     = idxarr_sz + 8;

  idxarr = kmm_zalloc(idxarr_sz);
  if (!idxarr)
    {
      ret = -ENOMEM;
      goto errout;
    }

  buf    = kmm_zalloc(buf_sz);
  if (!buf)
    {
      ret = -ENOMEM;
      goto errout_with_idxarr;
    }

  nxmutex_lock(&sb->fs_lock);
  for (idxarr_idx = 0; idxarr_idx < n_blks + 2; idxarr_idx++)
    {
      tmp = mfs_get_blk(sb);
      if (tmp == 0)
        {
          ret = -ENOMEM;
          goto errout_with_lock;
        }

      idxarr[idxarr_idx] = tmp;
    }

  memcpy(buf, MFS_JRNL_MAGIC, 7);
  memcpy(buf + 7, &idxarr_idx, 1);
  memcpy(buf + 8, idxarr, idxarr_sz);

  /* Handles the case where the block size may be smaller than the buffer. */

  if (predict_true(sb->blk_sz >= buf_sz))
    {
      mfs_write_data(sb, buf, buf_sz, MFS_BLK2PG(sb, idxarr[0]), 0);
      wr_s_pg = MFS_BLK2PG(sb, idxarr[0]) +
                ((buf_sz + (sb->pg_sz - 1)) / sb->pg_sz); /* ceil */
    }
  else
    {
      /* Split between journal blocks if the array is bigger than a block. */

      tmp         = 0;
      i           = 0;
      wr_s_pg     = MFS_BLK2PG(sb, idxarr[i]);
      while (i < n_blks)
        {
          if (predict_true(tmp + sb->blk_sz < buf_sz))
            {
              tmp += mfs_write_data(sb, buf + tmp, sb->blk_sz, wr_s_pg++, 0);
            }
          else
            {
              tmp += mfs_write_data(sb, buf + tmp, buf_sz - tmp,
                                    wr_s_pg++, 0);
              break;
            }

          if (wr_s_pg - idxarr[i] == sb->pg_in_blk)
            {
              i++;
              wr_s_pg = MFS_BLK2PG(sb, idxarr[i]);
            }
        }

      if (wr_s_pg - idxarr[i] == sb->pg_in_blk)
        {
          i++;
          wr_s_pg = MFS_BLK2PG(sb, idxarr[i]);
        }

      /* TODO: Asserts. wr_s_pg needs to work properly. Also, at this point,
       * it needs to point to a valid location where writes can be made. If
       * not such location is found, errout out before this point.
       */

      /* If this is true, the journal has not enough space. */

      if (i == n_blks && tmp < buf_sz)
        {
          ret = -ENOMEM;
          goto errout_with_lock;
        }
    }

  kmm_free(buf);
  sb->j_state.n_blks = n_blks;
  sb->j_state.idxarr = idxarr; /* Ownership transferred to sb */
  sb->j_state.s_off  = buf_sz;
  sb->j_state.r_off  = buf_sz;
  sb->j_state.w_off  = buf_sz;

  nxmutex_unlock(&sb->fs_lock);
  return ret;

errout_with_lock:
  nxmutex_unlock(&sb->fs_lock);

  kmm_free(buf);

errout_with_idxarr:
  kmm_free(idxarr);

errout:
  return ret;
}

/* Read and initialize journal from the flash. */

/* Provides information for the location of the latest master node. */

int mfs_jrnl_init(FAR struct mfs_sb_info * const sb, mfs_t blk,
                  mfs_t *m_node)
{
  char magic[7];
  int idxarr_sz;
  int buf_sz;
  uint8_t i;
  mfs_t tmp;
  mfs_t pg;
  int ret         = OK;
  char *buf       = NULL;

  memset(&sb->j_state, 0, sizeof(sb->j_state));

  nxmutex_lock(&sb->fs_lock);

  mfs_read_data(sb, magic, 7, MFS_BLK2PG(sb, blk), 0);
  if (!strcmp(magic, MFS_JRNL_MAGIC))
    {
      ret = -EINVAL;
      goto errout_with_lock;
    }

  mfs_read_data(sb, (char *) &sb->j_state.n_blks, 1, MFS_BLK2PG(sb, blk), 7);
  idxarr_sz = (sb->j_state.n_blks + 2) * sizeof(mfs_t);
  buf_sz    = idxarr_sz + 8;

  sb->j_state.idxarr = kmm_zalloc(idxarr_sz);
  if (!sb->j_state.idxarr)
    {
      ret = -ENOMEM;
      goto errout_with_lock;
    }

  buf = kmm_zalloc(buf_sz);
  if (!buf)
    {
      ret = -ENOMEM;
      goto errout_with_idxarr;
    }

  if (predict_true(sb->blk_sz >= buf_sz))
    {
        mfs_read_data(sb, buf, buf_sz, MFS_BLK2PG(sb, blk), 0);

        sb->j_state.idxarr = (mfs_t *) (buf + 8);
    }
  else
    {
      i   = 0;
      tmp = 0;
      tmp += mfs_read_data(sb, buf, sb->blk_sz, MFS_BLK2PG(sb, blk), 0);
      sb->j_state.idxarr = (mfs_t *) (buf + 8);

      i++;
      while (tmp < buf_sz)
        {
          pg = MFS_BLK2PG(sb, sb->j_state.idxarr[i]);
          if (predict_true(tmp + sb->blk_sz < buf_sz))
            {
              mfs_read_data(sb, buf, sb->blk_sz, pg, 0);
            }
          else
            {
              tmp = mfs_read_data(sb, buf, buf_sz - tmp, pg, 0);
              pg  += ((tmp + (sb->pg_sz - 1)) / sb->pg_sz);
              break;
            }

          i++;
        }
    }

  sb->j_state.s_off = buf_sz;
  sb->j_state.r_off = buf_sz;
  sb->j_state.w_off = buf_sz;

  /* TODO: Need to traverse the journal from end to get sb->j_state.w_* */

  nxmutex_unlock(&sb->fs_lock);
  return ret;

errout_with_idxarr:
  kmm_free(sb->j_state.idxarr);
  sb->j_state.idxarr = NULL;

errout_with_lock:
  nxmutex_unlock(&sb->fs_lock);

  return ret;
}

/* Traversal */

/* Reset journal state to pre-traversal. */

void mfs_jrnl_statereset(FAR struct mfs_sb_info * const sb)
{
  nxmutex_lock(&sb->fs_lock);
  sb->j_state.r_off = sb->j_state.s_off;
  nxmutex_unlock(&sb->fs_lock);
}

/* Advance the read pointer to the next log. -1 if reached writable
 * pointer.
 */

int mfs_jrnl_radv(FAR struct mfs_sb_info * const sb,
                  FAR struct mfs_jrnl_info * info)
{
  int ret;
  mfs_t blk;
  mfs_t pg;
  mfs_t pgoff;
  mfs_t sz;
  mfs_t rem_sz;
  mfs_t rem_blk_sz;
  char *tmp;
  char *buf;
  struct mfs_jrnl_log log;

  ret = OK;
  buf = NULL;

  nxmutex_lock(&sb->fs_lock);

  ret = jrnl_off2details(sb, sb->j_state.r_off, &blk, &pg, &pgoff);
  if (ret != 0)
    {
      goto errout;
    }

  mfs_read_page(sb, (char *) &sz, sizeof(mfs_t), pg, pgoff);

  if (predict_true(info != NULL))
    {
      buf = kmm_zalloc(sz);
      if (!buf)
        {
          ret = -ENOMEM;
          goto errout;
        }

      rem_sz = sz;
      tmp    = buf;

      while (1)
        {
          jrnl_off2details(sb, sb->j_state.w_off, &blk, &pg, &pgoff);
          rem_blk_sz        = mfs_blkremsz(sb, pg, pgoff);
          mfs_read_data(sb, tmp, MIN(rem_blk_sz, rem_sz), pg, pgoff);
          tmp               += MIN(rem_blk_sz, rem_sz);
          sb->j_state.w_off += MIN(rem_blk_sz, rem_sz);

          if (predict_false(rem_blk_sz > rem_sz))
            {
              break;
            }
        }

      deser_jrnl_log(buf, &log);
      *info = jrnl_log2info(&log);

      kmm_free(buf);
    }

  if (predict_false(sb->j_state.r_off + sz > sb->j_state.w_off))
    {
      ret = -1;
      goto errout;
    }

  sb->j_state.r_off += sz;

errout:
  nxmutex_unlock(&sb->fs_lock);
  return ret;
}

/* Write */

/* Can only advance the write pointer by writing a log. */

int mfs_jrnl_wadv(FAR struct mfs_sb_info * const sb,
                  FAR const struct mfs_jrnl_info * const info)
{
  int ret;
  char *tmp;
  char *buf;
  mfs_t blk;
  mfs_t pg;
  mfs_t pgoff;
  mfs_t rem_sz;
  mfs_t rem_blk_sz;
  struct mfs_jrnl_log log;

  buf = NULL;
  ret = OK;

  if (info == NULL)
    {
      ret = -1;
      goto errout;
    }

  rem_sz = jrnl_info2logsz(info);
  buf    = kmm_zalloc(rem_sz);
  if (!buf)
    {
      ret = -ENOMEM;
      goto errout;
    }

  log = jrnl_info2log(info);
  ser_jrnl_log(&log, buf);

  tmp = buf;

  nxmutex_lock(&sb->fs_lock);

  while (1)
    {
      jrnl_off2details(sb, sb->j_state.w_off, &blk, &pg, &pgoff);
      rem_blk_sz        = mfs_blkremsz(sb, pg, pgoff);
      mfs_write_data(sb, tmp, MIN(rem_blk_sz, rem_sz), pg, pgoff);
      tmp               += MIN(rem_blk_sz, rem_sz);
      sb->j_state.w_off += MIN(rem_blk_sz, rem_sz);

      if (predict_false(rem_blk_sz > rem_sz))
        {
          break;
        }
    }

  kmm_free(buf);

  nxmutex_unlock(&sb->fs_lock);

errout:
  return ret;
}

int mfs_jrnl_nwadv(FAR struct mfs_sb_info * const sb,
                  FAR struct list_node list)
{
  int tmp;
  int ret                    = 0;
  struct mfs_jrnl_info *info = NULL;

  list_for_every_entry(&list, info, struct mfs_jrnl_info, list)
    {
      tmp = mfs_jrnl_wadv(sb, info);
      if (tmp < 0)
        {
          return ret;
        }

      ret++;
    }

  return ret;
}

void mfs_jrml_updatectz(FAR struct mfs_sb_info * const sb,
                        FAR struct mfs_ctz_store_s * const path,
                        const mfs_t depth)
{
  struct mfs_jrnl_info info;
  struct mfs_ctz_store_s cur = path[depth - 1];

  nxmutex_lock(&sb->fs_lock);

  mfs_jrnl_statereset(sb); /* Just to be sure */

  while (mfs_jrnl_radv(sb, &info) > 0)
    {
      if (info.depth == depth && info.path[depth - 1].pg_e == cur.pg_e &&
          info.path[depth - 1].idx_e == cur.idx_e)
        {
          cur = info.new;
        }
    }

  mfs_jrnl_statereset(sb);
  path[depth - 1].idx_e = cur.idx_e;
  path[depth - 1].pg_e  = cur.pg_e;

  nxmutex_unlock(&sb->fs_lock);
}
