/****************************************************************************
 * fs/mnemofs/mnemofs.h
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

#ifndef __FS_MNEMOFS_MNEMOFS_H
#define __FS_MNEMOFS_MNEMOFS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <debug.h>
#include <nuttx/fs/fs.h>
#include <nuttx/list.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MFS_JRNL_MAGIC  "-mfs!j!-"
#define MFS_MN_MAGIC    "-mfs!m!-"

#define MFS_CEILDIVIDE(num, denom) (((num) + ((denom) - 1)) / (denom))
#define MFS_UPPER8(num)            (((num) + 7) & (-8))

#define MFS_BLK2PG(sb, blk)        ((blk) << (sb)->log_pg_in_blk)
#define MFS_PG2BLK(sb, pg)         ((pg) >> (sb)->log_pg_in_blk)
#define MFS_PG2BLKPGOFF(sb, pg)    ((pg) % (1 << (sb)->log_pg_in_blk))

#define MFS_LOCK(sb)               ((sb)->fs_lock)
#define MFS_PGSZ(sb)               ((sb)->pg_sz)
#define MFS_LRU(sb)                ((sb)->lru)
#define MFS_BLKSZ(sb)              ((sb)->blk_sz)
#define MFS_MN(sb)                 ((sb)->mn)
#define MFS_JRNL(sb)               ((sb)->j_state)
#define MFS_LOGPGSZ(sb)            (MFS_CEILDIVIDE((sb)->log_pg_sz, 8))
#define MFS_PGINBLK(sb)            ((sb)->pg_in_blk)
#define MFS_MTD(sb)                ((sb)->drv->u.i_mtd)
#define MFS_RWBUF(sb)              ((sb)->rw_buf)
#define MFS_BA(sb)                 ((sb)->ba_state)
#define MFS_NBLKS(sb)              ((sb)->n_blks)
#define MFS_OFILES(sb)             ((sb)->of)
#define MFS_FLUSH(sb)              ((sb)->flush)
#define MFS_NPGS(sb)               (MFS_NBLKS(sb) * MFS_PGINBLK(sb))

#define MFS_HASHSZ                 16
#define MFS_CTZ_SZ(l)              ((l)->sz)
#define MFS_DIRENTSZ(dirent)       ((2 * 2) + 4 + (16 * 3) + 8 + 1 \
                                    + (dirent)->namelen)

#define MFS_JRNL_LIM(sb)           (MFS_JRNL(sb).n_blks / 2)
#define MFS_TRAVERSE_INITSZ        8

#define MFS_LOG(fn, fmt, ...)          finfo("[mnemofs | " fn "] " fmt, ##__VA_ARGS__)
#ifdef CONFIG_MNEMOFS_EXTRA_DEBUG
#define MFS_EXTRA_LOG(fn, fmt, ...)    MFS_LOG(fn, fmt, ##__VA_ARGS__)
#else
#define MFS_EXTRA_LOG(fmt, ...)    { }
#endif
#define MFS_STRLITCMP(a, lit)      strncmp(a, lit, strlen(lit))

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef uint32_t  mfs_t;

enum
{
  MFS_PG_USED,
  MFS_PG_FREE,
  MFS_BLK_BAD,
  MFS_BLK_ERASABLE,
  MFS_BLK_FREE,
  MFS_BLK_USED,
};

enum MFS_PATH_FLAGS
{
  MFS_ISDIR   = (1 << 0),  /* Path is a directory. */
  MFS_ISFILE  = (1 << 1),  /* Path is a file. */
  MFS_EXIST   = (1 << 2),  /* Path Exists */
  MFS_FINPATH = (1 << 3),  /* File in midele of path before bottom most
                            * child. Not reachable.
                            */
  MFS_P_EXIST = (1 << 4),  /* Parent of the bottom most element exists. */
  MFS_P_ISDIR = (1 << 5),  /* Parent is a directory. */
};

struct mfs_ctz_s
{
  mfs_t pg_e;
  mfs_t idx_e;
};

struct mfs_path_s
{
  struct mfs_ctz_s ctz;
  mfs_t            off;
  mfs_t            sz;
};

struct mfs_ba_state_s
{
  mfs_t       s_blk;         /* Start block */
  mfs_t       c_pg;          /* Current page */
  FAR mfs_t   *k_del;        /* Delete counter for blocks. */
  size_t      n_bmap_upgs;
  FAR uint8_t *bmap_upgs;    /* Bitmap of used pages. */
};

struct mfs_mn_s
{
  mfs_t            pg;            /* Only mblk1's pg will be used here. */
  mfs_t            jrnl_blk;      /* Start of journal. */
  mfs_t            mblk_idx;      /* Index for next MN entry inside blk. */
  struct mfs_ctz_s root_ctz;
  mfs_t            root_sz;
  struct timespec  ts;
  mode_t           root_mode;
  struct timespec  root_st_atim;
  struct timespec  root_st_ctim;
  struct timespec  root_st_mtim;
};

struct mfs_jrnl_state_s
{
  mfs_t    jrnl_start;
  mfs_t    mblk1;
  mfs_t    mblk2;
  mfs_t    n_logs;
  mfs_t    log_cpg;       /* Current (last) page */
  mfs_t    log_cblkidx;   /* Current (last) block index. */
  mfs_t    log_spg;       /* First log's page */
  mfs_t    log_sblkidx;   /* First jrnl blk index. TODO: jrnlarr > 1 blk. */
  mfs_t    jrnlarr_pg;
  mfs_t    jrnlarr_pgoff;
  uint16_t n_blks;        /* TODO: Does not include the master node. */
};

struct mfs_sb_s
{
  FAR uint8_t             *rw_buf;
  FAR struct inode        *drv;
  mutex_t                 fs_lock;
  mfs_t                   sb_blk;        /* Block number of the superblock */
  mfs_t                   pg_sz;
  uint8_t                 log_pg_sz;
  mfs_t                   blk_sz;
  uint8_t                 log_blk_sz;
  mfs_t                   n_blks;
  uint8_t                 log_n_blks;
  uint16_t                pg_in_blk;
  uint8_t                 log_pg_in_blk;
  struct mfs_mn_s         mn;            /* Master Node */
  struct mfs_jrnl_state_s j_state;       /* Journal State */
  struct mfs_ba_state_s   ba_state;      /* Block Allocator State */
  struct list_node        lru;
  struct list_node        of;            /* open files. */
  bool                    flush;
};

/* This is for *dir VFS methods. */

struct mfs_fsdirent_s
{
  struct fs_dirent_s    base; /* VFS directory structure */
  uint8_t               idx;  /* This only goes from 0 for ., 1 for .. and
                               * 2 for others.
                               */
  FAR struct mfs_pitr_s *pitr;
  FAR struct mfs_path_s *path;
  mfs_t                 depth;
};

/* LRU Delta */

struct mfs_delta_s
{
  struct list_node list;
  mfs_t            off;
  mfs_t            n_b;
  FAR char         *upd;
};

/* LRU Node */

struct mfs_node_s
{
  struct list_node      list;
  struct list_node      delta;
  mfs_t                 n_list;
  mfs_t                 depth;
  mfs_t                 sz;
  mfs_t                 range_min;
  mfs_t                 range_max;
  struct timespec       st_mtim;
  struct timespec       st_atim;
  struct timespec       st_ctim;
  FAR struct mfs_path_s *path;
};

/* Common Part Open File Descriptor */

struct mfs_ocom_s
{
  bool                  new_ent;
  mfs_t                 sz;       /* Current file size. */
  mfs_t                 off;
  mfs_t                 depth;
  uint8_t               refcount;
  int                   oflags;
  FAR struct mfs_path_s *path;
};

/* Open part for file descriptors. */

struct mfs_ofd_s
{
  struct list_node      list;
  FAR struct mfs_ocom_s *com;
};

struct mfs_dirent_s
{
  uint16_t         name_hash;  /* Should be at start to improve efficiency. */
  uint16_t         mode;
  mfs_t            sz;
  struct timespec  st_atim;    /* Time of last access */
  struct timespec  st_mtim;    /* Time of last modification */
  struct timespec  st_ctim;    /* Time of last status change */
  struct mfs_ctz_s ctz;
  uint8_t          namelen;
  char             name[];
};

/* Parent iterator */

struct mfs_pitr_s
{
  struct mfs_path_s p;     /* Parent's path representation */
  mfs_t             depth;
  mfs_t             c_off; /* Current iteration offset. */
};

/* TODO: depth >= 1 */

/* IMP TODO: sizeof(x) != size of buffer required to store it. Need to fix. */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Inline Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mfs_blkremsz
 *
 * Description:
 *   Given a page and a page offset, it returns the bytes left in the entire
 *   block after the offset location.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   pg    - Page number.
 *   pgoff - Page offset.
 *
 * Returned Value:
 *   Bytes left in the block.
 *
 ****************************************************************************/

static mfs_t inline mfs_blkremsz(FAR const struct mfs_sb_s * const sb,
                                 mfs_t pg, mfs_t pgoff)
{
  return MFS_BLKSZ(sb) - (MFS_PG2BLKPGOFF(sb, pg) * sb->pg_sz + pgoff);
}

static inline mfs_t mfs_ctz(const uint32_t x)
{
  if (predict_false(x == 0))
  {
/* Special case, since we're using this for the CTZ skip list. The 0th
 * block has no pointers.
 */

    return 0;
  }

#if defined(__GNUC__)
  return __builtin_ctz(x);
#else
  uint32_t c;

/* Credits:
 * http://graphics.stanford.edu/~seander/bithacks.html#ZerosOnRightBinSearch
 */

  if (x & 0x1)
  {
    /* special case for odd x (assumed to happen half of the time) */

    c = 0;
  }
  else
  {
    c = 1;
    if ((x & 0xffff) == 0)
    {
      x >>= 16;
      c += 16;
    }
    if ((x & 0xff) == 0)
    {
      x >>= 8;
      c += 8;
    }
    if ((x & 0xf) == 0)
    {
      x >>= 4;
      c += 4;
    }
    if ((x & 0x3) == 0)
    {
      x >>= 2;
      c += 2;
    }
    c -= x & 0x1;
  }
  return c;
#endif
}

static inline mfs_t mfs_clz(const uint32_t x)
{
  if (predict_false(x == UINT32_MAX))
  {
/* Special case, since we're using this for the CTZ skip list. The 0th
 * block has no pointers.
 */

    return 0;
  }

#if defined(__GNUC__)
  return __builtin_clz(x);
#else
  return 0; /* TODO */
#endif
}

static inline mfs_t mfs_popcnt(mfs_t x)
{
#if defined(__GNUC__)
  return __builtin_popcount(x);
#else
/* Can be found at:
 * http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetKernighan
 */

  mfs_t c;
  for (c = 0; x; c++)
  {
    x &= x - 1;
  }

#endif
}

static inline void MFS_EXTRA_LOG_DIRENT(FAR const struct mfs_dirent_s * const
                                        dirent)
{
  MFS_EXTRA_LOG("EXTRA_LOG_DIRENT", "Direntry details.");
  MFS_EXTRA_LOG("EXTRA_LOG_DIRENT", "\tDirent location %p", dirent);
  MFS_EXTRA_LOG("EXTRA_LOG_DIRENT", "\tMode is %" PRIu16, dirent->mode);
  MFS_EXTRA_LOG("EXTRA_LOG_DIRENT", "\tName is \"%.*s\"", dirent->namelen,
                dirent->name);
  MFS_EXTRA_LOG("EXTRA_LOG_DIRENT", "\tNamelen is %" PRIu32,
                dirent->namelen);
  MFS_EXTRA_LOG("EXTRA_LOG_DIRENT", "\tName Hash is %" PRIu16,
                dirent->name_hash);
  MFS_EXTRA_LOG("EXTRA_LOG_DIRENT", "\tSize is %" PRIu16,
                dirent->sz);

  /* TODO: Timespecs */
}

static inline void MFS_EXTRA_LOG_FSDIRENT(FAR const struct mfs_fsdirent_s *
                                          const fsdirent)
{
  MFS_EXTRA_LOG("EXTRA_LOG_FSDIRENT", "FS Direntry details.");
  MFS_EXTRA_LOG("EXTRA_LOG_FSDIRENT", "\tDirentry depth %" PRIu32,
                fsdirent->depth);
  MFS_EXTRA_LOG("EXTRA_LOG_FSDIRENT", "\tRead index %" PRIu32,
                fsdirent->idx);
  MFS_EXTRA_LOG("EXTRA_LOG_FSDIRENT", "\tPath %p.", fsdirent->path);
  MFS_EXTRA_LOG("EXTRA_LOG_FSDIRENT", "\tPitr %p.", fsdirent->pitr);
  MFS_EXTRA_LOG("EXTRA_LOG_FSDIRENT", "\tDepth %" PRIu32, fsdirent->depth);
}

static inline void MFS_EXTRA_LOG_PITR(FAR const struct mfs_pitr_s * const
                                      pitr)
{
  MFS_EXTRA_LOG("EXTRA_LOG_PITR", "Pitr details.");
  MFS_EXTRA_LOG("EXTRA_LOG_PITR", "\tDepth %" PRIu32, pitr->depth);
  MFS_EXTRA_LOG("EXTRA_LOG_PITR", "\tCurrent Offset %" PRIu32, pitr->c_off);
  MFS_EXTRA_LOG("EXTRA_LOG_PITR", "\tParent CTZ (%" PRIu32 ", %" PRIu32 ")",
                pitr->p.ctz.idx_e, pitr->p.ctz.pg_e);
  MFS_EXTRA_LOG("EXTRA_LOG_PITR", "\tParent Size %" PRIu32, pitr->p.sz);
  MFS_EXTRA_LOG("EXTRA_LOG_PITR", "\tParent Offset %" PRIu32, pitr->p.off);
}

static inline void MFS_EXTRA_LOG_MN(FAR const struct mfs_mn_s * const mn)
{
  MFS_EXTRA_LOG("EXTRA_LOG_MN", "Master node details.");
  MFS_EXTRA_LOG("EXTRA_LOG_MN", "\tFirst journal block is %" PRIu32,
                mn->jrnl_blk);
  MFS_EXTRA_LOG("EXTRA_LOG_MN", "\tNext MN entry index %" PRIu32,
                mn->mblk_idx);
  MFS_EXTRA_LOG("EXTRA_LOG_MN", "\tRoot CTZ (%" PRIu32 ", %" PRIu32 ")",
                mn->root_ctz.idx_e, mn->root_ctz.pg_e);
  MFS_EXTRA_LOG("EXTRA_LOG_MN", "\tRoot Size %" PRIu32, mn->root_sz);
  MFS_EXTRA_LOG("EXTRA_LOG_MN", "\tRoot Mode is %u.", mn->root_mode);

  /* TODO: Timespecs */
}

static inline void MFS_EXTRA_LOG_F(FAR struct mfs_ofd_s *f)
{
  MFS_EXTRA_LOG("EXTRA_LOG_F", "File structure details.");
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\tList details.");
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\tPrevious node %p.", f->list.prev);
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\tNext node %p.", f->list.next);
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\tCommon structure details.");
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\tDepth: %" PRIu32, f->com->depth);
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\tNew Entry: %s.",
                f->com->new_ent ? "true" : "false");
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\tOffset: %" PRIu32, f->com->off);
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\tFlags: 0x%x.", f->com->oflags);
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\tReference Counter: %" PRIu8,
                f->com->refcount);
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\tFile Size: %" PRIu32, f->com->sz);
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\tPath details.");
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\t\tOffset: %" PRIu32, f->com->path->off);
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\t\tSize: %" PRIu32, f->com->path->sz);
  MFS_EXTRA_LOG("EXTRA_LOG_F", "\t\t\tCTZ (%" PRIu32 ", %" PRIu32 ").",
                f->com->path->ctz.idx_e, f->com->path->ctz.pg_e);
}

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* mnemofs.c */

int mnemofs_flush(FAR struct mfs_sb_s *sb);

/* mnemofs_journal.c */

/****************************************************************************
 * Name: mfs_jrnl_init
 *
 * Description:
 *   Initializes journal when it's already formatted into the device.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   blk - First block of the journal.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   Does not initialize the master node.
 *
 ****************************************************************************/

int mfs_jrnl_init(FAR struct mfs_sb_s * const sb, mfs_t blk);

/****************************************************************************
 * Name: mfs_jrnl_fmt
 *
 * Description:
 *   Format a journal to the device.
 *
 * Input Parameters:
 *   sb   - Superblock instance of the device.
 *   blk1 - First master block for the journal.
 *   blk2 - Second master block for the journal.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   If blk1 == 0 and blk2 == 0, this means that this will also format in the
 *   master blocks. If this is not satisfied, the provided values will be
 *   taken to denote the master nodes.
 *
 *   Does not format the master node.
 *
 ****************************************************************************/

int mfs_jrnl_fmt(FAR struct mfs_sb_s * const sb, mfs_t *blk1, mfs_t *blk2,
                 FAR mfs_t *jrnl_blk);

/****************************************************************************
 * Name: mfs_jrnl_free
 *
 * Description:
 *   Free the journal.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 ****************************************************************************/

void mfs_jrnl_free(FAR struct mfs_sb_s * const sb);

/****************************************************************************
 * Name: mfs_jrnl_blkidx2blk
 *
 * Description:
 *   Gets the block number of a journal block at an index.
 *
 * Input Parameters:
 *   sb      - Superblock instance of the device.
 *   blk_idx - Index of the block who's block number to find.
 *
 * Returned Value:
 *   Block number.
 *
 * Assumptions/Limitations:
 *   Assumes valid index. This index can also include master blocks. Also
 *   assumes, for now, that the entire journal array can fit in the first
 *   block.
 *
 ****************************************************************************/

mfs_t mfs_jrnl_blkidx2blk(FAR const struct mfs_sb_s * const sb,
                          const mfs_t blk_idx);

/****************************************************************************
 * Name: mfs_jrnl_updatedinfo
 *
 * Description:
 *   Update the path information from the journal.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   path  - "Base state" path.
 *   depth - Path depth.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   This applies updates over data that is already gathered from the data
 *   section of the flash. The data section is the "base state" over which
 *   the updates in the journal are applied.
 *
 ****************************************************************************/

int mfs_jrnl_updatedinfo(FAR const struct mfs_sb_s * const sb,
                         FAR struct mfs_path_s * const path,
                         const mfs_t depth);

/****************************************************************************
 * Name: mfs_jrnl_wrlog
 *
 * Description:
 *   Write a log for LRU node when its popped from the LRU.
 *
 * Input Parameters:
 *   sb      - Superblock instance of the device.
 *   node    - LRU node.
 *   loc_new - New location of the CTZ skip list.
 *   sz_new  - New size of the CTZ skip list.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   When a node from LRU is flushed, it's written to the data in any space
 *   that is available according to the block allocator, and the new location
 *   is recorded as a log along with the old location. Any log of this same
 *   CTZ skip list in the journal will use this "snapshot" of the location,
 *   ie. the updated path will be used until another log of the same CTZ
 *   skip list is stored, after which the path will be updated again, and
 *   so on.
 *
 ****************************************************************************/

int mfs_jrnl_wrlog(FAR struct mfs_sb_s * const sb,
                   FAR const struct mfs_node_s *node,
                   const struct mfs_ctz_s loc_new, const mfs_t sz_new);

/****************************************************************************
 * Name: mfs_jrnl_flush
 *
 * Description:
 *   Flush the entire journal. This is the entry point of the entire flush
 *   operation and includes messing with the master node as well.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

int mfs_jrnl_flush(FAR struct mfs_sb_s * const sb);

/****************************************************************************
 * Name: mfs_jrnl_isempty
 *
 * Description:
 *   Check if the journal is empty.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Returned Value:
 *   True is the journal is empty, false otherwise.
 *
 ****************************************************************************/

bool mfs_jrnl_isempty(FAR const struct mfs_sb_s * const sb);

struct mfs_jrnl_log_s
{
  /* A field denoting the size of the below elements is added here on-flash.
   */

  mfs_t                  depth;
  mfs_t                  sz_new;
  struct mfs_ctz_s       loc_new;
  struct timespec        st_mtim_new;
  struct timespec        st_atim_new;
  struct timespec        st_ctim_new;
  FAR struct mfs_path_s *path;
  uint16_t               hash;
};

int mfs_jrnl_rdlog(FAR const struct mfs_sb_s *const sb,
                      FAR mfs_t *blkidx, FAR mfs_t *pg_in_blk,
                      FAR struct mfs_jrnl_log_s *log);

void mfs_jrnl_log_free(FAR const struct mfs_jrnl_log_s * const log);

/* mnemofs_blkalloc.c */

/****************************************************************************
 * Name: mfs_ba_init
 *
 * Description:
 *   Formats and initializes the block allocator.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Returned Value:
 *   0        - OK
 *   -ENOMEM  - No memory left.
 *
 ****************************************************************************/

int mfs_ba_fmt(FAR struct mfs_sb_s * const sb);

/****************************************************************************
 * Name: mfs_ba_init
 *
 * Description:
 *   Initializes the block allocator.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Returned Value:
 *   0        - OK
 *   -ENOMEM  - No memory left.
 *
 ****************************************************************************/

int mfs_ba_init(FAR struct mfs_sb_s * const sb);

/****************************************************************************
 * Name: mfs_ba_getpg
 *
 * Description:
 *   Returns an allocated page.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Returned Value:
 *   0    - No more space left
 *   > 0  - Page number
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

mfs_t mfs_ba_getpg(FAR struct mfs_sb_s * const sb);

/****************************************************************************
 * Name: mfs_ba_getblk
 *
 * Description:
 *   Returns an allocated block.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Returned Value:
 *   0    - No more space left
 *   > 0  - Block number
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

mfs_t mfs_ba_getblk(FAR struct mfs_sb_s * const sb);

/****************************************************************************
 * Name: mfs_ba_pgmarkdel
 *
 * Description:
 *   Mark a page as being ready for deletion.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *   pg - Page number.
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

void mfs_ba_pgmarkdel(FAR struct mfs_sb_s * const sb, mfs_t pg);

/****************************************************************************
 * Name: mfs_ba_blkmarkdel
 *
 * Description:
 *   Mark a block as being ready for deletion.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   blk - Block number.
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

void mfs_ba_blkmarkdel(FAR struct mfs_sb_s * const sb, mfs_t blk);

/****************************************************************************
 * Name: mfs_ba_delmarked
 *
 * Description:
 *   Delete all marked for deletion blocks.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

int mfs_ba_delmarked(FAR struct mfs_sb_s * const sb);

/****************************************************************************
 * Name: mfs_ba_markusedpg
 *
 * Description:
 *   Marked page as being used.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *   pg - Page number
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

void mfs_ba_markusedpg(FAR struct mfs_sb_s * const sb, mfs_t pg);

/****************************************************************************
 * Name: mfs_ba_markusedblk
 *
 * Description:
 *   Marked block as being used.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   blk - Block number
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

void mfs_ba_markusedblk(FAR struct mfs_sb_s * const sb, mfs_t blk);

/****************************************************************************
 * Name: mfs_ba_getavailpgs
 *
 * Description:
 *   Get number of available pages.
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *   pg - Page number
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

mfs_t mfs_ba_getavailpgs(FAR const struct mfs_sb_s * const sb);

/****************************************************************************
 * Name: mfs_ba_free
 *
 * Description:
 *   Free the block allocator
 *
 * Input Parameters:
 *   sb - Superblock instance of the device.
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

void mfs_ba_free(FAR struct mfs_sb_s * const sb);

/* mnemofs_rw.c */

/****************************************************************************
 * Name: mfs_isbadblk
 *
 * Description:
 *   Is a block bad.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   blk - Block Number
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

int mfs_isbadblk(FAR const struct mfs_sb_s * const sb, mfs_t blk);

/****************************************************************************
 * Name: mfs_markbadblk
 *
 * Description:
 *   Mark a block as bad.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   blk - Block Number
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

int mfs_markbadblk(FAR const struct mfs_sb_s * const sb, mfs_t blk);

/****************************************************************************
 * Name: mfs_write_page
 *
 * Description:
 *   Write a page.
 *
 * Input Parameters:
 *   sb      - Superblock instance of the device.
 *   data    - Buffer
 *   datalen - Length of buffer.
 *   pg      - Page number.
 *   pgoff   - Offset into the page.
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

ssize_t mfs_write_page(FAR const struct mfs_sb_s * const sb,
                       FAR const char *data, const mfs_t datalen,
                       const off_t page, const mfs_t pgoff);

/****************************************************************************
 * Name: mfs_read_page
 *
 * Description:
 *   Read a page.
 *
 * Input Parameters:
 *   sb      - Superblock instance of the device.
 *   data    - Buffer
 *   datalen - Length of buffer.
 *   pg      - Page number.
 *   pgoff   - Offset into the page.
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

ssize_t mfs_read_page(FAR const struct mfs_sb_s * const sb,
                      FAR char *data, const mfs_t datalen, const off_t page,
                      const mfs_t pgoff);

/****************************************************************************
 * Name: mfs_erase_blk
 *
 * Description:
 *   Erase a block.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   blk - Block Number
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

int mfs_erase_blk(FAR const struct mfs_sb_s * const sb, const off_t blk);

/****************************************************************************
 * Name: mfs_erase_nblks
 *
 * Description:
 *   Erase consecutive blocks.
 *
 * Input Parameters:
 *   sb  - Superblock instance of the device.
 *   blk - Block Number
 *
 * Assumptions/Limitations:
 *   This assumes a locked environment when called.
 *
 ****************************************************************************/

int mfs_erase_nblks(FAR const struct mfs_sb_s * const sb, const off_t blk,
                    const size_t n);

/* mnemofs_util.c */

/****************************************************************************
 * Name: mfs_arrhash
 *
 * Description:
 *   Returns an 8-bit hash of an entire array of data.
 *
 * Input Parameters:
 *   arr - Data array.
 *   len - Length of the array.
 *
 * Returned Value:
 *   16-bit hash of the array.
 *
 ****************************************************************************/

uint8_t mfs_arrhash(FAR const char *arr, ssize_t len);

/* TODO: Put below in place of above. */

uint16_t mfs_hash(FAR const char *arr, ssize_t len);

/****************************************************************************
 * Name: mfs_ser_8
 *
 * Description:
 *   Serialize a 8 bit type into output.
 *
 * Input Parameters:
 *   n   - 8 bit to serialize
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR char *mfs_ser_8(const uint8_t n, FAR char * const out);

/****************************************************************************
 * Name: mfs_deser_8
 *
 * Description:
 *   Deserialize a 8 bit type from input.
 *
 * Input Parameters:
 *   in - Input array from where to deserialize.
 *   n  - 8 bit to deserialize
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR const char *mfs_deser_8(FAR const char * const in, uint8_t *n);

/****************************************************************************
 * Name: mfs_ser_str
 *
 * Description:
 *   Serialize a string into output.
 *
 * Input Parameters:
 *   str - String to serialize
 *   len - Length of string
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR char *mfs_ser_str(FAR const char * const str, const mfs_t len,
                      FAR char * const out);

/****************************************************************************
 * Name: mfs_deser_str
 *
 * Description:
 *   Deserialize a string from intput.
 *
 * Input Parameters:
 *   in  - Intput array from where to deserialize.
 *   str - String to deserialize
 *   len - Length of string
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR const char *mfs_deser_str(FAR const char * const in,
                              FAR char * const str, const mfs_t len);

/****************************************************************************
 * Name: mfs_ser_mfs
 *
 * Description:
 *   Serialize a mfs_t type into output.
 *
 * Input Parameters:
 *   n   - mfs_t to serialize
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR char *mfs_ser_mfs(const mfs_t n, FAR char * const out);

/****************************************************************************
 * Name: mfs_deser_mfs
 *
 * Description:
 *   Deserialize a mfs_t type from input..
 *
 * Input Parameters:
 *   in - Input array from where to deserialize.
 *   n  - mfs_t to deserialize
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR const char *mfs_deser_mfs(FAR const char * const in,
                              FAR mfs_t * const n);

/****************************************************************************
 * Name: mfs_ser_ctz
 *
 * Description:
 *   Serialize a mfs_ctz_store_s type into output.
 *
 * Input Parameters:
 *   x   - mfs_ctz_s to serialize
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR char *mfs_ser_ctz(FAR const struct mfs_ctz_s * const x,
                      FAR char * const out);

/****************************************************************************
 * Name: mfs_deser_ctz
 *
 * Description:
 *   Deserialize a mfs_ctz_store_s type into output.
 *
 * Input Parameters:
 *   in - Input array from where to deserialize.
 *   x  - mfs_ctz_s to deserialize
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR const char *mfs_deser_ctz(FAR const char * const in,
                              FAR struct mfs_ctz_s * const x);

FAR char *mfs_ser_path(FAR const struct mfs_path_s * const x,
                      FAR char * const out);

FAR const char *mfs_deser_path(FAR const char * const in,
                               FAR struct mfs_path_s * const x);

/****************************************************************************
 * Name: mfs_ser_timespec
 *
 * Description:
 *   Serialize timespec.
 *
 * Input Parameters:
 *   x   - Value to serialize
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR char *mfs_ser_timespec(FAR const struct timespec * const x,
                          FAR char * const out);

/****************************************************************************
 * Name: mfs_deser_timespec
 *
 * Description:
 *   Deserialize timespec.
 *
 * Input Parameters:
 *   in - Input array from where to deserialize.
 *   x  - Value to deserialize
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR const char *mfs_deser_timespec(FAR const char * const in,
                                  FAR struct timespec * const x);

/****************************************************************************
 * Name: mfs_ser_16
 *
 * Description:
 *   Serialize 16 bit values.
 *
 * Input Parameters:
 *   x   - Value to serialize
 *   out - Output array where to serialize.
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR char *mfs_ser_16(const uint16_t n, FAR char * const out);

/****************************************************************************
 * Name: mfs_deser_16
 *
 * Description:
 *   Deserialize 16 bit value.
 *
 * Input Parameters:
 *   in - Input array from where to deserialize.
 *   x  - Value to deserialize
 *
 * Returned Value:
 *   Pointer to byte after the end of serialized value.
 *
 ****************************************************************************/

FAR const char *mfs_deser_16(FAR const char * const in, FAR uint16_t *n);

/****************************************************************************
 * Name: mfs_v2n
 *
 * Description:
 *   v2n(n) math function.
 *
 * Input Parameters:
 *   n   - Number.
 *
 * Returned Value:
 *   v2n(n).
 *
 ****************************************************************************/

mfs_t mfs_v2n(mfs_t n);

/****************************************************************************
 * Name: mfs_set_msb
 *
 * Description:
 *   The mosr significant set bit location.
 *
 * Input Parameters:
 *   n   - Number.
 *
 * Returned Value:
 *   Number after setting the bit.
 *
 ****************************************************************************/

mfs_t mfs_set_msb(mfs_t n);

bool mfs_ctz_eq(FAR const struct mfs_ctz_s * const a,
                FAR const struct mfs_ctz_s * const b);

bool mfs_path_eq(FAR const struct mfs_path_s * const a,
                 FAR const struct mfs_path_s * const b);

/* mnemofs_ctz.c */

/****************************************************************************
 * Name: mfs_ctz_rdfromoff_new
 *
 * Description:
 *   Read from a specific offset into the data in a CTZ file. New version.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   ctz      - CTZ list.
 *   data_off - Offset into the data.
 *   len      - Length of the buffer.
 *   buf      - Buffer to store read contents.
 *
 * Assumptions/Limitations:
 *   The CTZ list provided should be the updated location from the journal.
 *
 ****************************************************************************/

int mfs_ctz_rdfromoff(FAR const struct mfs_sb_s * const sb,
                          const struct mfs_ctz_s ctz, mfs_t data_off,
                          mfs_t len, FAR char * buf);

/****************************************************************************
 * Name: mfs_ctz_wrtnode
 *
 * Description:
 *   Write an LRU node to the flash. It also adds a log of it to the journal.
 *
 * Input Parameters:
 *   sb   - Superblock instance of the device.
 *   node - LRU node
 *
 * Assumptions/Limitations:
 *   - Assumes path is updated by journal. This will also write corresponding
 *     journal log.
 *
 *   - This is the most computationally heavy part of the entire file system
 *     from a human POV, and one of the most for MCU or computer (as init
 *     methods are heavier).
 *
 ****************************************************************************/

int mfs_ctz_wrtnode(FAR struct mfs_sb_s * const sb,
                    FAR const struct mfs_node_s * const node,
                    FAR struct mfs_ctz_s *new_loc);

/****************************************************************************
 * Name: mfs_ctz_travel
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

mfs_t mfs_ctz_travel(FAR const struct mfs_sb_s * const sb,
                     mfs_t idx_src, mfs_t pg_src, mfs_t idx_dest);

/* mnemofs_lru.c */

/****************************************************************************
 * Name: mfs_lru_ctzflush
 *
 * Description:
 *   Flush the updates of a CTZ list inside the LRU to the flash.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   path     - CTZ representation of the relpath.
 *   depth    - Depth of path.
 *
 ****************************************************************************/

int mfs_lru_ctzflush(FAR struct mfs_sb_s * const sb,
                     FAR struct mfs_path_s * const path, const mfs_t depth);

/****************************************************************************
 * Name: mfs_lru_del
 *
 * Description:
 *   Delete instruction to LRU.
 *
 * Input Parameters:
 *   sb     - Superblock instance of the device.
 *   off    - Offset into the data.
 *   bytes  - Number of bytes to delete.
 *   path   - CTZ representation of the relpath.
 *   depth  - Depth of path.
 *
 ****************************************************************************/

int mfs_lru_del(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
                mfs_t bytes, FAR struct mfs_path_s * const path,
                const mfs_t depth);

/****************************************************************************
 * Name: mfs_lru_wr
 *
 * Description:
 *   Write to LRU
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   data_off - Offset into the data.
 *   bytes    - Number of bytes to delete.
 *   path     - CTZ representation of the relpath.
 *   depth    - Depth of path.
 *   buf      - Buffer.
 *
 ****************************************************************************/

int mfs_lru_wr(FAR struct mfs_sb_s * const sb, const mfs_t data_off,
               mfs_t bytes, FAR struct mfs_path_s * const path,
               const mfs_t depth, FAR const char *buf);

/****************************************************************************
 * Name: mfs_lru_init
 *
 * Description:
 *   Initialize LRU.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *
 ****************************************************************************/

void mfs_lru_init(FAR struct mfs_sb_s * const sb);

/****************************************************************************
 * Name: mfs_lru_rdfromoff
 *
 * Description:
 *   Read updated data from an offset upto a certain number of bytes.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   data_off - Offset into the data.
 *   path     - CTZ representation of the relpath.
 *   depth    - Depth of path.
 *   buf      - Buffer to populate with data.
 *   buflen   - Length of data to read.
 *
 ****************************************************************************/

int mfs_lru_rdfromoff(FAR const struct mfs_sb_s * const sb,
                      const mfs_t data_off,
                      FAR struct mfs_path_s * const path, const mfs_t depth,
                      FAR char *buf, const mfs_t buflen);

/****************************************************************************
 * Name: mfs_lru_getupdatedinfo
 *
 * Description:
 *   Update information of the path from the LRU.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   path     - CTZ representation of the relpath.
 *   depth    - Depth of path.
 *
 * Assumptions/Limitations:
 *   The will update path.
 *
 ****************************************************************************/

int mfs_lru_getupdatedinfo(FAR const struct mfs_sb_s * const sb,
                           FAR struct mfs_path_s * const path,
                           const mfs_t depth);

/****************************************************************************
 * Name: mfs_lru_updatectz
 *
 * Description:
 *   Update CTZ location of an fs object.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   path     - Old CTZ representation of the relpath.
 *   depth    - Depth of path.
 *   new_ctz  - New CTZ location.
 *
 * Assumptions/Limitations:
 *   The update the CTZ portion of the direntry in the parent.
 *   This updates the path as well.
 *
 ****************************************************************************/

int mfs_lru_updatectz(FAR struct mfs_sb_s * sb,
                      FAR struct mfs_path_s * const path, const mfs_t depth,
                      const struct mfs_ctz_s new_ctz, mfs_t new_sz);

bool mfs_lru_isempty(FAR struct mfs_sb_s * const sb);
int mfs_lru_flush(FAR struct mfs_sb_s * const sb);

/* mnemofs_master.c */

/****************************************************************************
 * Name: mfs_mn_init
 *
 * Description:
 *   Initialize master node by reading from the flash.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   jrnl_blk - First block of the journal.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   The journal will have to be initialized before this.
 *
 ****************************************************************************/

int mfs_mn_init(FAR struct mfs_sb_s * const sb, const mfs_t jrnl_blk);

/****************************************************************************
 * Name: mfs_mn_fmt
 *
 * Description:
 *   Format a master node.
 *
 * Input Parameters:
 *   sb       - Superblock instance of the device.
 *   jrnl_blk - First block of the journal.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   The journal will have to be formatted before this.
 *
 ****************************************************************************/

int mfs_mn_fmt(FAR struct mfs_sb_s * const sb, const mfs_t blk1,
               const mfs_t blk2, const mfs_t jrnl_blk);

/****************************************************************************
 * Name: mfs_mn_move
 *
 * Description:
 *   Move the master node.
 *
 * Input Parameters:
 *   sb      - Superblock instance of the device.
 *   root    - New location of the root of the file system.
 *   root_sz - New size of the CTZ list of the root of the file syste.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *   This is called when the root is updated to a new location.
 *
 ****************************************************************************/

int mfs_mn_move(FAR struct mfs_sb_s * const sb, struct mfs_ctz_s root,
                const mfs_t root_sz);

int mfs_mn_sync(FAR struct mfs_sb_s *sb,
                FAR struct mfs_path_s * const new_loc,
                const mfs_t blk1, const mfs_t blk2, const mfs_t jrnl_blk);

/* mnemofs_fsobj.c */

/****************************************************************************
 * Name: mfs_path2childname
 *
 * Description:
 *   Given a path, point to the start of the name of the last fs object in
 *   that path (child).
 *
 * Input Parameters:
 *   relpath - Path.
 *
 * Returned Value:
 *   The pointer pointing to where the name of the child starts.
 *
 * Assumptions/Limitations:
 *   This does not allocate a new array for the name, but rather just points
 *   to the place in `relpath` where the name starts.
 *
 ****************************************************************************/

FAR const char * mfs_path2childname(FAR const char *relpath);

/****************************************************************************
 * Name: mfs_get_fsz
 *
 * Description:
 *   Get updated file size of a file.
 *
 * Input Parameters:
 *   sb      - Superblock instance of the device.
 *   path    - CTZ representation of the path.
 *   depth   - Depth of the path.
 *
 * Returned Value:
 *   The size of the file.
 *
 ****************************************************************************/

mfs_t mfs_get_fsz(FAR struct mfs_sb_s * const sb,
                  FAR const struct mfs_path_s * const path,
                  const mfs_t depth);

/****************************************************************************
 * Name: mfs_get_patharr
 *
 * Description:
 *   Takes a relpath, and returns the CTZ location of every file system
 *   object that occurs in the relpath. This also returns a bit flag
 *   consisting of items from `MFS_PATH_FLAGS`.
 *
 * Input Parameters:
 *   sb      - Superblock instance of the device.
 *   relpath - Path of the file.
 *   path    - To populate with CTZ representation of the path.
 *   depth   - To populate with depth of the path.
 *
 * Returned Value:
 *   0 - OK
 *   < - Error
 *
 * Assumptions/Limitations:
 *   This allocates the `path` array in heap, and transfers the ownership
 *   of this array to the caller. It's the caller's reponsibility to use this
 *   with `mfs_free_patharr`.
 *
 ****************************************************************************/

int mfs_get_patharr(FAR const struct mfs_sb_s * const sb,
                    FAR const char * relpath, FAR struct mfs_path_s **path,
                    FAR mfs_t *depth);

/****************************************************************************
 * Name: mfs_free_patharr
 *
 * Description:
 *   Frees up a CTZ representation of the relpath.
 *
 * Input Parameters:
 *   path    - CTZ representation of the relpath.
 *
 ****************************************************************************/

void mfs_free_patharr(FAR struct mfs_path_s *path);

/****************************************************************************
 * Name: mfs_obj_isempty
 *
 * Description:
 *   Checks if a fs object is empty by reading its directory entry.
 *
 * Input Parameters:
 *   sb   - Superblock instance of the device.
 *   pitr - Parent iterator pointing to the fs object.
 *
 * Returned Value:
 *   Is the file or directory pointer by the direntry empty.
 *
 * Assumptions/Limitations:
 *   The `pitr` should point to the fs object being checked.
 *
 ****************************************************************************/

bool mfs_obj_isempty(FAR struct mfs_sb_s * const sb,
                     FAR struct mfs_path_s *path,
                     FAR struct mfs_pitr_s * const pitr);

/****************************************************************************
 * Name: mfs_pitr_init
 *
 * Description:
 *   Initialize a parent iterator (pitr). This iterator starts at the start
 *   of the parent's directory file, and is used to iterate over the
 *   directory entries (direntries) the file has.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   path  - CTZ representation of the relpath.
 *   depth - Depth of CTZ representation of the path.
 *   pitr  - To initialize Parent iterator.
 *   child - If the child exists in the path.
 *
 * Assumptions/Limitations:
 *   - The `pitr` should point to the fs object being checked.
 *
 *   - If the `child` is set to true, it assumes the child is present in the
 *     directory file. If not, it assumes it's not present in the path (due
 *     to whatever circumstances like it's not known if the child exists in
 *     the parent or not, search in parent file, etc.).
 *
 *   - This assumes the pitr is initialized and freed in the same locked
 *     context. If, at all, this is not the case, then at the start of every
 *     new locked context till it's freed, this needs to be synced.
 *
 *   - This also assumes that a CTZ will not change location before the pitr
 *     is destroyed. If at all a CTZ changes location, the pitr needs to be
 *     updated.
 *
 *   - This contains a pitr with all fields set to 0 when it's invalid.
 *
 ****************************************************************************/

int mfs_pitr_init(FAR const struct mfs_sb_s * const sb,
                  FAR const struct mfs_path_s * const path,
                  const mfs_t depth, FAR struct mfs_pitr_s * const pitr,
                  bool child);

/****************************************************************************
 * Name: mfs_pitr_free
 *
 * Description:
 *   Free an initialized pitr.
 *
 * Input Parameters:
 *   pitr  - Parent iterator.
 *
 * Assumptions/Limitations:
 *  It's mainly for symbolic purpose, and the only thing it does is make all
 *  the members reset to 0.
 *
 ****************************************************************************/

void mfs_pitr_free(FAR const struct mfs_pitr_s * const pitr);

/****************************************************************************
 * Name: mfs_pitr_adv
 *
 * Description:
 *   Advance a pitr to the next direntry.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   pitr  - Parent iterator.
 *
 * Assumptions/Limitations:
 *  This is best used if we have not already read the value of a dirent. If
 *  we have, then we do it the `mfs_pitr_adv_dirent` way.
 *
 ****************************************************************************/

int mfs_pitr_adv(FAR struct mfs_sb_s * const sb,
                 FAR struct mfs_path_s *path,
                 FAR struct mfs_pitr_s * const pitr);

/****************************************************************************
 * Name: mfs_pitr_adv_bydirent
 *
 * Description:
 *   Advance a pitr to the next direntry by using current direntry.
 *
 * Input Parameters:
 *   pitr   - Parent iterator.
 *   dirent - Current directory entry.
 *
 * Assumptions/Limitations:
 *  This is best used if we have already got the dirent by which we want to
 *  advance entry (the one pointed by pitr). If we don't, then we do it the
 *  `mfs_pitr_adv` way.
 *
 ****************************************************************************/

void mfs_pitr_adv_bydirent(FAR struct mfs_pitr_s * const pitr,
                           FAR const struct mfs_dirent_s * const dirent);

/****************************************************************************
 * Name: mfs_pitr_adv_off
 *
 * Description:
 *   Advance a pitr by an offset.
 *
 * Input Parameters:
 *   pitr  - Parent iterator.
 *
 * Assumptions/Limitations:
 *  This is best used if we have already got the offset by which we want to
 *  advance entry. This assumes that once the offset is applied, the new
 *  pitr location is valid, and start of a direntry.
 *
 ****************************************************************************/

void mfs_pitr_adv_off(FAR struct mfs_pitr_s * const pitr,
                      const mfs_t off);

/****************************************************************************
 * Name: mfs_pitr_adv_tochild
 *
 * Description:
 *   Advance the pitr to point to the child mentioned in the path.
 *
 * Input Parameters:
 *   pitr   - Parent iterator.
 *   path   - CTZ representation of the relpath
 *
 * Assumptions/Limitations:
 *  This assumes that the pitr is initialized for the immediate parent of the
 *  child.
 *
 ****************************************************************************/

void mfs_pitr_adv_tochild(FAR struct mfs_pitr_s * const pitr,
                          FAR const struct mfs_path_s * const path);

/****************************************************************************
 * Name: mfs_pitr_reset
 *
 * Description:
 *   Reset a pitr to point to the start of the CTZ file.
 *
 * Input Parameters:
 *   pitr  - Parent iterator.
 *
 ****************************************************************************/

void mfs_pitr_reset(FAR struct mfs_pitr_s * const pitr);

/****************************************************************************
 * Name: mfs_pitr_readdirent
 *
 * Description:
 *   Reads the direntry pointed to by the pitr.
 *
 * Input Parameters:
 *   sb     - Superblock instance of the device.
 *   pitr   - Parent Iterator.
 *   dirent - Direntry to populate.
 *
 * Returned Value:
 *   Length of the name of the FS object in the direntry.
 *
 * Assumptions/Limitations:
 *  This alloctes `dirent`, and transfers the ownership to the caller. It's
 *  the caller's responsibility to get it freed using mfs_free_dirent.
 *
 ****************************************************************************/

int mfs_pitr_readdirent(FAR const struct mfs_sb_s * const sb,
                        FAR struct mfs_path_s *path,
                        FAR struct mfs_pitr_s * const pitr,
                        FAR struct mfs_dirent_s **dirent);

/****************************************************************************
 * Name: mfs_pitr_readdirent
 *
 * Description:
 *   Free an allocated direntry.
 *
 * Input Parameters:
 *   dirent - Direntry.
 *
 ****************************************************************************/

void mfs_free_dirent(FAR struct mfs_dirent_s *dirent);

/****************************************************************************
 * Name: mfs_searchfopen
 *
 * Description:
 *   Checks if a file is already open.
 *
 *   In mnemofs, this is done by iterating through the kernel list of open
 *   files.
 *
 * Input Parameters:
 *   sb    - Superblock instance of the device.
 *   path  - CTZ representation of the relpath.
 *   depth - Depth of path.
 *
 * Returned Value:
 *   True  - File is already open.
 *   False - File is not open.
 *
 ****************************************************************************/

bool mfs_searchfopen(FAR const struct mfs_sb_s * const sb,
                    FAR const struct mfs_path_s * const path,
                    const mfs_t depth);

/****************************************************************************
 * Name: mfs_pitr_appenddirent
 *
 * Description:
 *   Appeend a direntry at the end of the directory file of parent that's
 *   initialized in pitr.
 *
 *   In mnemofs, this is done by iterating through the kernel list of open
 *   files.
 *
 * Input Parameters:
 *   sb     - Superblock instance of the device.
 *   path   - CTZ representation of the relpath.
 *   depth  - Depth of path.
 *   pitr   - Parent Iterator.
 *   dirent - Directory entry to be added.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

int mfs_pitr_appenddirent(FAR struct mfs_sb_s * const sb,
                          FAR struct mfs_path_s * const path,
                          const mfs_t depth,
                          FAR struct mfs_pitr_s * const pitr,
                          FAR const struct mfs_dirent_s * const dirent);

/****************************************************************************
 * Name: mfs_pitr_appendnew
 *
 * Description:
 *   Appeend a new entry at the end of the directory file of parent that's
 *   initialized in pitr. This creates the direntry.
 *
 * Input Parameters:
 *   sb      - Superblock instance of the device.
 *   path    - CTZ representation of the relpath.
 *   depth   - Depth of path.
 *   pitr    - Parent Iterator.
 *   relpath - Relative path of the fs object.
 *   mode    - Mode of the file/directory.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *  This assumes that the fs object desired to be appended appears at the
 *  end of the entire relpath.
 *
 ****************************************************************************/

int mfs_pitr_appendnew(FAR struct mfs_sb_s * const sb,
                       FAR struct mfs_path_s * const path, const mfs_t depth,
                       FAR struct mfs_pitr_s * const pitr,
                       FAR const char * const relpath, const mode_t mode);

/****************************************************************************
 * Name: mfs_pitr_rmdirent
 *
 * Description:
 *   Removes the dirent from its parent's directory file.
 *
 * Input Parameters:
 *   sb     - Superblock instance of the device.
 *   path   - CTZ representation of the relpath.
 *   depth  - Depth of path.
 *   pitr   - Parent Iterator.
 *   dirent - Directory entry to be added.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 ****************************************************************************/

int mfs_pitr_rmdirent(FAR struct mfs_sb_s * const sb,
                      FAR struct mfs_path_s * const path,
                      const mfs_t depth, FAR struct mfs_pitr_s * const pitr,
                      FAR const struct mfs_dirent_s * const dirent);

/****************************************************************************
 * Name: mfs_pitr_rm
 *
 * Description:
 *   Removes the fs object pointed by path from its parent's directory file.
 *
 * Input Parameters:
 *   sb     - Superblock instance of the device.
 *   path   - CTZ representation of the relpath.
 *   depth  - Depth of path.
 *
 * Returned Value:
 *   0   - OK
 *   < 0 - Error
 *
 * Assumptions/Limitations:
 *  This does not require any form of pitr initialization. It does it inside
 *  itself.
 *
 ****************************************************************************/

int mfs_pitr_rm(FAR struct mfs_sb_s * const sb,
                FAR struct mfs_path_s * const path,
                const mfs_t depth, bool rm_child);

int mfs_pitr_traversefs(FAR struct mfs_sb_s * sb, const struct mfs_ctz_s ctz,
                        int type);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __FS_MNEMOFS_MNEMOFS_H */
