/****************************************************************************
 * drivers/mtd/mtd_log.c
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
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/nuttx.h>

#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/crc8.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/mtd_log.h>
#include <nuttx/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The magic number for the mtdlog storage format. */

#define MTDLOG_MAGIC (0xa5a5)

/* The minimum allowed block size. A size smaller than this
 * will affect storage performance.
 */

#define MTDLOG_MIN_BLKSIZE (512)

/* The maximum block sequence number, the number of blocks of
 * the mtdlog device must be less than this value.
 */

#define MTDLOG_MAX_BLKSEQ UINT16_MAX

/* The maximum block group number, the number of blocks of
 * the mtdlog device must be less than this value.
 */

#define MTDLOG_MAX_BLKGRP UINT16_MAX

/* The data written to mtdlog must be progsize aligned. */

#define MTDLOG_ALIGN(mtdlog, len) \
  ALIGN_UP((len), (mtdlog)->progsize)

/* An auxiliary macro for traversing mtdlog_entry_s. */

#define MTDLOG_FIRST_ENTRY_OFF(mtdlog) \
  ((mtdlog)->blocksize - MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s)))

#define MTDLOG_NEXT_ENTRY_OFF(mtdlog, off) \
  (off) - MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s))

/* An auxiliary macro for traversing log data. */

#define MTDLOG_FIRST_DATA_OFF(mtdlog) \
  MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_block_s))

#define MTDLOG_NEXT_DATA_OFF(mtdlog, off, len) \
  (off) + MTDLOG_ALIGN(mtdlog, len)

/* The maximum length of a log data allowed by mtdlog. The layout
 * like: blkhdr + logdata + empty_loginfo + valid_loginfo.
 */

#define MTDLOG_MAX_DATA_LEN(mtdlog) \
  ((mtdlog)->blocksize \
   - MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_block_s)) \
   - (MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s)) * 2))

/****************************************************************************
 * Private Types
 ****************************************************************************/

begin_packed_struct struct mtdlog_block_s
{
  uint16_t magic;  /* The magic number */
  uint16_t blkseq; /* The block sequence number */
  uint16_t blkgrp; /* The block group number */
  uint8_t  resv;   /* Reserved for future use */
  uint8_t  crc8;   /* The crc8 of the mtdlog_block_s */
} end_packed_struct;

begin_packed_struct struct mtdlog_entry_s
{
  uint16_t magic;    /* The magic number */
  uint32_t offset;   /* The log data offset in the block */
  uint32_t length;   /* The log data length in the block */
  uint8_t  datacrc8; /* The crc value of the log data */
  uint8_t  crc8;     /* The crc8 of the mtdlog_entry_s */
} end_packed_struct;

struct mtdlog_pos_s
{
  uint16_t blkidx;   /* The block index */
  uint16_t blkseq;   /* The block sequence number */
  uint16_t blkgrp;   /* The block group number */
  uint16_t entrycnt; /* The log count in the block */
  uint32_t entryoff; /* The log entry offset in the block */
  uint32_t dataoff;  /* The log data offset in the block */
};

struct mtdlog_s
{
  FAR struct mtd_dev_s *mtd;       /* MTD device used by the mtdlog */
  spinlock_t           lock;       /* Spinlock for access to the mtdlog */
  struct mtdlog_pos_s  tailpos;    /* Tail position on the mtdlog */
  struct mtdlog_pos_s  headpos;    /* Head position on the mtdlog */
  uint32_t             progsize;   /* Size of one read/write block */
  uint32_t             blocksize;  /* Size of one logical block */
  uint32_t             nblocks;    /* Number of logical blocks */
  uint32_t             erasesize;  /* Size of one erase block */
  uint8_t              erasestate; /* The erase value */
};

struct mtdlog_user_s
{
  struct mtdlog_pos_s  rpos;   /* read position */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int mtdlog_open(FAR struct file *filep);
static int mtdlog_close(FAR struct file *filep);
static ssize_t mtdlog_read(FAR struct file *filep,
                           FAR char *buffer, size_t len);
static ssize_t mtdlog_write(FAR struct file *filep,
                            FAR const char *buffer, size_t len);
static int mtdlog_ioctl(FAR struct file *filep,
                        int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mtdlog_fops =
{
  mtdlog_open,  /* Open */
  mtdlog_close, /* Close */
  mtdlog_read,  /* Read */
  mtdlog_write, /* Write */
  NULL,         /* Seek */
  mtdlog_ioctl, /* Ioctl */
  NULL,         /* Truncate */
  NULL,         /* Mmap */
  NULL          /* Poll */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdlog_is_valid_block
 ****************************************************************************/

static inline bool
mtdlog_is_valid_block(FAR struct mtdlog_block_s *block_info)
{
  uint8_t crc8_val;

  crc8_val = crc8((uint8_t *)block_info, sizeof(struct mtdlog_block_s) - 1);

  return ((block_info->crc8 == crc8_val) &&
          (block_info->magic == MTDLOG_MAGIC));
}

/****************************************************************************
 * Name: mtdlog_is_valid_entry
 ****************************************************************************/

static inline bool
mtdlog_is_valid_entry(FAR struct mtdlog_entry_s *entry_info)
{
  uint8_t crc8_val;

  crc8_val = crc8((uint8_t *)entry_info, sizeof(struct mtdlog_entry_s) - 1);

  return ((entry_info->crc8 == crc8_val) &&
          (entry_info->magic == MTDLOG_MAGIC));
}

/****************************************************************************
 * Name: mtdlog_is_empty_region
 ****************************************************************************/

static inline bool mtdlog_is_empty_region(FAR struct mtdlog_s *mtdlog,
                                          FAR const uint8_t *buf,
                                          size_t len)
{
  size_t i;

  for (i = 0; i < len; i++)
    {
      if (buf[i] != mtdlog->erasestate)
        {
          return false;
        }
    }

  return true;
}

/****************************************************************************
 * Name: mtdlog_is_tailpos
 ****************************************************************************/

static bool mtdlog_is_tailpos(FAR struct mtdlog_s *mtdlog,
                              FAR struct mtdlog_pos_s *pos)
{
  return ((pos->blkidx == mtdlog->tailpos.blkidx) &&
          (pos->blkseq == mtdlog->tailpos.blkseq) &&
          (pos->entryoff == mtdlog->tailpos.entryoff));
}

/****************************************************************************
 * Name: mtdlog_is_valid_pos
 ****************************************************************************/

static bool mtdlog_is_valid_pos(FAR struct mtdlog_s *mtdlog,
                                FAR struct mtdlog_pos_s *pos)
{
  if (mtdlog->tailpos.blkseq >= mtdlog->headpos.blkseq)
    {
      if ((pos->blkseq < mtdlog->headpos.blkseq) ||
          (pos->blkseq > mtdlog->tailpos.blkseq))
        {
          return false;
        }
    }
  else
    {
      if ((pos->blkseq < mtdlog->headpos.blkseq) &&
          (pos->blkseq > mtdlog->tailpos.blkseq))
        {
          return false;
        }
    }

  if ((pos->blkidx == mtdlog->tailpos.blkidx) &&
      (pos->blkseq == mtdlog->tailpos.blkseq) &&
      (pos->entryoff < mtdlog->tailpos.entryoff))
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: mtdlog_read_block
 ****************************************************************************/

static int mtdlog_read_block(FAR struct mtdlog_s *mtdlog,
                             uint16_t blkidx,
                             FAR struct mtdlog_block_s *info)
{
  int ret;
  uint32_t offset;

  offset = blkidx * mtdlog->blocksize;
  ret = MTD_READ(mtdlog->mtd, offset,
                 sizeof(struct mtdlog_block_s), (uint8_t *)info);
  if (ret != sizeof(struct mtdlog_block_s))
    {
      ferr("ERROR: MTD_READ failed: %" PRIu32 ", %d\n", offset, ret);
      return ret < 0 ? ret : -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_write_block
 ****************************************************************************/

static int mtdlog_write_block(FAR struct mtdlog_s *mtdlog,
                              uint16_t blkidx,
                              FAR struct mtdlog_block_s *info)
{
  int ret;
  uint32_t offset;
  uint32_t tmp_buf_size = MTDLOG_ALIGN(mtdlog,
                                       sizeof(struct mtdlog_block_s));
  uint8_t tmp_buf[tmp_buf_size];

  /* Some mtd devices do not require erasing before writing,
   * and thus do not have an erase interface.
   */

  ret = MTD_ERASE(mtdlog->mtd,
                  blkidx * (mtdlog->blocksize / mtdlog->erasesize),
                  (mtdlog->blocksize / mtdlog->erasesize));
  if (ret < 0 && ret != -ENOSYS)
    {
      ferr("ERROR: MTD_ERASE failed: %d, %d\n", blkidx, ret);
      return ret;
    }

  memset(tmp_buf, 0, tmp_buf_size);
  memcpy(tmp_buf, info, sizeof(struct mtdlog_block_s));

  offset = blkidx * mtdlog->blocksize;
  ret = MTD_BWRITE(mtdlog->mtd, (offset / mtdlog->progsize),
                   (tmp_buf_size / mtdlog->progsize), tmp_buf);
  if (ret < 0)
    {
      ferr("ERROR: MTD_BWRITE failed: %" PRIu32 ", %d\n", offset, ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_read_entry
 ****************************************************************************/

static int mtdlog_read_entry(FAR struct mtdlog_s *mtdlog,
                             FAR struct mtdlog_pos_s *pos,
                             FAR struct mtdlog_entry_s *info)
{
  int ret;
  uint32_t offset;

  offset = pos->blkidx * mtdlog->blocksize;
  offset += pos->entryoff;
  ret = MTD_READ(mtdlog->mtd, offset,
                 sizeof(struct mtdlog_entry_s), (uint8_t *)info);
  if (ret != sizeof(struct mtdlog_entry_s))
    {
      ferr("ERROR: MTD_READ failed: %" PRIu32 ", %d\n", offset, ret);
      return ret < 0 ? ret : -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_write_entry
 ****************************************************************************/

static int mtdlog_write_entry(FAR struct mtdlog_s *mtdlog,
                              FAR struct mtdlog_pos_s *pos,
                              FAR struct mtdlog_entry_s *info)
{
  int ret;
  uint32_t offset;
  uint32_t tmp_buf_size = MTDLOG_ALIGN(mtdlog,
                                       sizeof(struct mtdlog_entry_s));
  uint8_t tmp_buf[tmp_buf_size];

  memset(tmp_buf, 0, tmp_buf_size);
  memcpy(tmp_buf, info, sizeof(struct mtdlog_entry_s));

  offset = pos->blkidx * mtdlog->blocksize;
  offset += pos->entryoff;
  ret = MTD_BWRITE(mtdlog->mtd, (offset / mtdlog->progsize),
                   (tmp_buf_size / mtdlog->progsize), tmp_buf);
  if (ret < 0)
    {
      ferr("ERROR: MTD_BWRITE failed: %" PRIu32 ", %d\n", offset, ret);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_read_data
 ****************************************************************************/

static int mtdlog_read_data(FAR struct mtdlog_s *mtdlog,
                            uint32_t offset, uint32_t length,
                            FAR char *buf)
{
  int ret;

  ret = MTD_READ(mtdlog->mtd, offset, length, (uint8_t *)buf);
  if (ret != length)
    {
      ferr("ERROR: MTD_READ failed: %" PRIu32 ", %d\n", offset, ret);
      return ret < 0 ? ret : -EIO;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_write_data
 ****************************************************************************/

static int mtdlog_write_data(FAR struct mtdlog_s *mtdlog,
                             uint32_t offset, uint32_t length,
                             FAR const uint8_t *buf)
{
  int ret;
  uint32_t remain;
  uint32_t tmp_buf_size = mtdlog->progsize;
  uint8_t tmp_buf[tmp_buf_size];

  if (length >= mtdlog->progsize)
    {
      ret = MTD_BWRITE(mtdlog->mtd, (offset / mtdlog->progsize),
                       (length / mtdlog->progsize), buf);
      if (ret < 0)
        {
          ferr("ERROR: MTD_BWRITE failed: %" PRIu32 ", %d\n", offset, ret);
          return ret;
        }
    }

  remain = length % mtdlog->progsize;
  if (remain != 0)
    {
      buf += length - remain;
      memset(tmp_buf, 0, tmp_buf_size);
      memcpy(tmp_buf, buf, remain);

      offset += length - remain;
      ret = MTD_BWRITE(mtdlog->mtd, (offset / mtdlog->progsize),
                       (tmp_buf_size / mtdlog->progsize), tmp_buf);
      if (ret < 0)
        {
          ferr("ERROR: MTD_BWRITE failed: %" PRIu32 ", %d\n", offset, ret);
          return ret;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_update_pos_info
 ****************************************************************************/

static int mtdlog_update_pos_info(FAR struct mtdlog_s *mtdlog,
                                  FAR struct mtdlog_pos_s *pos)
{
  int ret;
  struct mtdlog_entry_s entry_info;

  /* Traverse all entry_info entries in the block. */

  pos->entrycnt = 0;
  pos->entryoff = MTDLOG_FIRST_ENTRY_OFF(mtdlog);
  pos->dataoff = MTDLOG_FIRST_DATA_OFF(mtdlog);

  while (1)
    {
      ret = mtdlog_read_entry(mtdlog, pos, &entry_info);
      if (ret < 0)
        {
          return ret;
        }

      if (mtdlog_is_empty_region(mtdlog,
          (FAR const uint8_t *)&entry_info, sizeof(entry_info)))
        {
          break;
        }

      if (!mtdlog_is_valid_entry(&entry_info))
        {
          /* Power failure may cause the data of the entry to not
           * be fully written. When encountering a damaged entry,
           * handle it as an entry with a data length of 0.
           */

          entry_info.length = 0;
        }
      else
        {
          pos->dataoff = entry_info.offset;
        }

        pos->entrycnt++;
        pos->entryoff -=
          MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s));
        pos->dataoff +=
          MTDLOG_ALIGN(mtdlog, entry_info.length);
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_move_tail_to_nextblk
 ****************************************************************************/

static int mtdlog_move_tail_to_nextblk(FAR struct mtdlog_s *mtdlog,
                                       uint16_t blkgrp)
{
  int ret;
  uint16_t next_tail_blkidx;
  uint16_t next_head_blkidx;
  struct mtdlog_block_s block_info;

  next_tail_blkidx = (mtdlog->tailpos.blkidx + 1) % mtdlog->nblocks;

  /* If the next write block and the head block are the same
   * block, move the head block to the next block.
   */

  if (next_tail_blkidx == mtdlog->headpos.blkidx)
    {
      next_head_blkidx = (mtdlog->headpos.blkidx + 1) % mtdlog->nblocks;
      ret = mtdlog_read_block(mtdlog, next_head_blkidx, &block_info);
      if (ret < 0)
        {
          return ret;
        }

      mtdlog->headpos.blkseq = block_info.blkseq;
      mtdlog->headpos.blkidx = next_head_blkidx;
      mtdlog->headpos.blkgrp = block_info.blkgrp;
      mtdlog->headpos.entrycnt = 0;
      mtdlog->headpos.entryoff = MTDLOG_FIRST_ENTRY_OFF(mtdlog);
      mtdlog->headpos.dataoff = MTDLOG_FIRST_DATA_OFF(mtdlog);
    }

  /* Write the block information to the MTD and
   * update the tailpos information.
   */

  memset(&block_info, 0, sizeof(struct mtdlog_block_s));
  block_info.magic = MTDLOG_MAGIC;
  block_info.blkseq = (mtdlog->tailpos.blkseq + 1);
  block_info.blkgrp = blkgrp;
  block_info.crc8 = crc8((uint8_t *)&block_info,
                         sizeof(struct mtdlog_block_s) - 1);

  ret = mtdlog_write_block(mtdlog, next_tail_blkidx, &block_info);
  if (ret < 0)
    {
      return ret;
    }

  mtdlog->tailpos.blkidx = next_tail_blkidx;
  mtdlog->tailpos.blkseq = block_info.blkseq;
  mtdlog->tailpos.blkgrp = block_info.blkgrp;

  mtdlog->tailpos.entrycnt = 0;
  mtdlog->tailpos.entryoff = MTDLOG_FIRST_ENTRY_OFF(mtdlog);
  mtdlog->tailpos.dataoff = MTDLOG_FIRST_DATA_OFF(mtdlog);

  return 0;
}

/****************************************************************************
 * Name: mtdlog_move_rpos_to_nextblk
 ****************************************************************************/

static int mtdlog_move_rpos_to_nextblk(FAR struct mtdlog_s *mtdlog,
                                       FAR struct mtdlog_pos_s *rpos)
{
  int ret;
  uint16_t next_rpos_blkidx;
  struct mtdlog_block_s block_info;

  if (mtdlog->tailpos.blkidx == rpos->blkidx)
    {
      return -ENOENT;
    }

  next_rpos_blkidx = (rpos->blkidx + 1) % mtdlog->nblocks;
  ret = mtdlog_read_block(mtdlog, next_rpos_blkidx, &block_info);
  if (ret < 0)
    {
      return ret;
    }

  rpos->blkseq = block_info.blkseq;
  rpos->blkidx = next_rpos_blkidx;
  rpos->blkgrp = block_info.blkgrp;
  rpos->entrycnt = 0;
  rpos->entryoff = MTDLOG_FIRST_ENTRY_OFF(mtdlog);
  rpos->dataoff = MTDLOG_FIRST_DATA_OFF(mtdlog);

  return 0;
}

/****************************************************************************
 * Name: mtdlog_detect
 ****************************************************************************/

static int mtdlog_detect(FAR struct mtdlog_s *mtdlog)
{
  int ret;
  uint32_t i;
  uint32_t around = 0;
  uint16_t min_blkseq = 0;
  uint16_t max_blkseq = 0;
  uint16_t min_blkseq_idx = 0;
  uint16_t max_blkseq_idx = 0;
  uint16_t min_blkseq_grp = 0;
  uint16_t max_blkseq_grp = 0;
  uint16_t valid_block_cnt = 0;
  uint16_t valid_blkseq_info = 0;
  struct mtdlog_block_s block_info;

  /* Traverse all blocks to find the head and tail block. */

  for (i = 0; i < mtdlog->nblocks; i++)
    {
      ret = mtdlog_read_block(mtdlog, i, &block_info);
      if (ret < 0)
        {
          return ret;
        }

      if (!mtdlog_is_valid_block(&block_info))
        {
          finfo("Invalid block found at %" PRIu32 "\n", i);
          continue;
        }

      valid_block_cnt++;

      if (valid_block_cnt == 1)
        {
          /* The first valid block was found, initialize the information
           * of min_blkseq and max_blkseq using block_info. Meanwhile, the
           * corresponding blkidx and blkgrp were recorded.
           */

          min_blkseq = block_info.blkseq;
          min_blkseq_idx = i;
          min_blkseq_grp = block_info.blkgrp;
          max_blkseq = block_info.blkseq;
          max_blkseq_idx = i;
          max_blkseq_grp = block_info.blkgrp;
        }

      if ((max_blkseq == MTDLOG_MAX_BLKSEQ) && (block_info.blkseq == 0))
        {
          /* The blkseq wraparound occurs when the blkseq reaches
           * its maximum value and rolls back to zero.
           * In this case, stop updating the minimum blkseq and
           * reset the maximum blkseq with current block.
           */

          around = 1;
          max_blkseq = block_info.blkseq;
          max_blkseq_idx = i;
          max_blkseq_grp = block_info.blkgrp;
        }

      if ((around == 0) && (block_info.blkseq < min_blkseq))
        {
          min_blkseq = block_info.blkseq;
          min_blkseq_idx = i;
          min_blkseq_grp = block_info.blkgrp;
        }

      if (block_info.blkseq > max_blkseq)
        {
          max_blkseq = block_info.blkseq;
          max_blkseq_idx = i;
          max_blkseq_grp = block_info.blkgrp;
        }
    }

  /* No valid block was found. */

  if (valid_block_cnt == 0)
    {
      finfo("No valid block was found\n");
      return -ENOENT;
    }

  finfo("valid_block_cnt: %d\n", valid_block_cnt);

  /* Check whether blkseq information is invalid. */

  if (around == 0)
    {
      uint16_t block_cnt;

      block_cnt = (max_blkseq - min_blkseq + 1);

      if ((max_blkseq >= min_blkseq) && (block_cnt == valid_block_cnt))
        {
          valid_blkseq_info = 1;
        }
    }
  else
    {
      uint16_t block_cnt;

      block_cnt = (MTDLOG_MAX_BLKSEQ - min_blkseq + 1);
      block_cnt += (max_blkseq + 1);

      if ((max_blkseq < min_blkseq) && (block_cnt == valid_block_cnt))
        {
          valid_blkseq_info = 1;
        }
    }

  /* The blkseq information is invalid. */

  if (valid_blkseq_info == 0)
    {
      finfo("The blkseq information is invalid\n");
      return -EINVAL;
    }

  /* The max_blkseq corresponds to the last block, and
   * the min_blkseq corresponds to the first block.
   */

  mtdlog->headpos.blkseq = min_blkseq;
  mtdlog->headpos.blkidx = min_blkseq_idx;
  mtdlog->headpos.blkgrp = min_blkseq_grp;
  mtdlog->headpos.entrycnt = 0;
  mtdlog->headpos.entryoff = MTDLOG_FIRST_ENTRY_OFF(mtdlog);
  mtdlog->headpos.dataoff = MTDLOG_FIRST_DATA_OFF(mtdlog);

  mtdlog->tailpos.blkseq = max_blkseq;
  mtdlog->tailpos.blkidx = max_blkseq_idx;
  mtdlog->tailpos.blkgrp = max_blkseq_grp;
  ret = mtdlog_update_pos_info(mtdlog, &mtdlog->tailpos);
  if (ret < 0)
    {
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_start_new_blkgrp
 ****************************************************************************/

static int mtdlog_start_new_blkgrp(FAR struct mtdlog_s *mtdlog)
{
  uint16_t blkgrp;

  blkgrp = mtdlog->tailpos.blkgrp + 1;

  return mtdlog_move_tail_to_nextblk(mtdlog, blkgrp);
}

/****************************************************************************
 * Name: mtdlog_first_block_init
 ****************************************************************************/

static int mtdlog_first_block_init(FAR struct mtdlog_s *mtdlog)
{
  int ret;
  struct mtdlog_block_s block_info;

  mtdlog->headpos.blkidx = 0;
  mtdlog->headpos.blkseq = 0;
  mtdlog->headpos.blkgrp = 0;

  mtdlog->tailpos.blkidx = 0;
  mtdlog->tailpos.blkseq = 0;
  mtdlog->tailpos.blkgrp = 0;

  mtdlog->headpos.entrycnt = 0;
  mtdlog->headpos.entryoff = MTDLOG_FIRST_ENTRY_OFF(mtdlog);
  mtdlog->headpos.dataoff = MTDLOG_FIRST_DATA_OFF(mtdlog);

  mtdlog->tailpos.entrycnt = 0;
  mtdlog->tailpos.entryoff = MTDLOG_FIRST_ENTRY_OFF(mtdlog);
  mtdlog->tailpos.dataoff = MTDLOG_FIRST_DATA_OFF(mtdlog);

  memset(&block_info, 0, sizeof(struct mtdlog_block_s));
  block_info.magic = MTDLOG_MAGIC;
  block_info.blkseq = 0;
  block_info.blkgrp = 0;
  block_info.crc8 = crc8((uint8_t *)&block_info,
                         sizeof(struct mtdlog_block_s) - 1);

  ret = mtdlog_write_block(mtdlog, 0, &block_info);
  if (ret < 0)
    {
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_init
 ****************************************************************************/

static int mtdlog_init(FAR struct mtdlog_s *mtdlog)
{
  int ret;

  if ((mtdlog->nblocks <= 1) ||
      (mtdlog->nblocks > MTDLOG_MAX_BLKSEQ) ||
      (mtdlog->nblocks > MTDLOG_MAX_BLKGRP) ||
      (mtdlog->progsize == 0) ||
      (mtdlog->progsize >= mtdlog->blocksize) ||
      (mtdlog->blocksize % mtdlog->progsize != 0))
    {
      ferr("mtdlog: invalid mtd geometry parameters\n");
      return -EINVAL;
    }

  finfo("mtdlog->nblocks: %" PRIu32 "\n", mtdlog->nblocks);
  finfo("mtdlog->blocksize: %" PRIu32 "\n", mtdlog->blocksize);
  finfo("mtdlog->progsize: %" PRIu32 "\n", mtdlog->progsize);
  finfo("mtdlog->erasestate: %d\n", mtdlog->erasestate);

  ret = mtdlog_detect(mtdlog);
  if (ret == 0)
    {
      /* move tail to next block and update blkgrp. */

      ret = mtdlog_start_new_blkgrp(mtdlog);
    }
  else
    {
      /* init first block of the mtdlog device. */

      ret = mtdlog_first_block_init(mtdlog);
    }

  return ret;
}

/****************************************************************************
 * Name: mtdlog_read_log_entry
 ****************************************************************************/

static int mtdlog_read_log_entry(FAR struct mtdlog_s *mtdlog,
                                 FAR struct mtdlog_user_s *upriv,
                                 FAR char *buffer, size_t len)
{
  int ret;
  uint32_t offset;
  struct mtdlog_entry_s entry_info;

  while (1)
    {
      if (!mtdlog_is_valid_pos(mtdlog, &upriv->rpos))
        {
          upriv->rpos = mtdlog->headpos;
        }

      if (mtdlog_is_tailpos(mtdlog, &upriv->rpos))
        {
          return -ENOENT;
        }

      ret = mtdlog_read_entry(mtdlog, &upriv->rpos, &entry_info);
      if (ret < 0)
        {
          return ret;
        }

      if (mtdlog_is_empty_region(mtdlog,
          (FAR const uint8_t *)&entry_info, sizeof(entry_info)))
        {
          ret = mtdlog_move_rpos_to_nextblk(mtdlog, &upriv->rpos);
          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      if (!mtdlog_is_valid_entry(&entry_info))
        {
          /* Power failure may cause the data of the entry to not
           * be fully written. When encountering a damaged entry,
           * handle it as an entry with a data length of 0.
           */

          entry_info.length = 0;

          upriv->rpos.entryoff -=
            MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s));
          upriv->rpos.dataoff +=
            MTDLOG_ALIGN(mtdlog, entry_info.length);

          continue;
        }
      else
        {
          upriv->rpos.dataoff = entry_info.offset;
        }

      if (entry_info.length > len)
        {
          return -ENOSPC;
        }

      offset = upriv->rpos.blkidx * mtdlog->blocksize;
      offset += upriv->rpos.dataoff;
      ret = mtdlog_read_data(mtdlog, offset, entry_info.length, buffer);
      if (ret < 0)
        {
          return -EIO;
        }

      break;
    }

  upriv->rpos.entryoff -=
    MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s));
  upriv->rpos.dataoff +=
    MTDLOG_ALIGN(mtdlog, entry_info.length);

  return entry_info.length;
}

/****************************************************************************
 * Name: mtdlog_write_log_entry
 ****************************************************************************/

static int mtdlog_write_log_entry(FAR struct mtdlog_s *mtdlog,
                                  FAR const char *buffer, size_t len)
{
  int ret;
  uint32_t offset;
  uint32_t blk_remain;
  uint32_t len_write;
  struct mtdlog_entry_s entry_info;

  blk_remain = mtdlog->tailpos.entryoff -
    mtdlog->tailpos.dataoff;
  blk_remain -= MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s));
  if ((MTDLOG_ALIGN(mtdlog, len) +
       MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s))) > blk_remain)
    {
      ret = mtdlog_move_tail_to_nextblk(mtdlog,
                                          mtdlog->tailpos.blkgrp);
      if (ret < 0)
        {
          return ret;
        }
    }

  len_write = len;
  memset(&entry_info, 0, sizeof(struct mtdlog_entry_s));
  entry_info.magic = MTDLOG_MAGIC;
  entry_info.length = len_write;
  entry_info.offset = mtdlog->tailpos.dataoff;
  entry_info.datacrc8 = crc8((uint8_t *)buffer, len_write);
  entry_info.crc8 = crc8((uint8_t *)&entry_info, sizeof(entry_info) - 1);

  ret = mtdlog_write_entry(mtdlog, &mtdlog->tailpos, &entry_info);
  if (ret < 0)
    {
      return ret;
    }

  offset = mtdlog->tailpos.blkidx * mtdlog->blocksize;
  offset += mtdlog->tailpos.dataoff;
  ret = mtdlog_write_data(mtdlog, offset,
                          len_write, (const uint8_t *)buffer);
  if (ret < 0)
    {
      return ret;
    }

  mtdlog->tailpos.entrycnt++;
  mtdlog->tailpos.entryoff -=
    MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s));
  mtdlog->tailpos.dataoff +=
    MTDLOG_ALIGN(mtdlog, len_write);

  return len_write;
}

/****************************************************************************
 * Name: mtdlog_query_cur_loginfo
 ****************************************************************************/

static int mtdlog_query_cur_loginfo(FAR struct mtdlog_s *mtdlog,
                                    FAR struct mtdlog_user_s *upriv,
                                    FAR struct mtdlog_loginfo_s *info)
{
  int ret;
  struct mtdlog_entry_s entry_info;

  while (1)
    {
      if (!mtdlog_is_valid_pos(mtdlog, &upriv->rpos))
        {
          upriv->rpos = mtdlog->headpos;
        }

      if (mtdlog_is_tailpos(mtdlog, &upriv->rpos))
        {
          return -ENOENT;
        }

      ret = mtdlog_read_entry(mtdlog, &upriv->rpos, &entry_info);
      if (ret < 0)
        {
          return ret;
        }

      if (mtdlog_is_empty_region(mtdlog,
          (FAR const uint8_t *)&entry_info, sizeof(entry_info)))
        {
          ret = mtdlog_move_rpos_to_nextblk(mtdlog, &upriv->rpos);
          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      break;
    }

  if (!mtdlog_is_valid_entry(&entry_info))
    {
      info->valid = false;
    }
  else
    {
      uint32_t offset;

      offset = upriv->rpos.blkidx * mtdlog->blocksize;
      offset += entry_info.offset;

      info->valid = true;
      info->offset = offset;
      info->length = entry_info.length;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_query_cur_blkinfo
 ****************************************************************************/

static int mtdlog_query_cur_blkinfo(FAR struct mtdlog_s *mtdlog,
                                    FAR struct mtdlog_user_s *upriv,
                                    FAR struct mtdlog_blkinfo_s *info)
{
  int ret;
  uint16_t blkidx;
  struct mtdlog_block_s block_info;

  blkidx = upriv->rpos.blkidx;
  ret = mtdlog_read_block(mtdlog, blkidx, &block_info);
  if (ret < 0)
    {
      return ret;
    }

  if (!mtdlog_is_valid_block(&block_info))
    {
      info->valid = false;
    }
  else
    {
      info->valid = true;
      info->blkseq = block_info.blkseq;
      info->blkgrp = block_info.blkgrp;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_query_cur_status
 ****************************************************************************/

static int mtdlog_query_cur_status(FAR struct mtdlog_s *mtdlog,
                                   FAR struct mtdlog_user_s *upriv,
                                   FAR struct mtdlog_status_s *status)
{
  int ret;
  struct mtdlog_entry_s entry_info;
  struct mtdlog_pos_s log_rpos;

  status->logcnt = 0;
  status->logoff = 0;
  status->logsize = 0;

  log_rpos = mtdlog->headpos;
  while (1)
    {
      if (mtdlog_is_tailpos(mtdlog, &log_rpos))
        {
          break;
        }

      ret = mtdlog_read_entry(mtdlog, &log_rpos, &entry_info);
      if (ret < 0)
        {
          return ret;
        }

      if (mtdlog_is_empty_region(mtdlog,
          (FAR const uint8_t *)&entry_info, sizeof(entry_info)))
        {
          ret = mtdlog_move_rpos_to_nextblk(mtdlog, &log_rpos);
          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      if ((upriv->rpos.blkidx == log_rpos.blkidx) &&
          (upriv->rpos.entryoff == log_rpos.entryoff))
        {
          status->logoff = status->logcnt;
        }

      status->logcnt++;
      status->logsize += entry_info.length;

      log_rpos.entryoff -=
        MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s));
      log_rpos.dataoff +=
        MTDLOG_ALIGN(mtdlog, entry_info.length);
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_query_log_entry_count
 ****************************************************************************/

static int mtdlog_query_log_entry_count(FAR struct mtdlog_s *mtdlog,
                                        FAR struct mtdlog_user_s *upriv,
                                        FAR uint32_t *count)
{
  int ret;
  uint32_t total_count;
  struct mtdlog_entry_s entry_info;
  struct mtdlog_pos_s log_rpos;

  UNUSED(upriv);

  total_count = 0;
  log_rpos = mtdlog->headpos;
  while (1)
    {
      if (mtdlog_is_tailpos(mtdlog, &log_rpos))
        {
          break;
        }

      ret = mtdlog_read_entry(mtdlog, &log_rpos, &entry_info);
      if (ret < 0)
        {
          return ret;
        }

      if (mtdlog_is_empty_region(mtdlog,
          (FAR const uint8_t *)&entry_info, sizeof(entry_info)))
        {
          ret = mtdlog_move_rpos_to_nextblk(mtdlog, &log_rpos);
          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      total_count++;

      log_rpos.entryoff -=
        MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s));
      log_rpos.dataoff +=
        MTDLOG_ALIGN(mtdlog, entry_info.length);
    }

  *count = total_count;
  return 0;
}

/****************************************************************************
 * Name: mtdlog_seek_cur_log_entry
 ****************************************************************************/

static int mtdlog_seek_cur_log_entry(FAR struct mtdlog_s *mtdlog,
                                     FAR struct mtdlog_user_s *upriv,
                                     uint32_t count)
{
  int ret;
  struct mtdlog_entry_s entry_info;
  struct mtdlog_pos_s log_rpos;

  if (count == 0)
    {
      return 0;
    }

  log_rpos = upriv->rpos;
  while (1)
    {
      if (!mtdlog_is_valid_pos(mtdlog, &log_rpos))
        {
          log_rpos = mtdlog->headpos;
        }

      if (mtdlog_is_tailpos(mtdlog, &log_rpos))
        {
          return -ENOENT;
        }

      ret = mtdlog_read_entry(mtdlog, &log_rpos, &entry_info);
      if (ret < 0)
        {
          return ret;
        }

      if (mtdlog_is_empty_region(mtdlog,
          (FAR const uint8_t *)&entry_info, sizeof(entry_info)))
        {
          ret = mtdlog_move_rpos_to_nextblk(mtdlog, &log_rpos);
          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      count--;

      log_rpos.entryoff -=
        MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s));
      log_rpos.dataoff +=
        MTDLOG_ALIGN(mtdlog, entry_info.length);

      if (count == 0)
        {
          upriv->rpos = log_rpos;
          break;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_seek_set_log_entry
 ****************************************************************************/

static int mtdlog_seek_set_log_entry(FAR struct mtdlog_s *mtdlog,
                                     FAR struct mtdlog_user_s *upriv,
                                     uint32_t count)
{
  int ret;
  uint32_t total_count;
  struct mtdlog_entry_s entry_info;
  struct mtdlog_pos_s log_rpos;

  if (count == 0)
    {
      upriv->rpos = mtdlog->headpos;
      return 0;
    }

  total_count = 0;
  log_rpos = mtdlog->headpos;
  while (1)
    {
      if (mtdlog_is_tailpos(mtdlog, &log_rpos))
        {
          return -ENOENT;
        }

      ret = mtdlog_read_entry(mtdlog, &log_rpos, &entry_info);
      if (ret < 0)
        {
          return ret;
        }

      if (mtdlog_is_empty_region(mtdlog,
          (FAR const uint8_t *)&entry_info, sizeof(entry_info)))
        {
          ret = mtdlog_move_rpos_to_nextblk(mtdlog, &log_rpos);
          if (ret < 0)
            {
              return ret;
            }

          continue;
        }

      total_count++;

      log_rpos.entryoff -=
        MTDLOG_ALIGN(mtdlog, sizeof(struct mtdlog_entry_s));
      log_rpos.dataoff +=
        MTDLOG_ALIGN(mtdlog, entry_info.length);

      if (total_count == count)
        {
          upriv->rpos = log_rpos;
          break;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_query_blk_group_count
 ****************************************************************************/

static int mtdlog_query_blk_group_count(FAR struct mtdlog_s *mtdlog,
                                        FAR struct mtdlog_user_s *upriv,
                                        FAR uint32_t *count)
{
  int ret;
  uint16_t head_blkidx;
  uint16_t tail_blkidx;
  uint16_t head_blkgrp;
  uint16_t tail_blkgrp;
  struct mtdlog_block_s block_info;

  UNUSED(upriv);

  head_blkidx = mtdlog->headpos.blkidx;
  tail_blkidx = mtdlog->tailpos.blkidx;

  ret = mtdlog_read_block(mtdlog, head_blkidx, &block_info);
  if (ret < 0)
    {
      return ret;
    }

  head_blkgrp = block_info.blkgrp;

  ret = mtdlog_read_block(mtdlog, tail_blkidx, &block_info);
  if (ret < 0)
    {
      return ret;
    }

  tail_blkgrp = block_info.blkgrp;

  if (head_blkgrp <= tail_blkgrp)
    {
      *count = tail_blkgrp - head_blkgrp + 1;
    }
  else
    {
      *count = MTDLOG_MAX_BLKGRP - head_blkgrp + 1;
      *count += tail_blkgrp + 1;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_seek_cur_blk_group
 ****************************************************************************/

static int mtdlog_seek_cur_blk_group(FAR struct mtdlog_s *mtdlog,
                                     FAR struct mtdlog_user_s *upriv,
                                     uint32_t count)
{
  int ret;
  uint16_t found;
  uint16_t blkidx;
  uint16_t blkgrp;
  uint16_t head_blkidx;
  uint16_t tail_blkidx;
  uint16_t total_count;
  struct mtdlog_block_s block_info;

  if (count == 0)
    {
      return 0;
    }

  head_blkidx = upriv->rpos.blkidx;
  tail_blkidx = mtdlog->tailpos.blkidx;
  blkgrp = upriv->rpos.blkgrp;

  if (tail_blkidx < head_blkidx)
    {
      tail_blkidx += mtdlog->nblocks;
    }

  found = 0;
  total_count = 0;
  while (head_blkidx <= tail_blkidx)
    {
      blkidx = head_blkidx % mtdlog->nblocks;
      ret = mtdlog_read_block(mtdlog, blkidx, &block_info);
      if (ret < 0)
        {
          return ret;
        }

      if (block_info.blkgrp != blkgrp)
        {
          total_count++;
          blkgrp = block_info.blkgrp;
        }

      if (total_count == count)
        {
          found = 1;
          break;
        }

      head_blkidx++;
    }

  if (found)
    {
      struct mtdlog_pos_s log_rpos;

      log_rpos.blkseq = block_info.blkseq;
      log_rpos.blkidx = blkidx;
      log_rpos.blkgrp = block_info.blkgrp;
      log_rpos.entrycnt = 0;
      log_rpos.entryoff = MTDLOG_FIRST_ENTRY_OFF(mtdlog);
      log_rpos.dataoff = MTDLOG_FIRST_DATA_OFF(mtdlog);

      upriv->rpos = log_rpos;
      return 0;
    }
  else
    {
      return -ENOENT;
    }
}

/****************************************************************************
 * Name: mtdlog_seek_set_blk_group
 ****************************************************************************/

static int mtdlog_seek_set_blk_group(FAR struct mtdlog_s *mtdlog,
                                     FAR struct mtdlog_user_s *upriv,
                                     uint32_t count)
{
  int ret;
  uint16_t found;
  uint16_t blkidx;
  uint16_t blkgrp;
  uint16_t head_blkidx;
  uint16_t tail_blkidx;
  uint16_t total_count;
  struct mtdlog_block_s block_info;

  if (count == 0)
    {
      upriv->rpos = mtdlog->headpos;
      return 0;
    }

  head_blkidx = mtdlog->headpos.blkidx;
  tail_blkidx = mtdlog->tailpos.blkidx;
  blkgrp = mtdlog->headpos.blkgrp;

  if (tail_blkidx < head_blkidx)
    {
      tail_blkidx += mtdlog->nblocks;
    }

  found = 0;
  total_count = 0;
  while (head_blkidx <= tail_blkidx)
    {
      blkidx = head_blkidx % mtdlog->nblocks;
      ret = mtdlog_read_block(mtdlog, blkidx, &block_info);
      if (ret < 0)
        {
          return ret;
        }

      if (block_info.blkgrp != blkgrp)
        {
          total_count++;
          blkgrp = block_info.blkgrp;
        }

      if (total_count == count)
        {
          found = 1;
          break;
        }

      head_blkidx++;
    }

  if (found)
    {
      struct mtdlog_pos_s log_rpos;

      log_rpos.blkseq = block_info.blkseq;
      log_rpos.blkidx = blkidx;
      log_rpos.blkgrp = block_info.blkgrp;
      log_rpos.entrycnt = 0;
      log_rpos.entryoff = MTDLOG_FIRST_ENTRY_OFF(mtdlog);
      log_rpos.dataoff = MTDLOG_FIRST_DATA_OFF(mtdlog);

      upriv->rpos = log_rpos;
      return 0;
    }
  else
    {
      return -ENOENT;
    }
}

/****************************************************************************
 * Name: mtdlog_open
 ****************************************************************************/

static int mtdlog_open(FAR struct file *filep)
{
  irqstate_t flags;
  FAR struct mtdlog_user_s *upriv;
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdlog_s *mtdlog = inode->i_private;

  upriv = kmm_malloc(sizeof(struct mtdlog_user_s));
  if (upriv == NULL)
    {
      return -ENOMEM;
    }

  flags = spin_lock_irqsave(&mtdlog->lock);
  upriv->rpos = mtdlog->headpos;
  spin_unlock_irqrestore(&mtdlog->lock, flags);

  filep->f_priv = upriv;
  return 0;
}

/****************************************************************************
 * Name: mtdlog_close
 ****************************************************************************/

static int mtdlog_close(FAR struct file *filep)
{
  FAR struct mtdlog_user_s *upriv;

  upriv = filep->f_priv;
  kmm_free(upriv);

  return 0;
}

/****************************************************************************
 * Name: mtdlog_read
 ****************************************************************************/

static ssize_t mtdlog_read(FAR struct file *filep,
                           FAR char *buffer, size_t len)
{
  int ret;
  irqstate_t flags;
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdlog_s *mtdlog = inode->i_private;
  FAR struct mtdlog_user_s *upriv = filep->f_priv;

  flags = spin_lock_irqsave(&mtdlog->lock);
  ret = mtdlog_read_log_entry(mtdlog, upriv, buffer, len);
  spin_unlock_irqrestore(&mtdlog->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: mtdlog_write
 ****************************************************************************/

static ssize_t mtdlog_write(FAR struct file *filep,
                            FAR const char *buffer, size_t len)
{
  int ret;
  irqstate_t flags;
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdlog_s *mtdlog = inode->i_private;

  if (len > MTDLOG_MAX_DATA_LEN(mtdlog))
    {
      return -EINVAL;
    }

  flags = spin_lock_irqsave(&mtdlog->lock);
  ret = mtdlog_write_log_entry(mtdlog, buffer, len);
  spin_unlock_irqrestore(&mtdlog->lock, flags);

  return ret;
}

/****************************************************************************
 * Name: mtdlog_ioctl
 ****************************************************************************/

static int mtdlog_ioctl(FAR struct file *filep,
                        int cmd, unsigned long arg)
{
  int ret;
  irqstate_t flags;
  FAR struct inode *inode = filep->f_inode;
  FAR struct mtdlog_s *mtdlog = inode->i_private;
  FAR struct mtdlog_user_s *upriv = filep->f_priv;

  flags = spin_lock_irqsave(&mtdlog->lock);

  switch (cmd)
    {
      case MTDLOGIOC_GET_ENTRYINFO:
        {
          FAR struct mtdlog_loginfo_s *info = (struct mtdlog_loginfo_s *)arg;

          /* Obtain the log information pointed to by the
          * current read pointer.
          */

          if (info)
            {
              ret = mtdlog_query_cur_loginfo(mtdlog, upriv, info);
            }
          else
            {
              ret = -EINVAL;
            }

          break;
        }

      case MTDLOGIOC_GET_BLOCKINFO:
        {
          FAR struct mtdlog_blkinfo_s *info = (struct mtdlog_blkinfo_s *)arg;

          /* Obtain the blk information pointed to by the
          * current read pointer.
          */

          if (info)
            {
              ret = mtdlog_query_cur_blkinfo(mtdlog, upriv, info);
            }
          else
            {
              ret = -EINVAL;
            }

          break;
        }

      case MTDLOGIOC_STATUS:
        {
          FAR struct mtdlog_status_s *status = (struct mtdlog_status_s *)arg;

          /* Obtain the log status information pointed to by the
          * current read pointer.
          */

          if (status)
            {
              ret = mtdlog_query_cur_status(mtdlog, upriv, status);
            }
          else
            {
              ret = -EINVAL;
            }

          break;
        }

      case MTDLOGIOC_LOG_COUNT:
        {
          FAR uint32_t *log_count = (uint32_t *)arg;

          /* Obtain the total number of log entries in the
          * current storage device.
          */

          if (log_count)
            {
              ret = mtdlog_query_log_entry_count(mtdlog, upriv, log_count);
            }
          else
            {
              ret = -EINVAL;
            }

          break;
        }

      case MTDLOGIOC_LOG_SEEK_CUR:
        {
          uint32_t log_count = (uint32_t)arg;

          /* Move the current read pointer forward and move
          * the specified number of log entries.
          */

          ret = mtdlog_seek_cur_log_entry(mtdlog, upriv, log_count);

          break;
        }

      case MTDLOGIOC_LOG_SEEK_SET:
        {
          uint32_t log_count = (uint32_t)arg;

          /* Move the current read pointer to the specified position
          * and move it to the specified log entry.
          */

          ret = mtdlog_seek_set_log_entry(mtdlog, upriv, log_count);

          break;
        }

      case MTDLOGIOC_GRP_COUNT:
        {
          FAR uint32_t *grp_count = (uint32_t *)arg;

          /* Obtain the total number of blk groups in the
          * current storage device.
          */

          if (grp_count)
            {
              ret = mtdlog_query_blk_group_count(mtdlog, upriv, grp_count);
            }
          else
            {
              ret = -EINVAL;
            }

          break;
        }

      case MTDLOGIOC_GRP_SEEK_CUR:
        {
          uint32_t grp_count = (uint32_t)arg;

          /* Move the current read pointer forward and move
          * the specified number of blk groups.
          */

          ret = mtdlog_seek_cur_blk_group(mtdlog, upriv, grp_count);

          break;
        }

      case MTDLOGIOC_GRP_SEEK_SET:
        {
          uint32_t grp_count = (uint32_t)arg;

          /* Move the current read pointer to the specified position
          * and move it to the specified blk group.
          */

          ret = mtdlog_seek_set_blk_group(mtdlog, upriv, grp_count);

          break;
        }

      default:
        ret = -ENOTTY;
    }

  spin_unlock_irqrestore(&mtdlog->lock, flags);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdlog_register
 *
 * Description:
 *   Register a MTD device as a mtdlog device.
 *
 ****************************************************************************/

int mtdlog_register(FAR const char *path, FAR struct mtd_dev_s *mtd)
{
  int ret;
  struct mtd_geometry_s geo;
  FAR struct mtdlog_s *mtdlog;

  if (mtd == NULL || path == NULL)
    {
      ferr("ERROR: Invalid arguments\n");
      return -EINVAL;
    }

  mtdlog = kmm_malloc(sizeof(struct mtdlog_s));
  if (mtdlog == NULL)
    {
      ferr("ERROR: Failed to allocate memory\n");
      return -ENOMEM;
    }

  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY,
                  (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      ferr("ERROR: Failed to get geometry\n");
      kmm_free(mtdlog);
      return ret;
    }

  ret = MTD_IOCTL(mtd, MTDIOC_ERASESTATE,
                  (unsigned long)((uintptr_t)&mtdlog->erasestate));
  if (ret < 0)
    {
      ferr("ERROR: Failed to get erase state\n");
      kmm_free(mtdlog);
      return ret;
    }

  mtdlog->mtd = mtd;
  mtdlog->blocksize = CONFIG_MTDLOG_BLOCKSIZE_MULTIPLE * geo.erasesize;
  mtdlog->nblocks = geo.neraseblocks / CONFIG_MTDLOG_BLOCKSIZE_MULTIPLE;
  mtdlog->progsize = geo.blocksize;
  mtdlog->erasesize = geo.erasesize;
  spin_lock_init(&mtdlog->lock);

  if (mtdlog->blocksize < MTDLOG_MIN_BLKSIZE)
    {
      mtdlog->blocksize = ALIGN_UP(MTDLOG_MIN_BLKSIZE, geo.erasesize);
      mtdlog->nblocks = geo.neraseblocks /
                        (mtdlog->blocksize / geo.erasesize);
    }

  ret = mtdlog_init(mtdlog);
  if (ret < 0)
    {
      ferr("ERROR: Failed to initialize mtdlog\n");
      kmm_free(mtdlog);
      return ret;
    }

  ret = register_driver(path, &g_mtdlog_fops, 0666, mtdlog);
  if (ret < 0)
    {
      ferr("ERROR: Failed to register driver\n");
      kmm_free(mtdlog);
      return ret;
    }

  return 0;
}

/****************************************************************************
 * Name: mtdlog_unregister
 *
 * Description:
 *   Unregister a MTD device as a mtdlog device.
 *
 ****************************************************************************/

int mtdlog_unregister(FAR const char *path)
{
  int ret;
  struct file file;
  FAR struct inode *inode;
  FAR struct mtdlog_s *mtdlog;

  if (path == NULL)
    {
      ferr("ERROR: Invalid arguments\n");
      return -EINVAL;
    }

  ret = file_open(&file, path, O_CLOEXEC);
  if (ret < 0)
    {
      ferr("ERROR: Failed to open file\n");
      return ret;
    }

  inode = file.f_inode;
  mtdlog = inode->i_private;
  kmm_free(mtdlog);

  file_close(&file);

  unregister_driver(path);
  return 0;
}
