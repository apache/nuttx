/****************************************************************************
 * drivers/mtd/mtd_config_fs.c
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
 * NVS: non volatile storage in flash
 *
 * Copyright (c) 2018 Laczen
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/poll.h>
#include <nuttx/crc8.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/configdata.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MIN(a, b)                       ((a) > (b) ? (b) : (a))

/* MASKS AND SHIFT FOR ADDRESSES
 * an address in nvs is an uint32_t where:
 *   high 2 bytes represent the block number
 *   low 2 bytes represent the offset in a block
 */

#define ADDR_BLOCK_MASK                 0xFFFF0000
#define ADDR_BLOCK_SHIFT                16
#define ADDR_OFFS_MASK                  0x0000FFFF

/* We don't want to store all the read content in stack or heap,
 * so we make a buffer to do compare or move.
 */

#define NVS_BUFFER_SIZE                 32

/* If data is written after last ate, and power loss happens,
 * we need to find a clean offset by skipping dirty data.
 * This macro defines how many bytes to skip when dirty data
 * is spotted(may take several skips).
 * Normally 1 byte is okay, such process only happens when
 * nvs is started, and it is acceptable to take some time during
 * starting.
 */

#define NVS_CORRUPT_DATA_SKIP_STEP      1

/* Gc done or close ate has the id of 0xffffffff.
 * We can tell if the ate is special by looking at its id.
 */

#define NVS_SPECIAL_ATE_ID              0xffffffff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Non-volatile Storage File system structure */

struct nvs_fs
{
  FAR struct mtd_dev_s  *mtd;          /* MTD device */
  struct mtd_geometry_s geo;
  uint8_t               erasestate;    /* Erased value */
  uint32_t              ate_wra;       /* Next alloc table entry
                                        * Write address
                                        */
  uint32_t              data_wra;      /* Next data write address */
  uint32_t              step_addr;     /* For traverse */
  mutex_t               nvs_lock;
};

/* Allocation Table Entry */

begin_packed_struct struct nvs_ate
{
  uint32_t id;           /* Data id */
  uint16_t offset;       /* Data offset within block */
  uint16_t len;          /* Data len within block */
  uint16_t key_len;      /* Key string len */
  uint8_t  part;         /* Part of a multipart data - future extension */
  uint8_t  crc8;         /* Crc8 check of the ate entry */
  uint8_t  expired;      /* 0xFF-newest entry, others-old entry */
  uint8_t  reserved[3];  /* For future extension */
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD NVS opeation api */

static int     mtdconfig_open(FAR struct file *filep);
static int     mtdconfig_close(FAR struct file *filep);
static ssize_t mtdconfig_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static int     mtdconfig_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg);
static int     mtdconfig_poll(FAR struct file *filep, FAR struct pollfd *fds,
                              bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_mtdnvs_fops =
{
  mtdconfig_open,  /* Open */
  mtdconfig_close, /* Close */
  mtdconfig_read,  /* Read */
  NULL,            /* Write */
  NULL,            /* Seek */
  mtdconfig_ioctl, /* Ioctl */
  NULL,            /* Truncate */
  mtdconfig_poll   /* Poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL           /* Unlink */
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nvs_fnv_hash
 ****************************************************************************/

static uint32_t nvs_fnv_hash(FAR const uint8_t *input, uint32_t len)
{
  uint32_t i = 0;
  uint32_t hval = 2166136261;

  /* FNV-1 hash each octet in the buffer */

  while (i++ < len)
    {
      /* Multiply by the 32 bit FNV magic prime mod 2^32 */

      hval *= 0x01000193;

      /* Xor the bottom with the current octet */

      hval ^= *input++;
    }

  return hval;
}

/****************************************************************************
 * Name: nvs_flash_wrt
 *
 * Description:
 *   Flash routines, process offset then write.
 *
 ****************************************************************************/

static int nvs_flash_wrt(FAR struct nvs_fs *fs, uint32_t addr,
                         FAR const void *data, size_t len)
{
  off_t offset;
  int ret;

  offset = fs->geo.erasesize * (addr >> ADDR_BLOCK_SHIFT);
  offset += addr & ADDR_OFFS_MASK;

  ret = MTD_WRITE(fs->mtd, offset, len, data);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: nvs_flash_rd
 *
 * Description:
 *   Basic flash read from nvs address.
 *
 ****************************************************************************/

static int nvs_flash_rd(FAR struct nvs_fs *fs, uint32_t addr,
                        FAR void *data, size_t len)
{
  off_t offset;
  int ret;

  offset = fs->geo.erasesize * (addr >> ADDR_BLOCK_SHIFT);
  offset += addr & ADDR_OFFS_MASK;

  ret = MTD_READ(fs->mtd, offset, len, data);
  if (ret < 0)
    {
      return ret;
    }

  return OK;
}

/****************************************************************************
 * Name: nvs_flash_ate_wrt
 *
 * Description:
 *   Allocation entry write.
 *
 ****************************************************************************/

static int nvs_flash_ate_wrt(FAR struct nvs_fs *fs,
                             FAR const struct nvs_ate *entry)
{
  int rc;

  rc = nvs_flash_wrt(fs, fs->ate_wra, entry, sizeof(struct nvs_ate));
  fs->ate_wra -= sizeof(struct nvs_ate);

  return rc;
}

/****************************************************************************
 * Name: nvs_flash_data_wrt
 ****************************************************************************/

static int nvs_flash_data_wrt(FAR struct nvs_fs *fs,
                              FAR const void *data, size_t len)
{
  int rc;

  rc = nvs_flash_wrt(fs, fs->data_wra, data, len);
  fs->data_wra += len;
  finfo("write data done, data_wra=0x%" PRIx32 "\n",
        fs->data_wra);

  return rc;
}

/****************************************************************************
 * Name: nvs_flash_ate_rd
 ****************************************************************************/

static int nvs_flash_ate_rd(FAR struct nvs_fs *fs, uint32_t addr,
                            FAR struct nvs_ate *entry)
{
  return nvs_flash_rd(fs, addr, entry, sizeof(struct nvs_ate));
}

/****************************************************************************
 * Name: nvs_flash_block_cmp
 *
 * Description:
 *   Compares the data in flash at addr to data
 *   in blocks of size NVS_BLOCK_SIZE aligned to fs->write_block_size.
 *   Returns 0 if equal, 1 if not equal, errcode if error.
 *
 ****************************************************************************/

static int nvs_flash_block_cmp(FAR struct nvs_fs *fs, uint32_t addr,
                               FAR const void *data, size_t len)
{
  FAR const uint8_t *data8 = (FAR const uint8_t *)data;
  int rc;
  size_t bytes_to_cmp;
  uint8_t buf[NVS_BUFFER_SIZE];

  while (len > 0)
    {
      bytes_to_cmp = MIN(NVS_BUFFER_SIZE, len);
      rc = nvs_flash_rd(fs, addr, buf, bytes_to_cmp);
      if (rc)
        {
          return rc;
        }

      rc = memcmp(data8, buf, bytes_to_cmp);
      if (rc)
        {
          return 1;
        }

      len -= bytes_to_cmp;
      addr += bytes_to_cmp;
      data8 += bytes_to_cmp;
    }

  return 0;
}

/****************************************************************************
 * Name: nvs_flash_direct_cmp
 *
 * Description:
 *   Compares the data in flash at addr1 and addr2
 *   of len in blocks of size NVS_BLOCK_SIZE aligned to fs->write_block_size.
 *   Returns 0 if equal, 1 if not equal, errcode if error.
 *
 ****************************************************************************/

static int nvs_flash_direct_cmp(FAR struct nvs_fs *fs, uint32_t addr1,
                                uint32_t addr2, size_t len)
{
  int rc;
  size_t bytes_to_cmp;
  uint8_t buf1[NVS_BUFFER_SIZE];
  uint8_t buf2[NVS_BUFFER_SIZE];

  while (len > 0)
    {
      bytes_to_cmp = MIN(NVS_BUFFER_SIZE, len);
      rc = nvs_flash_rd(fs, addr1, buf1, bytes_to_cmp);
      if (rc)
        {
          return rc;
        }

      rc = nvs_flash_rd(fs, addr2, buf2, bytes_to_cmp);
      if (rc)
        {
          return rc;
        }

      rc = memcmp(buf1, buf2, bytes_to_cmp);
      if (rc)
        {
          return 1;
        }

      len -= bytes_to_cmp;
      addr1 += bytes_to_cmp;
      addr2 += bytes_to_cmp;
    }

  return 0;
}

/****************************************************************************
 * Name: nvs_flash_cmp_const
 *
 * Description:
 *   Compares the data in flash at addr to a constant
 *   value. returns 0 if all data in flash is equal to value, 1 if not equal,
 *   errcode if error.
 *
 ****************************************************************************/

static int nvs_flash_cmp_const(FAR struct nvs_fs *fs, uint32_t addr,
                               uint8_t value, size_t len)
{
  int rc;
  size_t bytes_to_cmp;
  uint8_t cmp[NVS_BUFFER_SIZE];

  memset(cmp, value, NVS_BUFFER_SIZE);
  while (len > 0)
    {
      bytes_to_cmp = MIN(NVS_BUFFER_SIZE, len);
      rc = nvs_flash_block_cmp(fs, addr, cmp, bytes_to_cmp);
      if (rc)
        {
          return rc;
        }

      len -= bytes_to_cmp;
      addr += bytes_to_cmp;
    }

  return 0;
}

/****************************************************************************
 * Name: nvs_flash_block_move
 *
 * Description:
 *   Move a block at addr to the current data write
 *   location and updates the data write location.
 *
 ****************************************************************************/

static int nvs_flash_block_move(FAR struct nvs_fs *fs, uint32_t addr,
                                size_t len)
{
  int rc;
  size_t bytes_to_copy;
  uint8_t buf[NVS_BUFFER_SIZE];

  while (len)
    {
      bytes_to_copy = MIN(NVS_BUFFER_SIZE, len);
      rc = nvs_flash_rd(fs, addr, buf, bytes_to_copy);
      if (rc)
        {
          return rc;
        }

      rc = nvs_flash_data_wrt(fs, buf, bytes_to_copy);
      if (rc)
        {
          return rc;
        }

      len -= bytes_to_copy;
      addr += bytes_to_copy;
    }

  return 0;
}

/****************************************************************************
 * Name: nvs_flash_erase_block
 *
 * Description:
 *   Erase a block by first checking it is used and then erasing if required.
 *   Return 0 if OK, errorcode on error.
 *
 ****************************************************************************/

static int nvs_flash_erase_block(FAR struct nvs_fs *fs, uint32_t addr)
{
  int rc;

  finfo("Erasing addr %" PRIx32 "\n", addr);
  rc = MTD_ERASE(fs->mtd, addr >> ADDR_BLOCK_SHIFT, 1);
  if (rc < 0)
    {
      ferr("Erasing failed %d\n", rc);
      return rc;
    }

  return 0;
}

/****************************************************************************
 * Name: nvs_ate_crc8_update
 *
 * Description:
 *   Crc update on allocation entry.
 *
 ****************************************************************************/

static void nvs_ate_crc8_update(FAR struct nvs_ate *entry)
{
  uint8_t ate_crc;

  ate_crc = crc8part((FAR const uint8_t *)entry,
                     offsetof(struct nvs_ate, crc8), 0xff);
  entry->crc8 = ate_crc;
}

/****************************************************************************
 * Name: nvs_ate_crc8_check
 *
 * Description:
 *   Crc check on allocation entry.
 *   Returns 0 if OK, 1 on crc fail.
 *
 ****************************************************************************/

static int nvs_ate_crc8_check(FAR const struct nvs_ate *entry)
{
  uint8_t ate_crc;

  ate_crc = crc8part((FAR const uint8_t *)entry,
                     offsetof(struct nvs_ate, crc8), 0xff);
  if (ate_crc == entry->crc8)
    {
      return 0;
    }

  return 1;
}

/****************************************************************************
 * Name: nvs_ate_cmp_const
 *
 * Description:
 *   Compares an ATE to a constant value. returns 0 if
 *   the whole ATE is equal to value, 1 if not equal.
 *
 ****************************************************************************/

static int nvs_ate_cmp_const(FAR const struct nvs_ate *entry,
                             uint8_t value)
{
  FAR const uint8_t *data8 = (FAR const uint8_t *)entry;
  int i;

  for (i = 0; i < sizeof(struct nvs_ate); i++)
    {
      if (data8[i] != value)
        {
          return 1;
        }
    }

  return 0;
}

/****************************************************************************
 * Name: nvs_ate_valid
 *
 * Description:
 *   Return 1 if crc8 and offset valid, 0 otherwise
 *
 ****************************************************************************/

static int nvs_ate_valid(FAR struct nvs_fs *fs,
                         FAR const struct nvs_ate *entry)
{
  if (nvs_ate_crc8_check(entry) ||
      entry->offset >= (fs->geo.erasesize - sizeof(struct nvs_ate)))
    {
      return 0;
    }

  return 1;
}

/****************************************************************************
 * Name: nvs_close_ate_valid
 *
 * Description:
 *   Validates an block close ate:
 *   A valid block close ate:
 *   - Calid ate.
 *   - Len = 0 and id = 0xFFFFFFFF.
 *   - Offset points to location at ate multiple from block size.
 *   Return 1 if valid, 0 otherwise.
 *
 ****************************************************************************/

static int nvs_close_ate_valid(FAR struct nvs_fs *fs,
                               FAR const struct nvs_ate *entry)
{
  if (!nvs_ate_valid(fs, entry) || entry->len != 0 ||
      entry->id != NVS_SPECIAL_ATE_ID)
    {
      return 0;
    }

  if ((fs->geo.erasesize - entry->offset) % sizeof(struct nvs_ate))
    {
      return 0;
    }

  return 1;
}

/****************************************************************************
 * Name: nvs_flash_wrt_entry
 *
 * Description:
 *   Store an entry in flash
 *
 ****************************************************************************/

static int nvs_flash_wrt_entry(FAR struct nvs_fs *fs, uint32_t id,
                               FAR const uint8_t *key, size_t key_size,
                               FAR const void *data, size_t len)
{
  int rc;
  struct nvs_ate entry;

  memset(&entry, fs->erasestate, sizeof(entry));
  entry.id = id;
  entry.offset = fs->data_wra & ADDR_OFFS_MASK;
  entry.len = len;
  entry.key_len = key_size;

  nvs_ate_crc8_update(&entry);

  /* Let's sew key and data into one, key comes first, then data */

  rc = nvs_flash_data_wrt(fs, key, key_size);
  if (rc)
    {
      ferr("Write key failed, rc=%d\n", rc);
      return rc;
    }

  rc = nvs_flash_data_wrt(fs, data, len);
  if (rc)
    {
      ferr("Write value failed, rc=%d\n", rc);
      return rc;
    }

  rc = nvs_flash_ate_wrt(fs, &entry);
  if (rc)
    {
      ferr("Write ate failed, rc=%d\n", rc);
      return rc;
    }

  return 0;
}

/****************************************************************************
 * Name: nvs_recover_last_ate
 *
 * Description:
 *   If the closing ate is invalid, its offset cannot be trusted and
 *   the last valid ate of the block should instead try to be recovered
 *   by going through all ate's.
 *
 *   Addr should point to the faulty closing ate and will be updated to
 *   the last valid ate. If no valid ate is found it will be left untouched.
 *
 ****************************************************************************/

static int nvs_recover_last_ate(FAR struct nvs_fs *fs,
                                FAR uint32_t *addr)
{
  uint32_t data_end_addr;
  uint32_t ate_end_addr;
  struct nvs_ate end_ate;
  int rc;

  finfo("Recovering last ate from block %" PRIu32 "\n",
        (*addr >> ADDR_BLOCK_SHIFT));

  *addr -= sizeof(struct nvs_ate);
  ate_end_addr = *addr;
  data_end_addr = *addr & ADDR_BLOCK_MASK;
  while (ate_end_addr > data_end_addr)
    {
      rc = nvs_flash_ate_rd(fs, ate_end_addr, &end_ate);
      if (rc)
        {
          return rc;
        }

      if (nvs_ate_valid(fs, &end_ate))
        {
          /* Found a valid ate, update data_end_addr and *addr */

          data_end_addr &= ADDR_BLOCK_MASK;
          data_end_addr += end_ate.offset + end_ate.key_len + end_ate.len;
          *addr = ate_end_addr;
        }

      ate_end_addr -= sizeof(struct nvs_ate);
    }

  return 0;
}

/****************************************************************************
 * Name: nvs_prev_ate
 *
 * Description:
 *   Walking through allocation entry list, from newest to oldest entries.
 *   Read ate from addr, modify addr to the previous ate.
 *
 ****************************************************************************/

static int nvs_prev_ate(FAR struct nvs_fs *fs, FAR uint32_t *addr,
                        FAR struct nvs_ate *ate)
{
  int rc;
  struct nvs_ate close_ate;

  rc = nvs_flash_ate_rd(fs, *addr, ate);
  if (rc)
    {
      return rc;
    }

  *addr += sizeof(struct nvs_ate);
  if (((*addr) & ADDR_OFFS_MASK) !=
      (fs->geo.erasesize - sizeof(struct nvs_ate)))
    {
      return 0;
    }

  /* Last ate in block, do jump to previous block */

  if (((*addr) >> ADDR_BLOCK_SHIFT) == 0)
    {
      *addr += ((fs->geo.neraseblocks - 1) << ADDR_BLOCK_SHIFT);
    }
  else
    {
      *addr -= (1 << ADDR_BLOCK_SHIFT);
    }

  rc = nvs_flash_ate_rd(fs, *addr, &close_ate);
  if (rc)
    {
      return rc;
    }

  rc = nvs_ate_cmp_const(&close_ate, fs->erasestate);

  /* At the end of filesystem */

  if (!rc)
    {
      *addr = fs->ate_wra;
      return 0;
    }

  /* Update the address if the close ate is valid. */

  if (nvs_close_ate_valid(fs, &close_ate))
    {
      *addr &= ADDR_BLOCK_MASK;
      *addr += close_ate.offset;
      return 0;
    }

  /* The close_ate was invalid, `lets find out the last valid ate
   * and point the address to this found ate.
   *
   * Remark: if there was absolutely no valid data in the block *addr
   * is kept at block_end - 2*ate_size, the next read will contain
   * invalid data and continue with a block jump
   */

  return nvs_recover_last_ate(fs, addr);
}

/****************************************************************************
 * Name: nvs_block_advance
 ****************************************************************************/

static void nvs_block_advance(FAR struct nvs_fs *fs, FAR uint32_t *addr)
{
  *addr += (1 << ADDR_BLOCK_SHIFT);
  if ((*addr >> ADDR_BLOCK_SHIFT) == fs->geo.neraseblocks)
    {
      *addr -= (fs->geo.neraseblocks << ADDR_BLOCK_SHIFT);
    }
}

/****************************************************************************
 * Name: nvs_block_close
 *
 * Description:
 *   Allocation entry close (this closes the current block) by writing
 *   offset of last ate to the block end.
 *
 ****************************************************************************/

static int nvs_block_close(FAR struct nvs_fs *fs)
{
  int rc;
  struct nvs_ate close_ate;

  memset(&close_ate, fs->erasestate, sizeof(close_ate));
  close_ate.id = NVS_SPECIAL_ATE_ID;
  close_ate.len = 0;
  close_ate.key_len = 0;
  close_ate.offset =
    (fs->ate_wra + sizeof(struct nvs_ate)) & ADDR_OFFS_MASK;

  fs->ate_wra &= ADDR_BLOCK_MASK;
  fs->ate_wra += fs->geo.erasesize - sizeof(struct nvs_ate);

  nvs_ate_crc8_update(&close_ate);

  rc = nvs_flash_ate_wrt(fs, &close_ate);
  if (rc < 0)
    {
      ferr("Write ate failed, rc=%d\n", rc);
    }

  nvs_block_advance(fs, &fs->ate_wra);

  fs->data_wra = fs->ate_wra & ADDR_BLOCK_MASK;
  finfo("block close, data_wra=0x%" PRIx32 "\n", fs->data_wra);

  return 0;
}

/****************************************************************************
 * Name: nvs_add_gc_done_ate
 ****************************************************************************/

static int nvs_add_gc_done_ate(FAR struct nvs_fs *fs)
{
  struct nvs_ate gc_done_ate;

  finfo("Adding gc done ate at %" PRIx32 "\n", fs->ate_wra & ADDR_OFFS_MASK);
  memset(&gc_done_ate, fs->erasestate, sizeof(gc_done_ate));
  gc_done_ate.id = NVS_SPECIAL_ATE_ID;
  gc_done_ate.len = 0;
  gc_done_ate.key_len = 0;
  gc_done_ate.offset = fs->data_wra & ADDR_OFFS_MASK;
  nvs_ate_crc8_update(&gc_done_ate);

  return nvs_flash_ate_wrt(fs, &gc_done_ate);
}

/****************************************************************************
 * Name: nvs_expire_ate
 ****************************************************************************/

static int nvs_expire_ate(FAR struct nvs_fs *fs, uint32_t addr)
{
  uint8_t expired = 0;

  return nvs_flash_wrt(fs, addr + offsetof(struct nvs_ate, expired),
                       &expired, sizeof(expired));
}

/****************************************************************************
 * Name: nvs_gc
 *
 * Description:
 *   Garbage collection: the address ate_wra has been updated to the new
 *   block that has just been started. The data to gc is in the block
 *   after this new block.
 *
 ****************************************************************************/

static int nvs_gc(FAR struct nvs_fs *fs)
{
  int rc;
  struct nvs_ate close_ate;
  struct nvs_ate gc_ate;
  uint32_t sec_addr;
  uint32_t gc_addr;
  uint32_t gc_prev_addr;
  uint32_t data_addr;
  uint32_t stop_addr;

  finfo("gc: before gc, ate_wra %" PRIx32 "\n", fs->ate_wra);

  sec_addr = (fs->ate_wra & ADDR_BLOCK_MASK);
  nvs_block_advance(fs, &sec_addr);
  gc_addr = sec_addr + fs->geo.erasesize - sizeof(struct nvs_ate);

  finfo("gc: set, sec_addr %" PRIx32 ", gc_addr %" PRIx32 "\n", sec_addr,
        gc_addr);

  /* If the block is not closed don't do gc */

  rc = nvs_flash_ate_rd(fs, gc_addr, &close_ate);
  if (rc < 0)
    {
      /* Flash error */

      return rc;
    }

  rc = nvs_ate_cmp_const(&close_ate, fs->erasestate);
  if (!rc)
    {
      goto gc_done;
    }

  stop_addr = gc_addr - sizeof(struct nvs_ate);

  if (nvs_close_ate_valid(fs, &close_ate))
    {
      gc_addr &= ADDR_BLOCK_MASK;
      gc_addr += close_ate.offset;
    }
  else
    {
      rc = nvs_recover_last_ate(fs, &gc_addr);
      if (rc)
        {
          return rc;
        }
    }

  do
    {
      gc_prev_addr = gc_addr;
      rc = nvs_prev_ate(fs, &gc_addr, &gc_ate);
      if (rc)
        {
          return rc;
        }

      if (gc_ate.expired != fs->erasestate)
        {
          /* Deleted or old ate, ignore it */

          continue;
        }

      if (!nvs_ate_valid(fs, &gc_ate))
        {
          continue;
        }

      if (gc_ate.id != NVS_SPECIAL_ATE_ID)
        {
          /* Copy needed */

          finfo("Moving %" PRIu32 ", key_len %" PRIu16 ", len %" PRIu16 "\n",
                gc_ate.id, gc_ate.key_len, gc_ate.len);

          data_addr = gc_prev_addr & ADDR_BLOCK_MASK;
          data_addr += gc_ate.offset;

          gc_ate.offset = fs->data_wra & ADDR_OFFS_MASK;
          nvs_ate_crc8_update(&gc_ate);

          rc = nvs_flash_block_move(fs, data_addr,
                                    gc_ate.key_len + gc_ate.len);
          if (rc)
            {
              return rc;
            }

          rc = nvs_flash_ate_wrt(fs, &gc_ate);
          if (rc)
            {
              return rc;
            }
        }
    }
  while (gc_prev_addr != stop_addr);

gc_done:
  rc = nvs_add_gc_done_ate(fs);
  if (rc)
    {
      return rc;
    }

  /* Erase the gc'ed block */

  rc = nvs_flash_erase_block(fs, sec_addr);
  if (rc)
    {
      return rc;
    }

  return 0;
}

/****************************************************************************
 * Name: nvs_startup
 ****************************************************************************/

static int nvs_startup(FAR struct nvs_fs *fs)
{
  int rc;
  struct nvs_ate last_ate;
  size_t empty_len;
  uint32_t wlk_addr;
  uint32_t second_addr;
  uint32_t last_addr;
  struct nvs_ate second_ate;

  /* Initialize addr to 0 for the case fs->geo.neraseblocks == 0. This
   * should never happen but both
   * Coverity and GCC believe the contrary.
   */

  uint32_t addr = 0;
  uint16_t i;
  uint16_t closed_blocks = 0;

  fs->ate_wra = 0;
  fs->data_wra = 0;

  /* Check the number of blocks, it should be at least 2. */

  if (fs->geo.neraseblocks < 2)
    {
      ferr("Configuration error - block count\n");
      return -EINVAL;
    }

  /* Get the device geometry. (Casting to uintptr_t first eliminates
   * complaints on some architectures where the sizeof long is different
   * from the size of a pointer).
   */

  rc = MTD_IOCTL(fs->mtd, MTDIOC_GEOMETRY,
                 (unsigned long)((uintptr_t)&(fs->geo)));
  if (rc < 0)
    {
      ferr("ERROR: MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", rc);
      return rc;
    }

  rc = MTD_IOCTL(fs->mtd, MTDIOC_ERASESTATE,
                 (unsigned long)((uintptr_t)&fs->erasestate));
  if (rc < 0)
    {
      ferr("ERROR: MTD ioctl(MTDIOC_ERASESTATE) failed: %d\n", rc);
      return rc;
    }

  /* Step through the blocks to find a open block following
   * a closed block, this is where NVS can write.
   */

  for (i = 0; i < fs->geo.neraseblocks; i++)
    {
      addr = (i << ADDR_BLOCK_SHIFT) +
        (uint16_t)(fs->geo.erasesize - sizeof(struct nvs_ate));
      rc = nvs_flash_cmp_const(fs, addr, fs->erasestate,
                               sizeof(struct nvs_ate));
      fwarn("rc=%d\n", rc);
      if (rc)
        {
          /* Closed block */

          closed_blocks++;
          nvs_block_advance(fs, &addr);
          rc = nvs_flash_cmp_const(fs, addr, fs->erasestate,
                                   sizeof(struct nvs_ate));
          if (!rc)
            {
              /* Open block */

              break;
            }
        }

      fwarn("i=%" PRIu16 ", closed_blocks=%" PRIu16 ", addr=0x%" PRIx32 "\n",
            i, closed_blocks, addr);
    }

  /* All blocks are closed, this is not a nvs fs */

  if (closed_blocks == fs->geo.neraseblocks)
    {
      return -EDEADLK;
    }

  if (i == fs->geo.neraseblocks)
    {
      /* None of the blocks where closed, in most cases we can set
       * the address to the first block, except when there are only
       * two blocks. Then we can only set it to the first block if
       * the last block contains no ate's. So we check this first
       */

      rc = nvs_flash_cmp_const(fs, addr - sizeof(struct nvs_ate),
                               fs->erasestate, sizeof(struct nvs_ate));
      if (!rc)
        {
          /* Empty ate */

          nvs_block_advance(fs, &addr);
        }
    }

  /* Addr contains address of closing ate in the most recent block,
   * search for the last valid ate using the recover_last_ate routine
   */

  rc = nvs_recover_last_ate(fs, &addr);
  if (rc)
    {
      return rc;
    }

  /* Addr contains address of the last valid ate in the most recent block
   * search for the first ate containing all cells erased, in the process
   * also update fs->data_wra.
   */

  fs->ate_wra = addr;
  fs->data_wra = addr & ADDR_BLOCK_MASK;
  finfo("recovered ate ate_wra=0x%" PRIx32 ", data_wra=0x%" PRIx32 "\n",
        fs->ate_wra, fs->data_wra);

  while (fs->ate_wra >= fs->data_wra)
    {
      rc = nvs_flash_ate_rd(fs, fs->ate_wra, &last_ate);
      if (rc)
        {
          return rc;
        }

      rc = nvs_ate_cmp_const(&last_ate, fs->erasestate);
      if (!rc)
        {
          /* Found 0xff empty location */

          break;
        }

      if (nvs_ate_valid(fs, &last_ate))
        {
          /* Complete write of ate was performed */

          fs->data_wra = addr & ADDR_BLOCK_MASK;
          fs->data_wra += last_ate.offset + last_ate.key_len +
            last_ate.len;
          finfo("recovered data_wra=0x%" PRIx32 "\n", fs->data_wra);
        }

      fs->ate_wra -= sizeof(struct nvs_ate);
    }

  /* If the block after the write block is not empty, gc was interrupted
   * we might need to restart gc if it has not yet finished. Otherwise
   * just erase the block.
   * When gc needs to be restarted, first erase the block otherwise the
   * data might not fit into the block.
   */

  addr = fs->ate_wra & ADDR_BLOCK_MASK;
  nvs_block_advance(fs, &addr);
  rc = nvs_flash_cmp_const(fs, addr, fs->erasestate, fs->geo.erasesize);
  if (rc < 0)
    {
      return rc;
    }

  if (rc)
    {
      /* The block after fs->ate_wrt is not empty, look for a marker
       * (gc_done_ate) that indicates that gc was finished.
       */

      bool gc_done_marker = false;
      struct nvs_ate gc_done_ate;

      addr = fs->ate_wra + sizeof(struct nvs_ate);
      while ((addr & ADDR_OFFS_MASK) <
             (fs->geo.erasesize - sizeof(struct nvs_ate)))
        {
          rc = nvs_flash_ate_rd(fs, addr, &gc_done_ate);
          if (rc)
            {
              return rc;
            }

          if (nvs_ate_valid(fs, &gc_done_ate) &&
              (gc_done_ate.id == NVS_SPECIAL_ATE_ID) &&
              (gc_done_ate.len == 0))
            {
              gc_done_marker = true;
              break;
            }

          addr += sizeof(struct nvs_ate);
        }

      if (gc_done_marker)
        {
          /* Erase the next block */

          fwarn("GC Done marker found\n");
          addr = fs->ate_wra & ADDR_BLOCK_MASK;
          nvs_block_advance(fs, &addr);
          rc = nvs_flash_erase_block(fs, addr);
          goto end;
        }

      fwarn("No GC Done marker found: restarting gc\n");
      rc = nvs_flash_erase_block(fs, fs->ate_wra);
      if (rc)
        {
          return rc;
        }

      fs->ate_wra &= ADDR_BLOCK_MASK;
      fs->ate_wra += (fs->geo.erasesize - 2 * sizeof(struct nvs_ate));
      fs->data_wra = (fs->ate_wra & ADDR_BLOCK_MASK);
      finfo("GC when data_wra=0x%" PRIx32 "\n", fs->data_wra);
      rc = nvs_gc(fs);
      goto end;
    }

  /* Possible data write after last ate write, update data_wra */

  while (fs->ate_wra > fs->data_wra)
    {
      empty_len = fs->ate_wra - fs->data_wra;

      rc = nvs_flash_cmp_const(fs, fs->data_wra, fs->erasestate, empty_len);
      if (rc < 0)
        {
          return rc;
        }

      if (!rc)
        {
          break;
        }

      fs->data_wra += NVS_CORRUPT_DATA_SKIP_STEP;
      finfo("update for powerloss data write, data_wra=0x%" PRIx32 "\n",
            fs->data_wra);
    }

  /* If the ate_wra is pointing to the first ate write location in a
   * block and data_wra is not 0, erase the block as it contains no
   * valid data (this also avoids closing a block without any data).
   */

  if (((fs->ate_wra + 2 * sizeof(struct nvs_ate)) == fs->geo.erasesize) &&
      (fs->data_wra != (fs->ate_wra & ADDR_BLOCK_MASK)))
    {
      rc = nvs_flash_erase_block(fs, fs->ate_wra);
      if (rc)
        {
          return rc;
        }

      fs->data_wra = fs->ate_wra & ADDR_BLOCK_MASK;
      finfo("erase due to no data, data_wra=0x%" PRIx32 "\n",
            fs->data_wra);
    }

  /* Check if there exists an old entry with the same id and key
   * as the newest entry.
   * If so, power loss occured before writing the old entry id as expired.
   * We need to set old entry expired.
   */

  wlk_addr = fs->ate_wra;
  while (1)
    {
      last_addr = wlk_addr;
      rc = nvs_prev_ate(fs, &wlk_addr, &last_ate);
      if (rc)
        {
          return rc;
        }

      /* Skip last one */

      if (wlk_addr == fs->ate_wra)
        {
          break;
        }

      if (nvs_ate_valid(fs, &last_ate)
          && (last_ate.id != NVS_SPECIAL_ATE_ID))
        {
          finfo("ate found at 0x%" PRIx32 ", id %" PRIu32 ", "
                "key_len %" PRIu16 ", offset %" PRIu16 "\n",
                last_addr, last_ate.id, last_ate.key_len, last_ate.offset);
          while (1)
            {
              second_addr = wlk_addr;
              rc = nvs_prev_ate(fs, &wlk_addr, &second_ate);
              if (rc)
                {
                  return rc;
                }

              if (nvs_ate_valid(fs, &second_ate)
                  && second_ate.id == last_ate.id
                  && second_ate.expired == fs->erasestate)
                {
                  finfo("same id at 0x%" PRIx32 ", key_len %" PRIu16 ", "
                        "offset %" PRIu16 "\n",
                        second_addr, second_ate.key_len, second_ate.offset);
                  if ((second_ate.key_len == last_ate.key_len) &&
                      !nvs_flash_direct_cmp(fs,
                                            (last_addr & ADDR_BLOCK_MASK) +
                                            last_ate.offset,
                                            (second_addr & ADDR_BLOCK_MASK) +
                                            second_ate.offset,
                                            last_ate.key_len))
                    {
                      finfo("old ate found at 0x%" PRIx32 "\n", second_addr);
                      rc = nvs_expire_ate(fs, second_addr);
                      if (rc < 0)
                        {
                          ferr("expire ate failed, addr %" PRIx32 "\n",
                               second_addr);
                          return rc;
                        }

                      goto end;
                    }
                  else
                    {
                      fwarn("hash conflict\n");
                    }
                }

              if (wlk_addr == fs->ate_wra)
                {
                  goto end;
                }
            }
        }
    }

end:
  /* If the block is empty, add a gc done ate to avoid having insufficient
   * space when doing gc.
   */

  if ((!rc) && ((fs->ate_wra & ADDR_OFFS_MASK) ==
      (fs->geo.erasesize - 2 * sizeof(struct nvs_ate))))
    {
      rc = nvs_add_gc_done_ate(fs);
    }

  finfo("%" PRIu32 " Eraseblocks of %" PRIu32 " bytes\n",
        fs->geo.neraseblocks, fs->geo.erasesize);
  finfo("alloc wra: %" PRIu32 ", 0x%" PRIx32 "\n",
        (fs->ate_wra >> ADDR_BLOCK_SHIFT), (fs->ate_wra & ADDR_OFFS_MASK));
  finfo("data wra: %" PRIu32 ", 0x%" PRIx32 "\n",
        (fs->data_wra >> ADDR_BLOCK_SHIFT), (fs->data_wra & ADDR_OFFS_MASK));

  return rc;
}

/****************************************************************************
 * Name: nvs_read_entry
 *
 * Description:
 *   Read An entry from the file system. But expired ones will return
 *   -ENOENT.
 *
 * Input Parameters:
 *   fs       - Pointer to file system.
 *   key      - Key of the entry to be read.
 *   key_size - Size of key.
 *   data     - Pointer to data buffer.
 *   len      - Number of bytes to be read.
 *   ate_addr - The addr of found ate.
 *
 * Returned Value:
 *   Number of bytes read. On success, it will be equal to the number
 *   of bytes requested to be read. When the return value is larger than the
 *   number of bytes requested to read this indicates not all bytes were
 *   read, and more data is available. On error returns -ERRNO code.
 *
 ****************************************************************************/

static ssize_t nvs_read_entry(FAR struct nvs_fs *fs, FAR const uint8_t *key,
                size_t key_size, FAR void *data, size_t len,
                FAR uint32_t *ate_addr)
{
  int rc;
  uint32_t wlk_addr;
  uint32_t rd_addr;
  uint32_t hist_addr;
  struct nvs_ate wlk_ate;
  uint32_t hash_id;

  hash_id = nvs_fnv_hash(key, key_size) % 0xfffffffd + 1;
  wlk_addr = fs->ate_wra;
  rd_addr = wlk_addr;

  do
    {
      rd_addr = wlk_addr;
      hist_addr = wlk_addr;
      rc = nvs_prev_ate(fs, &wlk_addr, &wlk_ate);
      if (rc)
        {
          ferr("Walk to previous ate failed, rc=%d\n", rc);
          return rc;
        }

      if ((wlk_ate.id == hash_id) && (nvs_ate_valid(fs, &wlk_ate)))
        {
          if ((wlk_ate.key_len == key_size)
              && (!nvs_flash_block_cmp(fs,
              (rd_addr & ADDR_BLOCK_MASK) + wlk_ate.offset, key, key_size)))
            {
              /* It is old or deleted, return -ENOENT */

              if (wlk_ate.expired != fs->erasestate)
                {
                  return -ENOENT;
                }
              break;
            }
          else
            {
              fwarn("hash conflict\n");
            }
        }

      if (wlk_addr == fs->ate_wra)
        {
          return -ENOENT;
        }
    }
  while (true);

  if (data)
    {
      rd_addr &= ADDR_BLOCK_MASK;
      rd_addr += wlk_ate.offset + wlk_ate.key_len;
      rc = nvs_flash_rd(fs, rd_addr, data,
                        MIN(len, wlk_ate.len));
      if (rc)
        {
          ferr("Data read failed, rc=%d\n", rc);
          return rc;
        }
    }

  if (ate_addr)
    {
      *ate_addr = hist_addr;
    }

  return wlk_ate.len;
}

/****************************************************************************
 * Name: nvs_write
 *
 * Description:
 *   Write an entry to the file system.
 *
 * Input Parameters:
 *   fs    - Pointer to file system.
 *   pdata - Pointer to data buffer.
 *
 * Returned Value:
 *   Number of bytes written. On success, it will be equal to the
 *   number of bytes requested to be written. On error returns -ERRNO code.
 *
 ****************************************************************************/

static ssize_t nvs_write(FAR struct nvs_fs *fs,
                         FAR struct config_data_s *pdata)
{
  int rc;
  int gc_count;
  size_t data_size;
  size_t key_size;
  struct nvs_ate wlk_ate;
  uint32_t wlk_addr;
  uint32_t rd_addr;
  uint32_t hist_addr;
  uint16_t required_space = 0;
  bool prev_found = false;
  uint32_t hash_id;
  uint16_t block_to_write_befor_gc;

#ifdef CONFIG_MTD_CONFIG_NAMED
  FAR const uint8_t *key;
#else
  uint8_t key[sizeof(pdata->id) + sizeof(pdata->instance)];
#endif

#ifdef CONFIG_MTD_CONFIG_NAMED
  key = (FAR const uint8_t *)pdata->name;
  key_size = strlen(pdata->name) + 1;
#else
  memcpy(key, &pdata->id, sizeof(pdata->id));
  memcpy(key + sizeof(pdata->id), &pdata->instance, sizeof(pdata->instance));
  key_size = sizeof(pdata->id) + sizeof(pdata->instance);
#endif

  /* Data now contains input data and input key, input key first. */

  data_size = key_size + pdata->len;

  /* The maximum data size is block size - 3 ate
   * where: 1 ate for data, 1 ate for block close, 1 ate for gc done.
   */

  finfo("key_size=%zu, len=%zu\n", key_size, pdata->len);

  if ((data_size > (fs->geo.erasesize - 3 * sizeof(struct nvs_ate))) ||
      ((pdata->len > 0) && (pdata->configdata == NULL)))
    {
      return -EINVAL;
    }

  /* Calc hash id of key. */

  hash_id = nvs_fnv_hash(key, key_size) % 0xfffffffd + 1;

  /* Find latest entry with same id. */

  wlk_addr = fs->ate_wra;

  while (1)
    {
      rd_addr = wlk_addr;
      hist_addr = wlk_addr;
      rc = nvs_prev_ate(fs, &wlk_addr, &wlk_ate);
      if (rc)
        {
          return rc;
        }

      if ((wlk_ate.id == hash_id) && (nvs_ate_valid(fs, &wlk_ate)))
        {
          if ((wlk_ate.key_len == key_size)
              && !nvs_flash_block_cmp(fs,
                                      (rd_addr & ADDR_BLOCK_MASK) +
                                      wlk_ate.offset, key, key_size))
            {
              prev_found = true;
              break;
            }
          else
            {
              fwarn("hash conflict\n");
            }
        }

      if (wlk_addr == fs->ate_wra)
        {
          break;
        }
    }

  if (prev_found)
    {
      finfo("Previous found\n");

      /* Previous entry found. */

      rd_addr &= ADDR_BLOCK_MASK;

      if (pdata->len == 0)
        {
          /* If prev ate is expired, it is deleted. */

          if (wlk_ate.expired != fs->erasestate)
            {
              /* Skip delete entry as it is already the
               * last one.
               */

              return 0;
            }
          else
            {
              rc = nvs_expire_ate(fs, hist_addr);
              if (rc < 0)
                {
                  ferr("expire ate failed, addr %" PRIx32 "\n", hist_addr);
                  return rc;
                }

              /* Delete now requires no extra space, so skip write and gc. */

              finfo("nvs_delete success\n");
              return 0;
            }
        }
      else if (pdata->len == wlk_ate.len &&
               wlk_ate.expired == fs->erasestate)
        {
          /* Do not try to compare if lengths are not equal
           * or prev one is deleted.
           * Compare the data and if equal return 0.
           */

          rd_addr += wlk_ate.offset + wlk_ate.key_len;
          rc = nvs_flash_block_cmp(fs, rd_addr, pdata->configdata,
                                   pdata->len);
          if (rc <= 0)
            {
              return rc;
            }
        }
    }
  else
    {
      /* Skip delete entry for non-existing entry. */

      if (pdata->len == 0)
        {
          return 0;
        }
    }

  /* Leave space for gc_done ate */

  required_space = data_size + sizeof(struct nvs_ate);
  gc_count = 0;
  block_to_write_befor_gc = fs->ate_wra >> ADDR_BLOCK_SHIFT;
  while (1)
    {
      if (gc_count == fs->geo.neraseblocks)
        {
          /* Gc'ed all blocks, no extra space will be created
           * by extra gc.
           */

          return -ENOSPC;
        }

      if (fs->ate_wra >= fs->data_wra + required_space)
        {
          /* Nvs is changed after gc, we will look for the old ate.
           */

          if (prev_found && wlk_ate.expired == fs->erasestate)
            {
              finfo("prev entry exists, search for it\n");

              /* If gc touched second latest ate, search for it again */

              if (gc_count >= fs->geo.neraseblocks - 1 -
                  (block_to_write_befor_gc + fs->geo.neraseblocks -
                  (hist_addr >> ADDR_BLOCK_SHIFT))
                  % fs->geo.neraseblocks)
                {
                  rc = nvs_read_entry(fs, key, key_size, NULL, 0,
                                     &hist_addr);
                  finfo("relocate for prev entry, %" PRIx32 ", "
                        "rc %d\n",
                        hist_addr, rc);
                  if (rc < 0)
                    {
                      ferr("read prev entry failed\n");
                      return rc;
                    }
                }
            }

          finfo("Write entry, ate_wra=0x%" PRIx32 ", "
                "data_wra=0x%" PRIx32 "\n",
                fs->ate_wra, fs->data_wra);
          rc = nvs_flash_wrt_entry(fs, hash_id, key, key_size,
                                   pdata->configdata, pdata->len);
          if (rc)
            {
              fwarn("Write entry failed\n");
              return rc;
            }

          finfo("Write entry success\n");

          /* Expiring the old ate if exists.
           * After this operation, only the latest ate is valid.
           * Expire the old one only if it is not deleted before(Or it is
           * already expired)
           */

          if (prev_found && wlk_ate.expired == fs->erasestate)
            {
              rc = nvs_expire_ate(fs, hist_addr);
              finfo("expir prev entry, %" PRIx32 ", rc %d\n",
                    hist_addr, rc);
              if (rc < 0)
                {
                  ferr("expire ate failed, addr %" PRIx32 "\n",
                       hist_addr);
                  return rc;
                }
            }

          break;
        }

      rc = nvs_block_close(fs);
      if (rc)
        {
          return rc;
        }

      rc = nvs_gc(fs);
      if (rc)
        {
          return rc;
        }

      gc_count++;
      finfo("Gc count=%d\n", gc_count);
    }

  finfo("nvs_write success\n");
  return 0;
}

/****************************************************************************
 * Name: nvs_delete
 *
 * Description:
 *   Delete an entry from the file system.
 *
 * Input Parameters:
 *   fs    - Pointer to file system.
 *   pdata - Pointer to data buffer.
 *
 * Returned Value:
 *   0 on success, -ERRNO errno code if error.
 *
 ****************************************************************************/

static int nvs_delete(FAR struct nvs_fs *fs, FAR struct config_data_s *pdata)
{
  /* If user wants to operate /dev/config directly.
   * Set len=0 to trigger delete, so that user doesn't need to do that.
   */

  pdata->len = 0;
  return nvs_write(fs, pdata);
}

/****************************************************************************
 * Name: nvs_read
 *
 * Description:
 *   Read an entry from the file system.
 *
 * Input Parameters:
 *   fs    - Pointer to file system.
 *   pdata - Pointer to data buffer.
 *
 * Returned Value:
 *   0 on success, -ERRNO errno code if error.
 *
 ****************************************************************************/

static ssize_t nvs_read(FAR struct nvs_fs *fs,
                        FAR struct config_data_s *pdata)
{
  size_t key_size;
  ssize_t ret;

#ifdef CONFIG_MTD_CONFIG_NAMED
  FAR const uint8_t *key;
#else
  uint8_t key[sizeof(pdata->id) + sizeof(pdata->instance)];
#endif

  if (pdata == NULL || pdata->len == 0)
    {
      return -EINVAL;
    }

#ifdef CONFIG_MTD_CONFIG_NAMED
  key = (FAR const uint8_t *)pdata->name;
  key_size = strlen(pdata->name) + 1;
#else
  memcpy(key, &pdata->id, sizeof(pdata->id));
  memcpy(key + sizeof(pdata->id), &pdata->instance, sizeof(pdata->instance));
  key_size = sizeof(pdata->id) + sizeof(pdata->instance);
#endif

  ret = nvs_read_entry(fs, key, key_size, pdata->configdata, pdata->len,
                      NULL);
  if (ret > 0)
    {
      pdata->len = ret;
      return 0;
    }

  return ret;
}

/****************************************************************************
 * Name: nvs_next
 *
 * Description:
 *   Get the next KV in database.
 *
 * Input Parameters:
 *   fs    - Pointer to file system.
 *   pdata - Pointer to data buffer.
 *   first - true if we are reading the first KV.
 *
 * Returned Value:
 *   0 on success, -ERRNO errno code if error.
 *
 ****************************************************************************/

static int nvs_next(FAR struct nvs_fs *fs,
                    FAR struct config_data_s *pdata, bool first)
{
  int rc;
  struct nvs_ate step_ate;
  uint32_t rd_addr;

#ifdef CONFIG_MTD_CONFIG_NAMED
  FAR uint8_t *key = (FAR uint8_t *)pdata->name;
#else
  uint8_t key[sizeof(pdata->id) + sizeof(pdata->instance)];
#endif

  if (first)
    {
      fs->step_addr = fs->ate_wra;
    }
  else
    {
      if (fs->step_addr == fs->ate_wra)
        {
          return -ENOENT;
        }
    }

  do
    {
      rd_addr = fs->step_addr;
      rc = nvs_prev_ate(fs, &(fs->step_addr), &step_ate);
      if (rc)
        {
          return rc;
        }

      if (nvs_ate_valid(fs, &step_ate)
          && step_ate.id != NVS_SPECIAL_ATE_ID
          && step_ate.expired == fs->erasestate)
        {
          break;
        }

      if (fs->step_addr == fs->ate_wra)
        {
          return -ENOENT;
        }
    }
  while (true);

#ifdef CONFIG_MTD_CONFIG_NAMED
  rc = nvs_flash_rd(fs, (rd_addr & ADDR_BLOCK_MASK) + step_ate.offset,
                    key, MIN(step_ate.key_len, CONFIG_MTD_CONFIG_NAME_LEN));
  if (rc)
    {
      ferr("Key read failed, rc=%d\n", rc);
      return rc;
    }

  key[CONFIG_MTD_CONFIG_NAME_LEN - 1] = 0;
#else
  rc = nvs_flash_rd(fs, (rd_addr & ADDR_BLOCK_MASK) + step_ate.offset,
                    key, MIN(sizeof(key), step_ate.key_len));
  if (rc)
    {
      ferr("Key read failed, rc=%d\n", rc);
      return rc;
    }

  memcpy(pdata->id, key, sizeof(pdata->id));
  memcpy(pdata->instance, key + sizeof(pdata->id), sizeof(pdata->instance));
#endif

  rc = nvs_flash_rd(fs, (rd_addr & ADDR_BLOCK_MASK) + step_ate.offset +
                    step_ate.key_len, pdata->configdata,
                    MIN(pdata->len, step_ate.len));
  if (rc)
    {
      ferr("Value read failed, rc=%d\n", rc);
      return rc;
    }

  pdata->len = MIN(pdata->len, step_ate.len);
  return OK;
}

/****************************************************************************
 * Name: mtdconfig_open
 ****************************************************************************/

static int mtdconfig_open(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mtdconfig_close
 ****************************************************************************/

static int mtdconfig_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: mtdconfig_read
 ****************************************************************************/

static ssize_t mtdconfig_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  return -ENOTSUP;
}

/****************************************************************************
 * Name: mtdconfig_ioctl
 ****************************************************************************/

static int mtdconfig_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct nvs_fs *fs = (FAR struct nvs_fs *)inode->i_private;
  FAR struct config_data_s *pdata = (FAR struct config_data_s *)arg;
  int ret = -ENOTTY;

  ret = nxmutex_lock(&fs->nvs_lock);
  if (ret < 0)
    {
      return ret;
    }

  switch (cmd)
    {
      case CFGDIOC_GETCONFIG:

        /* Read a nvs item. */

        ret = nvs_read(fs, pdata);
        break;

      case CFGDIOC_SETCONFIG:

        /* Write a nvs item. */

        ret = nvs_write(fs, pdata);
        break;

      case CFGDIOC_DELCONFIG:

        /* Delete a nvs item. */

        ret = nvs_delete(fs, pdata);
        break;

      case CFGDIOC_FIRSTCONFIG:

        /* Get the first item. */

        ret = nvs_next(fs, pdata, true);
        break;

      case CFGDIOC_NEXTCONFIG:

        /* Get the next item. */

        ret = nvs_next(fs, pdata, false);
        break;

      case MTDIOC_BULKERASE:

        /* Call the MTD's ioctl for this. */

        ret = MTD_IOCTL(fs->mtd, cmd, arg);
        if (ret >= 0)
          {
            ret = nvs_startup(fs);
          }

        break;
    }

  nxmutex_unlock(&fs->nvs_lock);
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_poll
 ****************************************************************************/

static int mtdconfig_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  if (setup)
    {
      poll_notify(&fds, 1, POLLIN | POLLOUT);
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdconfig_register
 *
 * Description:
 *   Register a /dev/config device backed by an fail-safe NVS.
 *
 ****************************************************************************/

int mtdconfig_register(FAR struct mtd_dev_s *mtd)
{
  int ret;
  FAR struct nvs_fs *fs;

  fs = (FAR struct nvs_fs *)kmm_malloc(sizeof(struct nvs_fs));
  if (fs == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the mtdnvs device structure */

  fs->mtd = mtd;
  ret = nxmutex_init(&fs->nvs_lock);
  if (ret < 0)
    {
      ferr("ERROR: nxmutex_init failed: %d\n", ret);
      goto errout;
    }

  ret = nvs_startup(fs);
  if (ret < 0)
    {
      ferr("ERROR: nvs_init failed: %d\n", ret);
      goto mutex_err;
    }

  ret = register_driver("/dev/config", &g_mtdnvs_fops, 0666, fs);
  if (ret < 0)
    {
      ferr("ERROR: register mtd config failed: %d\n", ret);
      goto mutex_err;
    }

  return ret;

mutex_err:
  nxmutex_destroy(&fs->nvs_lock);

errout:
  kmm_free(fs);
  return ret;
}

/****************************************************************************
 * Name: mtdconfig_unregister
 *
 * Description:
 *   Unregister a /dev/config device backed by an fail-safe NVS.
 *
 ****************************************************************************/

int mtdconfig_unregister(void)
{
  int ret;
  struct file file;
  FAR struct inode *inode;
  FAR struct nvs_fs *fs;

  ret = file_open(&file, "/dev/config", 0);
  if (ret < 0)
    {
      ferr("ERROR: open /dev/config failed: %d\n", ret);
      return ret;
    }

  inode = file.f_inode;
  fs = (FAR struct nvs_fs *)inode->i_private;
  nxmutex_destroy(&fs->nvs_lock);
  kmm_free(fs);
  file_close(&file);
  unregister_driver("/dev/config");

  return OK;
}

