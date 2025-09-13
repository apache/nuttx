/****************************************************************************
 * drivers/mtd/mtd_config_nvs.c
 *
 * SPDX-License-Identifier: Apache-2.0
 * SPDX-FileCopyrightText: 2018 Laczen
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

#include <sys/param.h>
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

/* MASKS AND SHIFT FOR ADDRESSES
 * an address in nvs is an uint32_t where:
 *   high 2 bytes represent the block number
 *   low 2 bytes represent the offset in a block
 */

#define NVS_ADDR_BLOCK_MASK             0xFFFF0000
#define NVS_ADDR_BLOCK_SHIFT            16
#define NVS_ADDR_OFFS_MASK              0x0000FFFF

#define NVS_CACHE_NO_ADDR               0xffffffff

#define NVS_HASH_INITIAL_VALUE          2166136261

#if CONFIG_MTD_CONFIG_BUFFER_SIZE > 0
#  define NVS_BUFFER_SIZE(fs)           CONFIG_MTD_CONFIG_BUFFER_SIZE
#  define NVS_ATE(name, size) \
    char name##_buf[CONFIG_MTD_CONFIG_BUFFER_SIZE]; \
    FAR struct nvs_ate *name = (FAR struct nvs_ate *)name##_buf
#else
#  define NVS_BUFFER_SIZE(fs)           nvs_align_up(fs, 32)
#  define NVS_ATE(name, size) \
    char name##_buf[size]; \
    FAR struct nvs_ate *name = (FAR struct nvs_ate *)name##_buf
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Non-volatile Storage File system structure */

struct nvs_fs
{
  FAR struct mtd_dev_s  *mtd;          /* MTD device */
  uint32_t              progsize;      /* Size of one read/write block */
  uint32_t              blocksize;     /* Size of one nvs block */
  uint32_t              nblocks;       /* Number of nvs blocks */
  uint8_t               erasestate;    /* Erased value */
  uint32_t              ate_wra;       /* Next alloc table entry
                                        * Write address
                                        */
  uint32_t              data_wra;      /* Next data write address */
  uint32_t              step_addr;     /* For traverse */
  mutex_t               nvs_lock;
  FAR struct pollfd     *fds;
  pollevent_t           events;
#if CONFIG_MTD_CONFIG_CACHE_SIZE > 0
  uint32_t              cache[CONFIG_MTD_CONFIG_CACHE_SIZE];
#endif
};

/* Allocation Table Entry */

begin_packed_struct struct nvs_ate
{
  uint32_t id;           /* Data id */
  uint16_t offset;       /* Data offset within block */
  uint16_t len;          /* Data len within block */
  uint16_t key_len;      /* Key string len */
  uint8_t  data_crc8;    /* Crc8 check of the data */
  uint8_t  crc8;         /* Crc8 check of the ate entry */
  uint8_t  expired[0];
} end_packed_struct;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD NVS operation api */

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
  NULL,            /* Mmap */
  mtdconfig_poll   /* Poll */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nvs_cache_index
 ****************************************************************************/

#if CONFIG_MTD_CONFIG_CACHE_SIZE > 0
static inline size_t nvs_cache_index(uint32_t id)
{
  return id % CONFIG_MTD_CONFIG_CACHE_SIZE;
}

/****************************************************************************
 * Name: nvs_invalid_cache
 ****************************************************************************/

static void nvs_invalid_cache(struct nvs_fs *fs, uint32_t sector)
{
  FAR uint32_t *cache_entry = fs->cache;
  FAR const uint32_t *cache_end =
                  &fs->cache[CONFIG_MTD_CONFIG_CACHE_SIZE];

  for (; cache_entry < cache_end; ++cache_entry)
    {
      if ((*cache_entry >> NVS_ADDR_BLOCK_SHIFT) == sector)
        {
          *cache_entry = NVS_CACHE_NO_ADDR;
        }
    }
}
#endif /* CONFIG_MTD_CONFIG_CACHE_SIZE */

/****************************************************************************
 * Name: nvs_fnv_hash_part
 ****************************************************************************/

static uint32_t nvs_fnv_hash_part(FAR const void *input, uint32_t len,
                                  uint32_t hval)
{
  FAR const uint8_t *key8 = (FAR const uint8_t *)input;

  /* FNV-1 hash each octet in the buffer */

  while (len-- > 0)
    {
      /* Multiply by the 32 bit FNV magic prime mod 2^32 */

      hval *= 0x01000193;

      /* Xor the bottom with the current octet */

      hval ^= *key8++;
    }

  return hval;
}

/****************************************************************************
 * Name: nvs_fnv_hash
 ****************************************************************************/

static uint32_t nvs_fnv_hash(FAR const void *input, uint32_t len)
{
  return nvs_fnv_hash_part(input, len, NVS_HASH_INITIAL_VALUE);
}

/****************************************************************************
 * Name: nvs_fnv_hash_id
 ****************************************************************************/

static uint32_t nvs_fnv_hash_id(uint32_t hash)
{
  return hash % 0xfffffffd + 1;
}

/****************************************************************************
 * Name: nvs_align_up
 ****************************************************************************/

static inline size_t nvs_align_up(FAR struct nvs_fs *fs, size_t len)
{
  return (len + (fs->progsize - 1)) & ~(fs->progsize - 1);
}

/****************************************************************************
 * Name: nvs_align_down
 ****************************************************************************/

static inline size_t nvs_align_down(FAR struct nvs_fs *fs, size_t len)
{
  return len & ~(fs->progsize - 1);
}

/****************************************************************************
 * Name: nvs_ate_size
 ****************************************************************************/

static inline size_t nvs_ate_size(FAR struct nvs_fs *fs)
{
  /* Stay compatible with situation which align byte be 1 */

  if (fs->progsize == 1)
    {
      return sizeof(struct nvs_ate) + 4;
    }

  return nvs_align_up(fs, sizeof(struct nvs_ate)) + fs->progsize;
}

/****************************************************************************
 * Name: nvs_ate_expired
 ****************************************************************************/

static inline bool nvs_ate_expired(FAR struct nvs_fs *fs,
                                   FAR struct nvs_ate *entry)
{
  return entry->expired[nvs_align_up(fs, sizeof(*entry)) - sizeof(*entry)] !=
         fs->erasestate;
}

/****************************************************************************
 * Name: nvs_special_ate_id
 *
 * Description:
 *   Gc done or close ate has the id of 0xffffffff.
 *   We can tell if the ate is special by looking at its id.
 *
 ****************************************************************************/

static inline uint32_t nvs_special_ate_id(FAR struct nvs_fs *fs)
{
  return (fs->erasestate << 24) | (fs->erasestate << 16) |
         (fs->erasestate << 8) | fs->erasestate;
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
  int rc;

  offset = fs->blocksize * (addr >> NVS_ADDR_BLOCK_SHIFT);
  offset += addr & NVS_ADDR_OFFS_MASK;

#ifdef CONFIG_MTD_BYTE_WRITE
  rc = MTD_WRITE(fs->mtd, offset, len, data);
#else
  rc = MTD_BWRITE(fs->mtd, offset / fs->progsize, len / fs->progsize, data);
#endif

  return rc < 0 ? rc : 0;
}

/****************************************************************************
 * Name: nvs_flash_brd
 ****************************************************************************/

static int nvs_flash_brd(FAR struct nvs_fs *fs, off_t offset,
                         FAR void *data, size_t len)
{
  int rc;

#ifdef CONFIG_MTD_BYTE_WRITE
  rc = MTD_READ(fs->mtd, offset, len, data);
#else
  rc = MTD_BREAD(fs->mtd, offset / fs->progsize, len / fs->progsize, data);
#endif
  if (rc == -EBADMSG)
    {
      /* ECC fail first time
       * try again to avoid transient electronic interference
       */

#ifdef CONFIG_MTD_BYTE_WRITE
      rc = MTD_READ(fs->mtd, offset, len, data);
#else
      rc = MTD_BREAD(fs->mtd, offset / fs->progsize, len / fs->progsize,
                     data);
#endif
      if (rc == -EBADMSG)
        {
          /* ECC fail second time
           * fill ~erasestate to trigger recovery process
           */

          memset(data, ~fs->erasestate, len);
          rc = 0;
        }
    }

  return rc < 0 ? rc : 0;
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
#ifdef CONFIG_MTD_BYTE_WRITE
  off_t offset;

  offset = fs->blocksize * (addr >> NVS_ADDR_BLOCK_SHIFT);
  offset += addr & NVS_ADDR_OFFS_MASK;

  return nvs_flash_brd(fs, offset, data, len);
#else
  uint8_t buf[NVS_BUFFER_SIZE(fs)];
  size_t bytes_to_rd;
  off_t begin_padding;
  off_t offset;
  int rc;

  offset = fs->blocksize * (addr >> NVS_ADDR_BLOCK_SHIFT) +
           (addr & NVS_ADDR_OFFS_MASK);
  begin_padding = offset % fs->progsize;

  if (begin_padding > 0)
    {
      offset -= begin_padding;
      bytes_to_rd = MIN(fs->progsize - begin_padding, len);
      rc = nvs_flash_brd(fs, offset, buf, fs->progsize);
      if (rc < 0)
        {
          return rc;
        }

      memcpy(data, buf + begin_padding, bytes_to_rd);
      offset += fs->progsize;
      data += bytes_to_rd;
      len -= bytes_to_rd;
    }

  if (len >= fs->progsize)
    {
      bytes_to_rd = len / fs->progsize * fs->progsize;
      rc = nvs_flash_brd(fs, offset, data, bytes_to_rd);
      if (rc < 0)
        {
          return rc;
        }

      offset += bytes_to_rd;
      data += bytes_to_rd;
      len -= bytes_to_rd;
    }

  if (len > 0)
    {
      rc = nvs_flash_brd(fs, offset, buf, fs->progsize);
      if (rc < 0)
        {
          return rc;
        }

      memcpy(data, buf, len);
    }

  return 0;
#endif
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
  size_t ate_size = nvs_ate_size(fs);
  int rc;

#if CONFIG_MTD_CONFIG_CACHE_SIZE > 0

  /* 0xFFFF is a special-purpose identifier. Exclude it from the cache */

  if (entry->id != nvs_special_ate_id(fs))
    {
      fs->cache[nvs_cache_index(entry->id)] = fs->ate_wra;
    }
#endif

  rc = nvs_flash_wrt(fs, fs->ate_wra, entry, ate_size);
  fs->ate_wra -= ate_size;

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
  return nvs_flash_rd(fs, addr, entry, nvs_ate_size(fs));
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
  uint8_t buf[NVS_BUFFER_SIZE(fs)];
  size_t buf_size = nvs_align_down(fs, sizeof(buf));
  size_t bytes_to_cmp;
  int rc;

  while (len > 0)
    {
      bytes_to_cmp = MIN(buf_size, len);
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
  uint8_t buf1[NVS_BUFFER_SIZE(fs)];
  uint8_t buf2[NVS_BUFFER_SIZE(fs)];
  size_t buf_size = nvs_align_down(fs, sizeof(buf1));
  size_t bytes_to_cmp;
  int rc;

  while (len > 0)
    {
      bytes_to_cmp = MIN(buf_size, len);
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
  uint8_t cmp[NVS_BUFFER_SIZE(fs)];
  size_t buf_size = nvs_align_down(fs, sizeof(cmp));
  size_t bytes_to_cmp;
  int rc;

  memset(cmp, value, sizeof(cmp));
  while (len > 0)
    {
      bytes_to_cmp = MIN(buf_size, len);
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

static int nvs_flash_block_move(FAR struct nvs_fs *fs,
                                FAR struct nvs_ate *entry, uint32_t addr)
{
  size_t len = nvs_align_up(fs, entry->key_len + entry->len);
  uint32_t data_begin = addr + entry->key_len;
  uint32_t data_end = data_begin + entry->len;
  uint8_t buf[NVS_BUFFER_SIZE(fs)];
  size_t buf_size = nvs_align_down(fs, sizeof(buf));
  uint32_t hash = NVS_HASH_INITIAL_VALUE;
  uint8_t data_crc8 = 0;
  int rc;

  while (len)
    {
      size_t bytes_to_copy = MIN(buf_size, len);
      rc = nvs_flash_rd(fs, addr, buf, bytes_to_copy);
      if (rc)
        {
          return rc;
        }

      if (addr < data_begin)
        {
          hash = nvs_fnv_hash_part(buf, MIN(bytes_to_copy,
                                   data_begin - addr), hash);
        }

      if (addr + bytes_to_copy > data_begin)
        {
          if (nvs_fnv_hash_id(hash) != entry->id)
            {
              return -EBADMSG;
            }

          uint32_t end_addr = MIN(data_end, addr + bytes_to_copy);
          uint32_t begin_addr = MAX(data_begin, addr);
          data_crc8 = crc8part(buf + (begin_addr - addr),
                               end_addr - begin_addr, data_crc8);
        }

      rc = nvs_flash_data_wrt(fs, buf, bytes_to_copy);
      if (rc)
        {
          return rc;
        }

      len -= bytes_to_copy;
      addr += bytes_to_copy;
    }

  return data_crc8 == entry->data_crc8 ? 0 : -EBADMSG;
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

#if CONFIG_MTD_CONFIG_CACHE_SIZE > 0
  nvs_invalid_cache(fs, addr >> NVS_ADDR_BLOCK_SHIFT);
#endif

  rc = MTD_ERASE(fs->mtd,
                 CONFIG_MTD_CONFIG_BLOCKSIZE_MULTIPLE *
                 (addr >> NVS_ADDR_BLOCK_SHIFT),
                 CONFIG_MTD_CONFIG_BLOCKSIZE_MULTIPLE);
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
 *   Returns true if OK, false on crc fail.
 *
 ****************************************************************************/

static bool nvs_ate_crc8_check(FAR struct nvs_fs *fs,
                               FAR const struct nvs_ate *entry)
{
  uint8_t ate_crc;

  ate_crc = crc8part((FAR const uint8_t *)entry,
                     offsetof(struct nvs_ate, crc8), 0xff);
  return ate_crc == entry->crc8 && (entry->len > 0 ||
         entry->data_crc8 == 0 || entry->data_crc8 == fs->erasestate);
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
                             uint8_t value, size_t len)
{
  FAR const uint8_t *data8 = (FAR const uint8_t *)entry;
  size_t i;

  for (i = 0; i < len; i++)
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
 *   Return true if crc8 and offset valid, false otherwise
 *
 ****************************************************************************/

static bool nvs_ate_valid(FAR struct nvs_fs *fs,
                         FAR const struct nvs_ate *entry)
{
  return nvs_ate_crc8_check(fs, entry) &&
         entry->offset < (fs->blocksize - nvs_ate_size(fs)) &&
         (entry->key_len > 0 || entry->id == nvs_special_ate_id(fs));
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
 *   Return true if valid, false otherwise.
 *
 ****************************************************************************/

static bool nvs_close_ate_valid(FAR struct nvs_fs *fs,
                               FAR const struct nvs_ate *entry)
{
  if (!nvs_ate_valid(fs, entry) || entry->len != 0 ||
      entry->id != nvs_special_ate_id(fs))
    {
      return false;
    }

  if ((fs->blocksize - entry->offset) % nvs_ate_size(fs))
    {
      return false;
    }

  return true;
}

/****************************************************************************
 * Name: nvs_flash_write_multi_blk
 *
 * Description:
 *   Store multi align block in flash
 *
 * Returned Value:
 *   number of bytes at the end of addr which is left to write next time.
 *   zero indicates all bytes were written . On error returns -ERRNO code.
 ****************************************************************************/

int nvs_flash_write_multi_blk(FAR struct nvs_fs *fs, const uint8_t *addr,
                              size_t size)
{
  int left;
  int rc;

  left = size % fs->progsize;

  if (size > left)
    {
      rc = nvs_flash_data_wrt(fs, addr, size - left);
      if (rc)
        {
          ferr("Write multi data value failed, rc=%d\n", rc);
          return rc;
        }
    }

  return left;
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
  size_t ate_size = nvs_ate_size(fs);
  NVS_ATE(entry, ate_size);
  uint8_t buf[NVS_BUFFER_SIZE(fs)];
  uint16_t copy_len = 0;
  uint16_t left;
  int rc;

  memset(entry, fs->erasestate, ate_size);
  entry->id = id;
  entry->offset = fs->data_wra & NVS_ADDR_OFFS_MASK;
  entry->len = len;
  entry->key_len = key_size;
  entry->data_crc8 = crc8(data, len);

  nvs_ate_crc8_update(entry);

  /* Let's save key and data into one, key comes first, then data */

  rc = nvs_flash_write_multi_blk(fs, key, key_size);
  if (rc < 0)
    {
      return rc;
    }

  if (rc)
    {
      /* Write align block which include part key + part data */

      left = rc;
      memset(buf, fs->erasestate, fs->progsize);

      copy_len = (left + len) <= fs->progsize ?
                  len : (fs->progsize - left);

      memcpy(buf, key + key_size - left, left);
      memcpy(buf + left, data, copy_len);
      rc = nvs_flash_data_wrt(fs, buf, fs->progsize);
      if (rc)
        {
          ferr("Write value failed, rc=%d\n", rc);
          return rc;
        }
    }

  rc = nvs_flash_write_multi_blk(fs, data + copy_len,  len - copy_len);
  if (rc < 0)
    {
      return rc;
    }

  if (rc)
    {
      /* Add padding at the end of data */

      left = rc;
      memset(buf, fs->erasestate, fs->progsize);
      memcpy(buf, data + len - left, left);

      rc = nvs_flash_data_wrt(fs, buf, fs->progsize);
      if (rc)
        {
          ferr("Write value failed, rc=%d\n", rc);
          return rc;
        }
    }

  /* Last, let's save entry to flash */

  rc = nvs_flash_ate_wrt(fs, entry);
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
  size_t ate_size = nvs_ate_size(fs);
  NVS_ATE(end_ate, ate_size);
  uint32_t data_end_addr;
  uint32_t ate_end_addr;
  int rc;

  finfo("Recovering last ate from block %" PRIu32 "\n",
        (*addr >> NVS_ADDR_BLOCK_SHIFT));

  *addr -= ate_size;
  ate_end_addr = *addr;
  data_end_addr = *addr & NVS_ADDR_BLOCK_MASK;
  while (ate_end_addr > data_end_addr)
    {
      rc = nvs_flash_ate_rd(fs, ate_end_addr, end_ate);
      if (rc)
        {
          return rc;
        }

      if (nvs_ate_valid(fs, end_ate) &&
          end_ate->offset >= (data_end_addr & NVS_ADDR_OFFS_MASK))
        {
          /* Found a valid ate, update data_end_addr and *addr */

          data_end_addr &= NVS_ADDR_BLOCK_MASK;
          data_end_addr += end_ate->offset +
                           nvs_align_up(fs, end_ate->key_len + end_ate->len);
          *addr = ate_end_addr;
        }

      if (ate_end_addr < ate_size)
        {
          break;
        }

      ate_end_addr -= ate_size;
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
  size_t ate_size = nvs_ate_size(fs);
  NVS_ATE(close_ate, ate_size);
  int rc;

  rc = nvs_flash_ate_rd(fs, *addr, ate);
  if (rc)
    {
      return rc;
    }

  *addr += ate_size;
  if (((*addr) & NVS_ADDR_OFFS_MASK) != (fs->blocksize - ate_size))
    {
      return 0;
    }

  /* Last ate in block, do jump to previous block */

  if (((*addr) >> NVS_ADDR_BLOCK_SHIFT) == 0)
    {
      *addr += ((fs->nblocks - 1) << NVS_ADDR_BLOCK_SHIFT);
    }
  else
    {
      *addr -= (1 << NVS_ADDR_BLOCK_SHIFT);
    }

  rc = nvs_flash_ate_rd(fs, *addr, close_ate);
  if (rc)
    {
      return rc;
    }

  rc = nvs_ate_cmp_const(close_ate, fs->erasestate, ate_size);

  /* At the end of filesystem */

  if (!rc)
    {
      *addr = fs->ate_wra;
      return 0;
    }

  /* Update the address if the close ate is valid. */

  if (nvs_close_ate_valid(fs, close_ate))
    {
      *addr &= NVS_ADDR_BLOCK_MASK;
      *addr += close_ate->offset;
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
  *addr += (1 << NVS_ADDR_BLOCK_SHIFT);
  if ((*addr >> NVS_ADDR_BLOCK_SHIFT) == fs->nblocks)
    {
      *addr -= (fs->nblocks << NVS_ADDR_BLOCK_SHIFT);
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
  size_t ate_size = nvs_ate_size(fs);
  NVS_ATE(close_ate, ate_size);
  int rc;

  memset(close_ate, fs->erasestate, ate_size);
  close_ate->id = nvs_special_ate_id(fs);
  close_ate->len = 0;
  close_ate->key_len = 0;
  close_ate->offset = (fs->ate_wra + ate_size) & NVS_ADDR_OFFS_MASK;

  fs->ate_wra &= NVS_ADDR_BLOCK_MASK;
  fs->ate_wra += fs->blocksize - ate_size;

  nvs_ate_crc8_update(close_ate);

  rc = nvs_flash_ate_wrt(fs, close_ate);
  if (rc < 0)
    {
      ferr("Write ate failed, rc=%d\n", rc);
    }

  nvs_block_advance(fs, &fs->ate_wra);

  fs->data_wra = fs->ate_wra & NVS_ADDR_BLOCK_MASK;
  finfo("block close, data_wra=0x%" PRIx32 "\n", fs->data_wra);

  return 0;
}

/****************************************************************************
 * Name: nvs_add_gc_done_ate
 ****************************************************************************/

static int nvs_add_gc_done_ate(FAR struct nvs_fs *fs)
{
  size_t ate_size = nvs_ate_size(fs);
  NVS_ATE(gc_done_ate, ate_size);

  finfo("Adding gc done ate at %" PRIx32 "\n",
        fs->ate_wra & NVS_ADDR_OFFS_MASK);
  memset(gc_done_ate, fs->erasestate, ate_size);
  gc_done_ate->id = nvs_special_ate_id(fs);
  gc_done_ate->len = 0;
  gc_done_ate->key_len = 0;
  gc_done_ate->offset = fs->data_wra & NVS_ADDR_OFFS_MASK;
  nvs_ate_crc8_update(gc_done_ate);

  return nvs_flash_ate_wrt(fs, gc_done_ate);
}

/****************************************************************************
 * Name: nvs_expire_ate
 ****************************************************************************/

static int nvs_expire_ate(FAR struct nvs_fs *fs, uint32_t addr)
{
  uint8_t expired[NVS_BUFFER_SIZE(fs)];
  memset(expired, ~fs->erasestate, fs->progsize);

  return nvs_flash_wrt(fs, addr + nvs_align_up(fs, sizeof(struct nvs_ate)),
                       expired, fs->progsize);
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
  size_t ate_size = nvs_ate_size(fs);
  NVS_ATE(close_ate, ate_size);
  NVS_ATE(gc_ate, ate_size);
  uint32_t gc_prev_addr;
  uint32_t data_addr;
  uint32_t stop_addr;
  uint32_t sec_addr;
  uint32_t gc_addr;
  int rc;

  finfo("gc: before gc, ate_wra %" PRIx32 "\n", fs->ate_wra);

  sec_addr = (fs->ate_wra & NVS_ADDR_BLOCK_MASK);
  nvs_block_advance(fs, &sec_addr);
  gc_addr = sec_addr + fs->blocksize - ate_size;

  finfo("gc: set, sec_addr %" PRIx32 ", gc_addr %" PRIx32 "\n", sec_addr,
        gc_addr);

  /* If the block is not closed don't do gc */

  rc = nvs_flash_ate_rd(fs, gc_addr, close_ate);
  if (rc < 0)
    {
      /* Flash error */

      return rc;
    }

  rc = nvs_ate_cmp_const(close_ate, fs->erasestate, ate_size);
  if (!rc)
    {
      goto gc_done;
    }

  stop_addr = gc_addr - ate_size;

  if (nvs_close_ate_valid(fs, close_ate))
    {
      gc_addr &= NVS_ADDR_BLOCK_MASK;
      gc_addr += close_ate->offset;
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
      rc = nvs_prev_ate(fs, &gc_addr, gc_ate);
      if (rc)
        {
          return rc;
        }

      if (nvs_ate_expired(fs, gc_ate))
        {
          /* Deleted or old ate, ignore it */

          continue;
        }

      if (!nvs_ate_valid(fs, gc_ate))
        {
          continue;
        }

      if (gc_ate->id != nvs_special_ate_id(fs))
        {
          /* Copy needed */

          finfo("Moving %" PRIu32 ", key_len %" PRIu16 ", len %" PRIu16 "\n",
                gc_ate->id, gc_ate->key_len, gc_ate->len);

          data_addr = gc_prev_addr & NVS_ADDR_BLOCK_MASK;
          data_addr += gc_ate->offset;
          gc_ate->offset = fs->data_wra & NVS_ADDR_OFFS_MASK;
          nvs_ate_crc8_update(gc_ate);

          rc = nvs_flash_block_move(fs, gc_ate, data_addr);
          if (rc == -EBADMSG)
            {
              continue;
            }
          else if (rc)
            {
              return rc;
            }

          rc = nvs_flash_ate_wrt(fs, gc_ate);
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
  struct mtd_geometry_s geo;
  uint32_t second_addr;
  uint32_t last_addr;
  uint32_t wlk_addr;
  size_t empty_len;

  /* Initialize addr to 0 for the case fs->nblocks == 0. This
   * should never happen but both
   * Coverity and GCC believe the contrary.
   */

  uint16_t closed_blocks = 0;
  uint32_t addr = 0;
  uint16_t i;
  int rc;

  fs->ate_wra = 0;
  fs->data_wra = 0;
  fs->events = 0;
  fs->fds = NULL;

  /* Get the device geometry. (Casting to uintptr_t first eliminates
   * complaints on some architectures where the sizeof long is different
   * from the size of a pointer).
   */

  rc = MTD_IOCTL(fs->mtd, MTDIOC_GEOMETRY,
                 (unsigned long)((uintptr_t)&(geo)));
  if (rc < 0)
    {
      ferr("ERROR: MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", rc);
      return rc;
    }

  fs->blocksize = CONFIG_MTD_CONFIG_BLOCKSIZE_MULTIPLE * geo.erasesize;
  fs->nblocks   = geo.neraseblocks / CONFIG_MTD_CONFIG_BLOCKSIZE_MULTIPLE;
  fs->progsize  = geo.blocksize;

  size_t ate_size = nvs_ate_size(fs);
  NVS_ATE(second_ate, ate_size);
  NVS_ATE(last_ate, ate_size);

#if CONFIG_MTD_CONFIG_BUFFER_SIZE > 0
  DEBUGASSERT(ate_size <= CONFIG_MTD_CONFIG_BUFFER_SIZE);
#endif

  rc = MTD_IOCTL(fs->mtd, MTDIOC_ERASESTATE,
                 (unsigned long)((uintptr_t)&fs->erasestate));
  if (rc < 0)
    {
      ferr("ERROR: MTD ioctl(MTDIOC_ERASESTATE) failed: %d\n", rc);
      return rc;
    }

  /* Check the number of blocks, it should be at least 2. */

  if (fs->nblocks < 2)
    {
      ferr("Configuration error - block count\n");
      return -EINVAL;
    }

  /* Step through the blocks to find a open block following
   * a closed block, this is where NVS can write.
   */

  for (i = 0; i < fs->nblocks; i++)
    {
      addr = (i << NVS_ADDR_BLOCK_SHIFT) +
             (uint16_t)(fs->blocksize - ate_size);
      rc = nvs_flash_cmp_const(fs, addr, fs->erasestate, ate_size);
      fwarn("rc=%d\n", rc);
      if (rc)
        {
          /* Closed block */

          closed_blocks++;
          nvs_block_advance(fs, &addr);
          rc = nvs_flash_cmp_const(fs, addr, fs->erasestate, ate_size);
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

  if (closed_blocks == fs->nblocks)
    {
      return -EDEADLK;
    }

  if (i == fs->nblocks)
    {
      /* None of the blocks where closed, in most cases we can set
       * the address to the first block, except when there are only
       * two blocks. Then we can only set it to the first block if
       * the last block contains no ate's. So we check this first
       */

      rc = nvs_flash_cmp_const(fs, addr - ate_size,
                               fs->erasestate, ate_size);
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
  fs->data_wra = addr & NVS_ADDR_BLOCK_MASK;
  finfo("recovered ate ate_wra=0x%" PRIx32 ", data_wra=0x%" PRIx32 "\n",
        fs->ate_wra, fs->data_wra);

  while (fs->ate_wra >= fs->data_wra)
    {
      rc = nvs_flash_ate_rd(fs, fs->ate_wra, last_ate);
      if (rc)
        {
          return rc;
        }

      rc = nvs_ate_cmp_const(last_ate, fs->erasestate, ate_size);
      if (!rc)
        {
          /* Found 0xff empty location */

          break;
        }

      if (nvs_ate_valid(fs, last_ate))
        {
          /* Complete write of ate was performed */

          fs->data_wra = addr & NVS_ADDR_BLOCK_MASK;
          fs->data_wra += last_ate->offset +
                          nvs_align_up(fs, last_ate->key_len +
                                       last_ate->len);
          finfo("recovered data_wra=0x%" PRIx32 "\n", fs->data_wra);
        }

      fs->ate_wra -= ate_size;
    }

  /* If the block after the write block is not empty, gc was interrupted
   * we might need to restart gc if it has not yet finished. Otherwise
   * just erase the block.
   * When gc needs to be restarted, first erase the block otherwise the
   * data might not fit into the block.
   */

  addr = fs->ate_wra & NVS_ADDR_BLOCK_MASK;
  nvs_block_advance(fs, &addr);
  rc = nvs_flash_cmp_const(fs, addr, fs->erasestate, fs->blocksize);
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
      NVS_ATE(gc_done_ate, ate_size);

      addr = fs->ate_wra + ate_size;
      while ((addr & NVS_ADDR_OFFS_MASK) <
             (fs->blocksize - ate_size))
        {
          rc = nvs_flash_ate_rd(fs, addr, gc_done_ate);
          if (rc)
            {
              return rc;
            }

          if (nvs_ate_valid(fs, gc_done_ate) &&
              (gc_done_ate->id == nvs_special_ate_id(fs)) &&
              (gc_done_ate->len == 0))
            {
              gc_done_marker = true;
              break;
            }

          addr += ate_size;
        }

      if (gc_done_marker)
        {
          /* Erase the next block */

          fwarn("GC Done marker found\n");
          addr = fs->ate_wra & NVS_ADDR_BLOCK_MASK;
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

      fs->ate_wra &= NVS_ADDR_BLOCK_MASK;
      fs->ate_wra += (fs->blocksize - 2 * ate_size);
      fs->data_wra = (fs->ate_wra & NVS_ADDR_BLOCK_MASK);
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

      fs->data_wra += fs->progsize;
      finfo("update for powerloss data write, data_wra=0x%" PRIx32 "\n",
            fs->data_wra);
    }

  /* Check if there exists an old entry with the same id and key
   * as the newest entry.
   * If so, power loss occurred before writing the old entry id as expired.
   * We need to set old entry expired.
   */

#if CONFIG_MTD_CONFIG_CACHE_SIZE > 0
  memset(fs->cache, 0xff, sizeof(fs->cache));
#endif

  wlk_addr = fs->ate_wra;
  while (1)
    {
      last_addr = wlk_addr;
      rc = nvs_prev_ate(fs, &wlk_addr, last_ate);
      if (rc)
        {
          return rc;
        }

      /* Skip last one */

      if (wlk_addr == fs->ate_wra)
        {
          break;
        }

      if (nvs_ate_valid(fs, last_ate)
          && (last_ate->id != nvs_special_ate_id(fs)))
        {
          finfo("ate found at 0x%" PRIx32 ", id %" PRIu32 ", "
                "key_len %" PRIu16 ", offset %" PRIu16 "\n",
                last_addr, last_ate->id, last_ate->key_len,
                last_ate->offset);

#if CONFIG_MTD_CONFIG_CACHE_SIZE > 0
          fs->cache[nvs_cache_index(last_ate->id)] = last_addr;
#endif
          while (1)
            {
              second_addr = wlk_addr;
              rc = nvs_prev_ate(fs, &wlk_addr, second_ate);
              if (rc)
                {
                  return rc;
                }

              if (nvs_ate_valid(fs, second_ate)
                  && !nvs_ate_expired(fs, second_ate))
                {
#if CONFIG_MTD_CONFIG_CACHE_SIZE > 0
                  fs->cache[nvs_cache_index(second_ate->id)] = second_addr;
#endif
                  if (second_ate->id == last_ate->id)
                    {
                      finfo("same id at 0x%" PRIx32 ", key_len %" PRIu16 ", "
                            "offset %" PRIu16 "\n", second_addr,
                            second_ate->key_len, second_ate->offset);
                      if ((second_ate->key_len == last_ate->key_len) &&
                          !nvs_flash_direct_cmp(fs,
                                                (last_addr &
                                                 NVS_ADDR_BLOCK_MASK) +
                                                last_ate->offset,
                                                (second_addr &
                                                 NVS_ADDR_BLOCK_MASK) +
                                                second_ate->offset,
                                                last_ate->key_len))
                        {
                          finfo("old ate found at 0x%" PRIx32 "\n",
                                second_addr);
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

  if ((!rc) && ((fs->ate_wra & NVS_ADDR_OFFS_MASK) ==
      (fs->blocksize - 2 * ate_size)))
    {
      rc = nvs_add_gc_done_ate(fs);
    }

  finfo("%" PRIu32 " Eraseblocks of %" PRIu32 " bytes\n",
        fs->nblocks, fs->blocksize);
  finfo("alloc wra: %" PRIu32 ", 0x%" PRIx32 "\n",
        fs->ate_wra >> NVS_ADDR_BLOCK_SHIFT,
        fs->ate_wra & NVS_ADDR_OFFS_MASK);
  finfo("data wra: %" PRIu32 ", 0x%" PRIx32 "\n",
        fs->data_wra >> NVS_ADDR_BLOCK_SHIFT,
        fs->data_wra & NVS_ADDR_OFFS_MASK);

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
  NVS_ATE(wlk_ate, nvs_ate_size(fs));
  uint32_t wlk_addr;
  uint32_t rd_addr;
  uint32_t hist_addr;
  uint32_t hash_id;
  uint8_t data_crc8;
  bool hit = true;
  int rc;

  hash_id = nvs_fnv_hash_id(nvs_fnv_hash(key, key_size));
#if CONFIG_MTD_CONFIG_CACHE_SIZE > 0
  wlk_addr = fs->cache[nvs_cache_index(hash_id)];
  if (wlk_addr == NVS_CACHE_NO_ADDR)
#endif
    {
      wlk_addr = fs->ate_wra;
      hit = false;
    }

  rd_addr = wlk_addr;

  do
    {
      rd_addr = wlk_addr;
      hist_addr = wlk_addr;
      rc = nvs_prev_ate(fs, &wlk_addr, wlk_ate);
      if (rc)
        {
          ferr("Walk to previous ate failed, rc=%d\n", rc);
          return rc;
        }

      if (wlk_ate->id == hash_id && nvs_ate_valid(fs, wlk_ate))
        {
          if ((wlk_ate->key_len == key_size)
              && (!nvs_flash_block_cmp(fs,
                                       (rd_addr & NVS_ADDR_BLOCK_MASK) +
                                       wlk_ate->offset, key, key_size)))
            {
              /* It is old or deleted, return -ENOENT */

              if (nvs_ate_expired(fs, wlk_ate))
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

      /* There are two types of hash conflicts found by cache, one is hash_id
       * conflict and the other is hash_id % CONFIG_MTD_CONFIG_CACHE_SIZE
       * conflict, both of which require traversing flash from the flash
       * beginning.
       */

      if (hit)
        {
          wlk_addr = fs->ate_wra;
          hit = false;
          continue;
        }

      if (wlk_addr == fs->ate_wra)
        {
          return -ENOENT;
        }
    }
  while (true);

  if (data && len)
    {
      rd_addr &= NVS_ADDR_BLOCK_MASK;
      rd_addr += wlk_ate->offset + wlk_ate->key_len;
      rc = nvs_flash_rd(fs, rd_addr, data,
                        MIN(len, wlk_ate->len));
      if (rc)
        {
          ferr("Data read failed, rc=%d\n", rc);
          return rc;
        }

      if (len >= wlk_ate->len && wlk_ate->data_crc8 != fs->erasestate)
        {
          /* Do not compute CRC for partial reads as CRC won't match */

          data_crc8 = crc8(data, wlk_ate->len);
          if (wlk_ate->data_crc8 != data_crc8)
            {
              ferr("Invalid data crc: %" PRIx8 ", wlk_ate->data_crc8: "
                   "%" PRIx8 "\n", data_crc8, wlk_ate->data_crc8);
              nvs_expire_ate(fs, hist_addr);
              return -EIO;
            }
        }
    }

  if (ate_addr)
    {
      *ate_addr = hist_addr;
    }

  return wlk_ate->len;
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
  size_t ate_size = nvs_ate_size(fs);
  NVS_ATE(wlk_ate, ate_size);
  uint32_t wlk_addr;
  uint32_t rd_addr;
  uint32_t hist_addr;
  uint16_t required_space = 0;
  bool prev_found = false;
  uint32_t hash_id;
  uint16_t block_to_write_befor_gc;
  bool hit = true;

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

  data_size = nvs_align_up(fs, key_size + pdata->len);

  /* The maximum data size is block size - 3 ate
   * where: 1 ate for data, 1 ate for block close, 1 ate for gc done.
   */

  finfo("key_size=%zu, len=%zu, data_size = %zu\n", key_size,
                                                    pdata->len, data_size);

  if ((data_size > (fs->blocksize - 3 * ate_size)) ||
      ((pdata->len > 0) && (pdata->configdata == NULL)))
    {
      return -EINVAL;
    }

  /* Calc hash id of key. */

  hash_id = nvs_fnv_hash_id(nvs_fnv_hash(key, key_size));

  /* Find latest entry with same id. */

#if CONFIG_MTD_CONFIG_CACHE_SIZE > 0
  wlk_addr = fs->cache[nvs_cache_index(hash_id)];
  if (wlk_addr == NVS_CACHE_NO_ADDR)
#endif
    {
      wlk_addr = fs->ate_wra;
      hit = false;
    }

  while (1)
    {
      rd_addr = wlk_addr;
      hist_addr = wlk_addr;
      rc = nvs_prev_ate(fs, &wlk_addr, wlk_ate);
      if (rc)
        {
          return rc;
        }

      if (wlk_ate->id == hash_id && nvs_ate_valid(fs, wlk_ate))
        {
          if ((wlk_ate->key_len == key_size)
              && !nvs_flash_block_cmp(fs,
                                      (rd_addr & NVS_ADDR_BLOCK_MASK) +
                                      wlk_ate->offset, key, key_size))
            {
              prev_found = true;
              break;
            }
          else
            {
              fwarn("hash conflict\n");
            }
        }

      /* There are two types of hash conflicts found by cache, one is hash_id
       * conflict and the other is hash_id % CONFIG_MTD_CONFIG_CACHE_SIZE
       * conflict, both of which require traversing flash from the flash
       * beginning.
       */

      if (hit)
        {
          wlk_addr = fs->ate_wra;
          hit = false;
          continue;
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

      rd_addr &= NVS_ADDR_BLOCK_MASK;

      if (pdata->len == 0)
        {
          /* If prev ate is expired, it is deleted. */

          if (nvs_ate_expired(fs, wlk_ate))
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
      else if (pdata->len == wlk_ate->len && !nvs_ate_expired(fs, wlk_ate))
        {
          /* Do not try to compare if lengths are not equal
           * or prev one is deleted.
           * Compare the data and if equal return 0.
           */

          rd_addr += wlk_ate->offset + wlk_ate->key_len;
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

  required_space = data_size + ate_size;
  gc_count = 0;
  block_to_write_befor_gc = fs->ate_wra >> NVS_ADDR_BLOCK_SHIFT;
  while (1)
    {
      if (gc_count == fs->nblocks)
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

          if (prev_found && !nvs_ate_expired(fs, wlk_ate))
            {
              finfo("prev entry exists, search for it\n");

              /* If gc touched second latest ate, search for it again */

              if (gc_count >= fs->nblocks - 1 -
                  (block_to_write_befor_gc + fs->nblocks -
                  (hist_addr >> NVS_ADDR_BLOCK_SHIFT))
                  % fs->nblocks)
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

          if (prev_found && !nvs_ate_expired(fs, wlk_ate))
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
  ssize_t rc;

#ifdef CONFIG_MTD_CONFIG_NAMED
  FAR const uint8_t *key;
#else
  uint8_t key[sizeof(pdata->id) + sizeof(pdata->instance)];
#endif

  if (pdata == NULL)
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

  rc = nvs_read_entry(fs, key, key_size, pdata->configdata, pdata->len,
                      NULL);
  if (rc > 0)
    {
      pdata->len = rc;
      return 0;
    }

  return rc;
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
  NVS_ATE(step_ate, nvs_ate_size(fs));
  uint32_t rd_addr;
  int rc;

  if (pdata == NULL || pdata->len == 0 || pdata->configdata == NULL)
    {
      return -EINVAL;
    }

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
      rc = nvs_prev_ate(fs, &(fs->step_addr), step_ate);
      if (rc)
        {
          return rc;
        }

      if (nvs_ate_valid(fs, step_ate)
          && step_ate->id != nvs_special_ate_id(fs)
          && !nvs_ate_expired(fs, step_ate))
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
  rc = nvs_flash_rd(fs, (rd_addr & NVS_ADDR_BLOCK_MASK) + step_ate->offset,
                    key, MIN(step_ate->key_len, CONFIG_MTD_CONFIG_NAME_LEN));
  if (rc)
    {
      ferr("Key read failed, rc=%d\n", rc);
      return rc;
    }

  key[CONFIG_MTD_CONFIG_NAME_LEN - 1] = 0;
#else
  rc = nvs_flash_rd(fs, (rd_addr & NVS_ADDR_BLOCK_MASK) + step_ate->offset,
                    key, MIN(sizeof(key), step_ate->key_len));
  if (rc)
    {
      ferr("Key read failed, rc=%d\n", rc);
      return rc;
    }

  memcpy(&pdata->id, key, sizeof(pdata->id));
  memcpy(&pdata->instance, key + sizeof(pdata->id), sizeof(pdata->instance));
#endif

  rc = nvs_flash_rd(fs, (rd_addr & NVS_ADDR_BLOCK_MASK) + step_ate->offset +
                    step_ate->key_len, pdata->configdata,
                    MIN(pdata->len, step_ate->len));
  if (rc)
    {
      ferr("Value read failed, rc=%d\n", rc);
      return rc;
    }

  pdata->len = MIN(pdata->len, step_ate->len);
  return OK;
}

/****************************************************************************
 * Name: mtdconfig_notify
 *
 * Description:
 *   Notify the poll if any waiter, or save events for next setup.
 *
 * Input Parameters:
 *   fs       - Pointer to file system.
 *   eventset - List of events to check for activity
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void mtdconfig_notify(FAR struct nvs_fs *fs, pollevent_t eventset)
{
  /* Handle events in two possible ways:
   * 1. Notify waters directly if any exist(`fs->fds` is not NULL)
   * 2. Save events for the following scenarios:
   *    a. Events that have changed but weren't waited for
   *       before being added to the interest list
   *    b. Events occurring after `epoll_wait()` returns and
   *       before it's called again
   */

  if (fs->fds)
    {
      poll_notify(&fs->fds, 1, eventset | fs->events);
      fs->events = 0;
    }
  else
    {
      fs->events |= eventset;
    }
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
  FAR struct nvs_fs *fs = inode->i_private;
  FAR struct config_data_s *pdata = (FAR struct config_data_s *)arg;
  int rc = -ENOTTY;

  rc = nxmutex_lock(&fs->nvs_lock);
  if (rc < 0)
    {
      return rc;
    }

  switch (cmd)
    {
      case CFGDIOC_GETCONFIG:

        /* Read a nvs item. */

        rc = nvs_read(fs, pdata);
        break;

      case CFGDIOC_SETCONFIG:

        /* Write a nvs item. */

        rc = nvs_write(fs, pdata);
        if (rc >= 0)
          {
            mtdconfig_notify(fs, POLLPRI);
          }

        break;

      case CFGDIOC_DELCONFIG:

        /* Delete a nvs item. */

        rc = nvs_delete(fs, pdata);
        if (rc >= 0)
          {
            mtdconfig_notify(fs, POLLPRI);
          }

        break;

      case CFGDIOC_FIRSTCONFIG:

        /* Get the first item. */

        rc = nvs_next(fs, pdata, true);
        break;

      case CFGDIOC_NEXTCONFIG:

        /* Get the next item. */

        rc = nvs_next(fs, pdata, false);
        break;

      case MTDIOC_BULKERASE:

        /* Call the MTD's ioctl for this. */

        rc = MTD_IOCTL(fs->mtd, cmd, arg);
        if (rc >= 0)
          {
            rc = nvs_startup(fs);
          }

        break;
    }

  nxmutex_unlock(&fs->nvs_lock);
  return rc;
}

/****************************************************************************
 * Name: mtdconfig_poll
 ****************************************************************************/

static int mtdconfig_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct nvs_fs *fs = inode->i_private;
  int ret;

  ret = nxmutex_lock(&fs->nvs_lock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      fs->fds = fds;
      mtdconfig_notify(fs, POLLIN | POLLOUT);
    }
  else
    {
      fs->fds = NULL;
    }

  nxmutex_unlock(&fs->nvs_lock);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtdconfig_register_by_path
 *
 * Description:
 *   Register a "path" device backed by an fail-safe NVS.
 *
 ****************************************************************************/

int mtdconfig_register_by_path(FAR struct mtd_dev_s *mtd,
                               FAR const char *path)
{
  FAR struct nvs_fs *fs;
  int rc;

  fs = kmm_malloc(sizeof(struct nvs_fs));
  if (fs == NULL)
    {
      return -ENOMEM;
    }

  /* Initialize the mtdnvs device structure */

  fs->mtd = mtd;
  rc = nxmutex_init(&fs->nvs_lock);
  if (rc < 0)
    {
      ferr("ERROR: nxmutex_init failed: %d\n", rc);
      goto errout;
    }

  rc = nvs_startup(fs);
  if (rc < 0)
    {
      ferr("ERROR: nvs_init failed: %d\n", rc);
      goto mutex_err;
    }

  rc = register_driver(path, &g_mtdnvs_fops, 0666, fs);
  if (rc < 0)
    {
      ferr("ERROR: register mtd config failed: %d\n", rc);
      goto mutex_err;
    }

  return rc;

mutex_err:
  nxmutex_destroy(&fs->nvs_lock);

errout:
  kmm_free(fs);
  return rc;
}

/****************************************************************************
 * Name: mtdconfig_register
 *
 * Description:
 *   Register a /dev/config device backed by an fail-safe NVS.
 *
 ****************************************************************************/

int mtdconfig_register(FAR struct mtd_dev_s *mtd)
{
  return mtdconfig_register_by_path(mtd, "/dev/config");
}

/****************************************************************************
 * Name: mtdconfig_unregister_by_path
 *
 * Description:
 *   Unregister a MTD device backed by an fail-safe NVS.
 *
 ****************************************************************************/

int mtdconfig_unregister_by_path(FAR const char *path)
{
  FAR struct inode *inode;
  FAR struct nvs_fs *fs;
  struct file file;
  int rc;

  rc = file_open(&file, path, O_CLOEXEC);
  if (rc < 0)
    {
      ferr("ERROR: open file %s err: %d\n", path, rc);
      return rc;
    }

  inode = file.f_inode;
  fs = inode->i_private;
  nxmutex_destroy(&fs->nvs_lock);
  kmm_free(fs);
  file_close(&file);
  unregister_driver(path);

  return OK;
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
  return mtdconfig_unregister_by_path("/dev/config");
}
