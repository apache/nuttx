/****************************************************************************
 * drivers/mtd/mtd_nandram.c
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

#include <debug.h>
#include <stddef.h>

#include <nuttx/compiler.h>
#include <nuttx/mutex.h>
#include <nuttx/mtd/nand_ram.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_MTD_NAND_RAM_DEBUG

#define NAND_RAM_DEBUG_1       1
#define NAND_RAM_DEBUG_2       5
#define NAND_RAM_DEBUG_3       10

#define NAND_RAM_STATUS_1      1
#define NAND_RAM_STATUS_2      5
#define NAND_RAM_STATUS_3      10
#define NAND_RAM_STATUS_4      50
#define NAND_RAM_STATUS_5      100
#define NAND_RAM_STATUS_6      500
#define NAND_RAM_STATUS_7      1000
#define NAND_RAM_STATUS_8      5000
#define NAND_RAM_STATUS_9      10000
#define NAND_RAM_STATUS_10     50000

#if CONFIG_MTD_NAND_RAM_DEBUG_LEVEL == 1
#define NAND_RAM_DEBUG_LEVEL   NAND_RAM_DEBUG_1
#elif CONFIG_MTD_NAND_RAM_DEBUG_LEVEL == 2
#define NAND_RAM_DEBUG_LEVEL   NAND_RAM_DEBUG_2
#elif CONFIG_MTD_NAND_RAM_DEBUG_LEVEL == 3
#define NAND_RAM_DEBUG_LEVEL   NAND_RAM_DEBUG_3
#endif /* CONFIG_MTD_NAND_RAM_DEBUG_LEVEL */

#if CONFIG_MTD_NAND_RAM_STATUS == 1
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_1
#elif CONFIG_MTD_NAND_RAM_STATUS == 2
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_2
#elif CONFIG_MTD_NAND_RAM_STATUS == 3
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_3
#elif CONFIG_MTD_NAND_RAM_STATUS == 4
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_4
#elif CONFIG_MTD_NAND_RAM_STATUS == 5
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_5
#elif CONFIG_MTD_NAND_RAM_STATUS == 6
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_6
#elif CONFIG_MTD_NAND_RAM_STATUS == 7
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_7
#elif CONFIG_MTD_NAND_RAM_STATUS == 8
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_8
#elif CONFIG_MTD_NAND_RAM_STATUS == 9
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_9
#elif CONFIG_MTD_NAND_RAM_STATUS == 10
#define NAND_RAM_STATUS_LEVEL  NAND_RAM_STATUS_10
#endif /* CONFIG_MTD_NAND_RAM_STATUS */

#define NAND_RAM_LOG(str, ...)                                    \
          {                                                       \
            if (nand_ram_ins_i % NAND_RAM_DEBUG_LEVEL == 0)       \
              {                                                   \
                syslog(LOG_DEBUG, "nand_ram: " str, __VA_ARGS__); \
              }                                                   \
          }                                                       \

#define NAND_RAM_STATUS_LOG(str, ...) \
          syslog(LOG_DEBUG, "nand_ram_status: " str, __VA_ARGS__);

#else

#define NAND_RAM_LOG(str, ...)
#define NAND_RAM_STATUS_LOG(str, ...)

#endif /* CONFIG_MTD_NAND_RAM_DEBUG */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct nand_ram_data_s
{
  uint8_t page[NAND_RAM_PAGE_SIZE];
};

/* 512 B page spare scheme */

struct nand_ram_spare_s
{
  uint8_t ecc_0;    /* 0 */
  uint8_t ecc_1;
  uint8_t ecc_2;
  uint8_t ecc_3;
  uint8_t __res1;
  uint8_t bad;      /* 5 */ /* NAND_RAM_BLOCK_* */
  uint8_t ecc_4;
  uint8_t ecc_5;

  /* Using reserved (8 bytes) */

  uint16_t n_read;
  uint16_t n_write; /* 10 */
  uint16_t n_erase;
  uint8_t  free;    /* Erased page: NAND_RAM_PAGE_* */
  uint8_t  __res2;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t                nand_ram_ins_i = 0; /* Instruction counter */
static mutex_t                 nand_ram_dev_mut;
static struct nand_ram_data_s  nand_ram_flash_data[NAND_RAM_N_PAGES];
static struct nand_ram_spare_s nand_ram_flash_spare[NAND_RAM_N_PAGES];

/* Hard coded array for bad block indexes */

static int g_nand_ram_rand_bad_blk_indx[] =
  {
    4, 14, 19, 21, 28, 30, 107,
    108, 164, 173, 179, 229, 268,
    362, 377, 382, 396, 410, 412,
    419, 428, 456, 500, 0
  };

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * External Functions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_ram_storage_status
 *
 * Description:
 *   Writes per-page status of virtual NAND Flash.
 *
 ****************************************************************************/

static void nand_ram_storage_status(void)
{
  uint32_t  i;
  uint16_t  reads;
  uint16_t  writes;
  uint16_t  erases;
  uint8_t   bad;

  /* Wear */

  for (i = 0; i < NAND_RAM_N_PAGES; i++)
    {
      reads = nand_ram_flash_spare[i].n_read;
      writes = nand_ram_flash_spare[i].n_write;
      erases = nand_ram_flash_spare[i].n_erase;
      bad = (nand_ram_flash_spare[i].bad != NAND_RAM_BLOCK_GOOD);

      NAND_RAM_STATUS_LOG(
        "Block %3" PRIi32 ", Page %6" PRIi32 ", Bad: %1" PRIi32 " |"
        " Reads: %6" PRIi32 ", Writes: %6" PRIi32 ", Erases: %6" PRIi32 "\n",
        i >> NAND_RAM_LOG_PAGES_PER_BLOCK, i, bad,
        reads, writes, erases);
    }

  return;
}

static inline void nand_ram_status(void)
{
#ifdef CONFIG_MTD_NAND_RAM_DEBUG
  if (nand_ram_ins_i % NAND_RAM_STATUS_LEVEL == 0)
    {
      nand_ram_storage_status();
    }
#endif
}

/****************************************************************************
 * Name: nand_ram_storage_init
 *
 * Description:
 *   Initializes the actual NAND Device that is emulated from RAM.
 *
 ****************************************************************************/

static void nand_ram_storage_init(void)
{
  int i;

  memset(nand_ram_flash_data, 0xff,
          sizeof(struct nand_ram_data_s) * NAND_RAM_N_PAGES);
  memset(nand_ram_flash_spare, 0,
          sizeof(struct nand_ram_spare_s) * NAND_RAM_N_PAGES);

  for (i = 0; i < NAND_RAM_N_PAGES; i++)
    {
      nand_ram_flash_spare[i].free = NAND_RAM_PAGE_FREE;
      nand_ram_flash_spare[i].bad = NAND_RAM_BLOCK_GOOD;
    }

  /* Bad blocks */

  for (i = 0;
        g_nand_ram_rand_bad_blk_indx[i] != 0 &&
        g_nand_ram_rand_bad_blk_indx[i] < NAND_RAM_N_BLOCKS;
        i++)
    {
      int j;

      for (j = 0; j < NAND_RAM_PAGES_PER_BLOCK; j++)
        {
          int page = (g_nand_ram_rand_bad_blk_indx[i] <<
                      NAND_RAM_LOG_PAGES_PER_BLOCK)+j;

          /* Set bad block marker to Anything but NAND_RAM_BLOCK_GOOD */

          nand_ram_flash_spare[page].bad = 0;
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_ram_eraseblock
 *
 * Description:
 *   Erases a block on the device.
 *
 * Input Parameters:
 *   raw: NAND MTD Device raw structure.
 *   block: Block number (0 indexing) to erase
 *
 * Returned Value:
 *   0: Successful
 *   < 0: Error
 *
 ****************************************************************************/

int nand_ram_eraseblock(FAR struct nand_raw_s *raw, off_t block)
{
  int      i;
  uint32_t start_page;
  uint32_t end_page;

  start_page  = block << NAND_RAM_LOG_PAGES_PER_BLOCK;
  end_page    = start_page + NAND_RAM_PAGES_PER_BLOCK;

  nxmutex_lock(&nand_ram_dev_mut);
  nand_ram_ins_i++;

  NAND_RAM_LOG(
    "[LOWER %" PRIu64 " | %s] Block %" PRIi32 ", Start Page: %" PRIi32
    ", Last Page: %" PRIi32, nand_ram_ins_i, "eraseblock", block, start_page,
    end_page - 1);
  nand_ram_status();

  /* [start_page, end_page) is cleared (all bits are set) */

  memset(nand_ram_flash_data + start_page, 0xff,
        (end_page - start_page) * sizeof(struct nand_ram_data_s));
  for (i = start_page; i < end_page; i++)
    {
      nand_ram_flash_spare[i].n_erase++;
      nand_ram_flash_spare[i].free = NAND_RAM_PAGE_FREE;
    }

  NAND_RAM_LOG("[LOWER %" PRIu64 " | %s] Done\n", nand_ram_ins_i,
               "eraseblock");

  nxmutex_unlock(&nand_ram_dev_mut);

  return OK;
}

/****************************************************************************
 * Name: nand_ram_rawread
 *
 * Description:
 *   Reads a page from the device.
 *
 * Input Parameters:
 *   raw: NAND MTD Device raw structure.
 *   block: Block number (0 indexing) to erase
 *   page: Page number (0 indexing) in (relative to) that block
 *   data: Preallocated memory where the data will be copied to
 *   spare: Preallocated memory where the spare data will be copied to
 *
 * Returned Value:
 *   0: Successful
 *
 ****************************************************************************/

int nand_ram_rawread(FAR struct nand_raw_s *raw, off_t block,
                      unsigned int page, FAR void *data, FAR void *spare)
{
  int                     ret;
  uint32_t                read_page;
  struct nand_ram_data_s  *read_page_data;
  struct nand_ram_spare_s *read_page_spare;

  ret             = OK;
  read_page       = (block << NAND_RAM_LOG_PAGES_PER_BLOCK) + page;
  read_page_data  = nand_ram_flash_data + read_page;
  read_page_spare = nand_ram_flash_spare + read_page;

  nxmutex_lock(&nand_ram_dev_mut);
  nand_ram_ins_i++;

  NAND_RAM_LOG("[LOWER %" PRIu64 " | %s] Page %" PRIi32 "\n",
              nand_ram_ins_i, "rawread", read_page);
  nand_ram_status();

  if (nand_ram_flash_spare[read_page].bad != NAND_RAM_BLOCK_GOOD)
    {
      ret = -EFAULT;
      NAND_RAM_LOG("[LOWER %" PRIu64 " | %s] Failed: %s\n",
                    nand_ram_ins_i, "rawread", EFAULT_STR);
      goto errout;
    }

  nand_ram_flash_spare[read_page].n_read++;

  if (data != NULL)
    {
      if (nand_ram_flash_spare[read_page].free == NAND_RAM_PAGE_FREE)
        {
          memset(data, 0, NAND_RAM_PAGE_SIZE);
        }
      else
        {
          memcpy(data, (const void *)read_page_data->page,
                 NAND_RAM_PAGE_SIZE);
        }
    }

  if (spare != NULL)
    {
      memcpy(spare, (const void *)read_page_spare, NAND_RAM_SPARE_SIZE);
    }

  NAND_RAM_LOG("[LOWER %" PRIu64 " | %s] Done\n", nand_ram_ins_i, "rawread");

errout:
  nxmutex_unlock(&nand_ram_dev_mut);

  return ret;
}

/****************************************************************************
 * Name: nand_ram_rawread
 *
 * Description:
 *   Writes a page to the device.
 *
 * Input Parameters:
 *   raw: NAND MTD Device raw structure.
 *   block: Block number (0 indexing) to erase
 *   page: Page number (0 indexing) in (relative to) that block
 *   data: Preallocated memory where the data will be copied to
 *   spare: Preallocated memory where the spare data will be copied to
 *
 * Returned Value:
 *   0: Successful
 *   -EACCESS: The page's block needs to be erased first before writing to it
 *
 ****************************************************************************/

int nand_ram_rawwrite(FAR struct nand_raw_s *raw, off_t block,
                      unsigned int page, FAR const void *data,
                      FAR const void *spare)
{
  int                     ret;
  uint32_t                write_page;
  struct nand_ram_data_s  *write_page_data;
  struct nand_ram_spare_s *write_page_spare;

  ret               = OK;
  write_page        = (block << NAND_RAM_LOG_PAGES_PER_BLOCK) + page;
  write_page_data   = nand_ram_flash_data + write_page;
  write_page_spare  = nand_ram_flash_spare + write_page;

  nxmutex_lock(&nand_ram_dev_mut);
  nand_ram_ins_i++;

  NAND_RAM_LOG("[LOWER %" PRIu64 " | %s] Page %" PRIi32 "\n",
                nand_ram_ins_i, "rawwrite", write_page);
  nand_ram_status();

  if (nand_ram_flash_spare[write_page].free != NAND_RAM_PAGE_FREE)
    {
      ret = -EACCES;
      NAND_RAM_LOG("[LOWER %" PRIu64 " | %s] Failed: %s\n",
                    nand_ram_ins_i, "rawwrite", EACCES_STR);
      goto errout;
    }

  nand_ram_flash_spare[write_page].n_write++;
  nand_ram_flash_spare[write_page].free = NAND_RAM_PAGE_WRITTEN;

  memset((FAR void *)write_page_data->page, 0, NAND_RAM_PAGE_SIZE);
  if (data != NULL)
    {
      memcpy((FAR void *)write_page_data->page, data, NAND_RAM_PAGE_SIZE);
    }

  if (spare != NULL)
    {
      memcpy((FAR void *)write_page_spare, data, NAND_RAM_SPARE_SIZE);
    }

  NAND_RAM_LOG("[LOWER %" PRIu64 " | %s] Done\n", nand_ram_ins_i,
               "rawwrite");

errout:
  nxmutex_unlock(&nand_ram_dev_mut);

  return ret;
}

/****************************************************************************
 * Name: nand_ram_init
 *
 * Description:
 *   Driver init.
 *
 * Input Parameters:
 *   raw: NAND MTD Device raw structure.
 *
 * Returned Value:
 *   A non-NULL MTD driver instance is returned on success.  NULL is
 *   returned on any failure.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *nand_ram_initialize(struct nand_raw_s *raw)
{
  NAND_RAM_LOG("[LOWER | %s]\n", "initialize");

  nand_ram_storage_init();
  nxmutex_init(&nand_ram_dev_mut);

  raw->model.devid     = 123;
  raw->model.pagesize  = NAND_RAM_PAGE_SIZE;
  raw->model.sparesize = NAND_RAM_SPARE_SIZE;
  raw->model.devsize   = NAND_RAM_SIZE / (1024 * 1024);
  raw->model.blocksize = NAND_RAM_BLOCK_SIZE / 1024;
  raw->model.scheme    = &g_nand_sparescheme512;

  raw->eraseblock      = nand_ram_eraseblock;
  raw->rawread         = nand_ram_rawread;
  raw->rawwrite        = nand_ram_rawwrite;

  return nand_raw_initialize(raw);
}
