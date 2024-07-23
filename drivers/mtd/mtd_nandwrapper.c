/****************************************************************************
 * drivers/mtd/mtd_nandwrapper.c
 * This deals with the wrapper over the upper half of the driver, to enable
 * logging for debugging, and essentially passes the parameters right to the
 * actual upper half of the NAND Flash device driver without changing them.
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

#include <assert.h>
#include <stddef.h>

#include <nuttx/config.h>
#include <nuttx/mtd/nand.h>
#include <nuttx/mtd/nand_ram.h>
#include <nuttx/mtd/nand_scheme.h>

#include <nuttx/mtd/nand_wrapper.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define NAND_WRAPPER_DEBUG_1       1
#define NAND_WRAPPER_DEBUG_2       5
#define NAND_WRAPPER_DEBUG_3       10

#if CONFIG_MTD_NAND_WRAPPER_DEBUG_LEVEL == 1
#define NAND_WRAPPER_DEBUG_LEVEL   NAND_WRAPPER_DEBUG_1
#elif CONFIG_MTD_NAND_WRAPPER_DEBUG_LEVEL == 2
#define NAND_WRAPPER_DEBUG_LEVEL   NAND_WRAPPER_DEBUG_2
#elif CONFIG_MTD_NAND_WRAPPER_DEBUG_LEVEL == 3
#define NAND_WRAPPER_DEBUG_LEVEL   NAND_WRAPPER_DEBUG_3
#endif /* CONFIG_MTD_NAND_WRAPPER_DEBUG_LEVEL */

#define NAND_WRAPPER_LOG(str, ...)                                    \
          {                                                           \
            if(nand_wrapper_ins_i % NAND_WRAPPER_DEBUG_LEVEL == 0)    \
              {                                                       \
                syslog(LOG_DEBUG, "nand_wrapper: " str, __VA_ARGS__); \
              }                                                       \
          }

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t nand_wrapper_ins_i = 0; /* Instruction counter */
static mutex_t  nand_wrapper_dev_mut;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * External Functions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nand_wrapper_erase
 *
 * Description:
 *   Wrapper for NAND MTD erase method.
 *
 * Input Parameters:
 *   dev: NAND MTD Device (with an actual type of `nand_wrapper_dev_s`
 *   startblock: Block number (0-indexing) to start erasing
 *   nblocks: Number of blocks to erase
 *
 * Returned Value:
 *   0: Successful
 *   < 0: Error
 *
 * Assumptions/Limitations:
 *   This assumes `dev` is specifically of type `struct nand_wrapper_dev_s *`
 *
 ****************************************************************************/

int nand_wrapper_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks)
{
  int ret;
  FAR struct nand_wrapper_dev_s *nand_dev;

  nand_dev = (struct nand_wrapper_dev_s *)dev;

  nxmutex_lock(&nand_wrapper_dev_mut);
  nand_wrapper_ins_i++;
  NAND_WRAPPER_LOG("[UPPER %lu | %s] Startblock: %d, N Blocks: %ld\n",
                    nand_wrapper_ins_i, "erase", startblock, nblocks);
  DEBUGASSERT(nand_dev && nand_dev->under.mtd.erase);

  ret = nand_dev->under.mtd.erase(dev, startblock, nblocks);

  if (ret >= 0)
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Done\n",
                        nand_wrapper_ins_i, "erase");
    }
  else
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Failed: %d!\n",
                        nand_wrapper_ins_i, "erase", ret);
    }

  nxmutex_unlock(&nand_wrapper_dev_mut);

  return ret;
}

/****************************************************************************
 * Name: nand_wrapper_bread
 *
 * Description:
 *   Wrapper for NAND MTD bread method.
 *
 * Input Parameters:
 *   dev: MTD Device
 *   startpage: Page number (0-indexing) to start reading
 *   npages: Number of pages to read.
 *   buffer: Preallocated memory where the data will be copied to
 *
 * Returned Value:
 *   0: Successful
 *   < 0: Error
 *
 * Assumptions/Limitations:
 *   This assumes `dev` is specifically of type `struct nand_wrapper_dev_s *`
 *
 ****************************************************************************/

ssize_t nand_wrapper_bread(FAR struct mtd_dev_s *dev, off_t startpage,
                      size_t npages, FAR uint8_t *buffer)
{
  int ret;
  FAR struct nand_wrapper_dev_s *nand_dev;

  nand_dev = (struct nand_wrapper_dev_s *)dev;

  nxmutex_lock(&nand_wrapper_dev_mut);
  nand_wrapper_ins_i++;
  NAND_WRAPPER_LOG("[UPPER %lu | %s] "
                    "Startblock: %d, N Pages: %ld, Buffer: %p\n",
                    nand_wrapper_ins_i, "bread", startpage, npages, buffer);
  DEBUGASSERT(nand_dev && nand_dev->under.mtd.bread);

  ret = nand_dev->under.mtd.bread(dev, startpage, npages, buffer);

  if (ret >= 0)
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Done\n",
                        nand_wrapper_ins_i, "bread");
    }
  else
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Failed: %d!\n",
                        nand_wrapper_ins_i, "bread", ret);
    }

  nxmutex_unlock(&nand_wrapper_dev_mut);

  return ret;
}

/****************************************************************************
 * Name: nand_wrapper_bwrite
 *
 * Description:
 *   Wrapper for NAND MTD bwrite method.
 *
 * Input Parameters:
 *   dev: MTD Device
 *   startpage: Page number (0-indexing) to start reading
 *   npages: Number of pages to read.
 *   buffer: Data which will be written to the device
 *
 * Returned Value:
 *   0: Successful
 *   < 0: Error
 *
 * Assumptions/Limitations:
 *   This assumes the length of `buffer` would be the same as the size of
 *   `npages * block_size`. This also assumes `dev` is specifically of
 *   type `struct nand_wrapper_dev_s *`
 *
 ****************************************************************************/

ssize_t nand_wrapper_bwrite(FAR struct mtd_dev_s *dev, off_t startpage,
                        size_t npages, FAR const uint8_t *buffer)
{
  int ret;
  FAR struct nand_wrapper_dev_s *nand_dev;

  nand_dev = (struct nand_wrapper_dev_s *)dev;

  nxmutex_lock(&nand_wrapper_dev_mut);
  nand_wrapper_ins_i++;
  NAND_WRAPPER_LOG("[UPPER %lu | %s] "
                    "Startblock: %d, N Pages: %ld, Buffer: %p \n",
                    nand_wrapper_ins_i, "bwrite", startpage, npages, buffer);
  DEBUGASSERT(nand_dev && nand_dev->under.mtd.bwrite);

  ret = nand_dev->under.mtd.bwrite(dev, startpage, npages, buffer);

  if (ret >= 0)
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Done\n",
                        nand_wrapper_ins_i, "bwrite");
    }
  else
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Failed: %d!\n",
                        nand_wrapper_ins_i, "bwrite", ret);
    }

  nxmutex_unlock(&nand_wrapper_dev_mut);

  return ret;
}

/****************************************************************************
 * Name: nand_wrapper_ioctl
 *
 * Description:
 *   Wrapper for NAND MTD ioctl method.
 *
 * Input Parameters:
 *   dev: MTD Device
 *   cmd: Command for IOCTL
 *   arg: Any argument required by command
 *
 * Returned Value:
 *   0: Successful
 *   < 0: Error
 *
 * Assumptions/Limitations:
 *   This assumes `dev` is specifically of type `struct nand_wrapper_dev_s *`
 *
 ****************************************************************************/

int nand_wrapper_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                            unsigned long arg)
{
  int ret;
  FAR struct nand_wrapper_dev_s *nand_dev;

  nand_dev = (struct nand_wrapper_dev_s *)dev;

  nxmutex_lock(&nand_wrapper_dev_mut);
  nand_wrapper_ins_i++;
  NAND_WRAPPER_LOG("[UPPER %lu | %s] Command: %d, Arg : %ld\n",
                    nand_wrapper_ins_i, "ioctl", cmd, arg);
  DEBUGASSERT(nand_dev && nand_dev->under.mtd.ioctl);

  ret = nand_dev->under.mtd.ioctl(dev, cmd, arg);

  if (ret >= 0)
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Done\n",
                        nand_wrapper_ins_i, "ioctl");
    }
  else
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Failed: %d!\n",
                        nand_wrapper_ins_i, "ioctl", ret);
    }

  nxmutex_unlock(&nand_wrapper_dev_mut);

  return ret;
}

/****************************************************************************
 * Name: nand_wrapper_isbad
 *
 * Description:
 *   Wrapper for NAND MTD isbad method.
 *
 * Input Parameters:
 *   dev: MTD Device
 *   block: Block number (0-indexing) to check if it is bad
 *
 * Returned Value:
 *   0: Successful
 *   < 0: Error
 *
 * Assumptions/Limitations:
 *   This assumes `dev` is specifically of type `struct nand_wrapper_dev_s *`
 *
 ****************************************************************************/

int nand_wrapper_isbad(FAR struct mtd_dev_s *dev, off_t block)
{
  int ret;
  FAR struct nand_wrapper_dev_s *nand_dev;

  nand_dev = (struct nand_wrapper_dev_s *)dev;

  nxmutex_lock(&nand_wrapper_dev_mut);
  nand_wrapper_ins_i++;
  NAND_WRAPPER_LOG("[UPPER %lu | %s] Blocks: %d\n",
                    nand_wrapper_ins_i, "isbad", block);
  DEBUGASSERT(nand_dev && nand_dev->under.mtd.isbad);

  ret = nand_dev->under.mtd.isbad(dev, block);

  if (ret >= 0)
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Done\n",
                        nand_wrapper_ins_i, "isbad");
    }
  else
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Failed: %d!\n",
                        nand_wrapper_ins_i, "isbad", ret);
    }

  nxmutex_unlock(&nand_wrapper_dev_mut);

  return ret;
}

/****************************************************************************
 * Name: nand_wrapper_markbad
 *
 * Description:
 *   Wrapper for NAND MTD markbad method.
 *
 * Input Parameters:
 *   dev: MTD Device
 *   block: Block number (0-indexing) to mark it as bad
 *
 * Returned Value:
 *   0: Successful
 *   < 0: Error
 *
 * Assumptions/Limitations:
 *   This assumes `dev` is specifically of type `struct nand_wrapper_dev_s *`
 *
 ****************************************************************************/

int nand_wrapper_markbad(FAR struct mtd_dev_s *dev, off_t block)
{
  int ret;
  FAR struct nand_wrapper_dev_s *nand_dev;

  nand_dev = (struct nand_wrapper_dev_s *)dev;

  nxmutex_lock(&nand_wrapper_dev_mut);
  nand_wrapper_ins_i++;
  NAND_WRAPPER_LOG("[UPPER %lu | %s] Blocks: %d\n",
                    nand_wrapper_ins_i, "markbad", block);
  DEBUGASSERT(nand_dev && nand_dev->under.mtd.markbad);

  ret = nand_dev->under.mtd.markbad(dev, block);

  if (ret >= 0)
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Done\n",
                        nand_wrapper_ins_i, "markbad");
    }
  else
    {
      NAND_WRAPPER_LOG("[UPPER %lu | %s] Failed: %d!\n",
                        nand_wrapper_ins_i, "markbad", ret);
    }

  nxmutex_unlock(&nand_wrapper_dev_mut);

  return ret;
}

/****************************************************************************
 * Name: nand_wrapper_initialize
 *
 * Description:
 *   Initializes wrapper.
 *
 * Returned Value:
 *   0: Successful
 *   < 0: Error
 *
 ****************************************************************************/

void nand_wrapper_initialize(void)
{
  nxmutex_init(&nand_wrapper_dev_mut);
}
