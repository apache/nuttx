/****************************************************************************
 * arch/arm/src/common/ameba/ameba_flash_mtd.c
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
 * NuttX MTD driver for the Ameba on-chip SPI NOR flash.
 *
 * This is the thin "flash backend" layer of the NuttX storage stack
 * (MTD -> littlefs -> file API / KV).  As on other SoCs whose boot flash is
 * not on a directly drivable SPI bus, it is the XIP flash managed by the
 * SDK's SPIC/ROM code and SHARED with the NP core.  So we wrap the SDK's
 * flash primitives, which already:
 *   - take the inter-core hardware semaphore and IPC-pause the NP
 *     (FLASH_Write_Lock) around erase/program, and
 *   - disable interrupts so no XIP fetch happens mid-operation.
 *
 *   read   -> FLASH_ReadStream   (memcpy from the XIP-mapped region)
 *   write  -> FLASH_WriteStream  (self-locked page program)
 *   erase  -> FLASH_EraseXIP     (self-locked 4 KiB sector erase)
 *
 * Only a sub-region of the flash is exposed (the SDK's VFS1 data partition),
 * so all addresses are biased by priv->base; littlefs can never touch the
 * firmware images.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include "ameba_flash_mtd.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define AMEBA_FLASH_ERASE_SECTOR  2     /* FLASH_Erase_Type: EraseSector     */
#define AMEBA_FLASH_ERASED_STATE  0xff  /* NOR erased byte value             */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ameba_mtd_dev_s
{
  struct mtd_dev_s mtd;      /* MTD interface -- must be the first member    */
  uint32_t         base;     /* Flash byte offset of the partition start     */
  uint32_t         nsectors; /* Number of erase sectors in the partition     */
};

/****************************************************************************
 * External Function Prototypes (SDK fwlib, ameba_flash_ram.c)
 ****************************************************************************/

extern int  FLASH_ReadStream(uint32_t address, uint32_t len, uint8_t *data);
extern int  FLASH_WriteStream(uint32_t address, uint32_t len, uint8_t *data);
extern void FLASH_EraseXIP(uint32_t erasetype, uint32_t address);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ameba_mtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks);
static ssize_t ameba_mtd_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, FAR uint8_t *buffer);
static ssize_t ameba_mtd_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                                size_t nblocks, FAR const uint8_t *buffer);
static ssize_t ameba_mtd_read(FAR struct mtd_dev_s *dev, off_t offset,
                              size_t nbytes, FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t ameba_mtd_write(FAR struct mtd_dev_s *dev, off_t offset,
                               size_t nbytes, FAR const uint8_t *buffer);
#endif
static int ameba_mtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_mtd_erase
 *
 * Description:
 *   Erase the specified erase blocks (units of AMEBA_FLASH_SECTOR_SIZE).
 *
 ****************************************************************************/

static int ameba_mtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks)
{
  FAR struct ameba_mtd_dev_s *priv = (FAR struct ameba_mtd_dev_s *)dev;
  size_t i;

  if (startblock + nblocks > priv->nsectors)
    {
      return -EINVAL;
    }

  for (i = 0; i < nblocks; i++)
    {
      uint32_t addr = priv->base +
                      (uint32_t)(startblock + i) * AMEBA_FLASH_SECTOR_SIZE;

      FLASH_EraseXIP(AMEBA_FLASH_ERASE_SECTOR, addr);
    }

  return (int)nblocks;
}

/****************************************************************************
 * Name: ameba_mtd_bread
 *
 * Description:
 *   Read the specified number of read/write blocks (units of
 *   AMEBA_FLASH_PAGE_SIZE).
 *
 ****************************************************************************/

static ssize_t ameba_mtd_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, FAR uint8_t *buffer)
{
  FAR struct ameba_mtd_dev_s *priv = (FAR struct ameba_mtd_dev_s *)dev;
  uint32_t addr = priv->base +
                  (uint32_t)startblock * AMEBA_FLASH_PAGE_SIZE;
  uint32_t len  = (uint32_t)nblocks * AMEBA_FLASH_PAGE_SIZE;

  if (FLASH_ReadStream(addr, len, buffer) != 1)
    {
      return -EIO;
    }

  return (ssize_t)nblocks;
}

/****************************************************************************
 * Name: ameba_mtd_bwrite
 *
 * Description:
 *   Write the specified number of read/write blocks.  The blocks must have
 *   been erased first (littlefs guarantees this at the erase-block level).
 *
 ****************************************************************************/

static ssize_t ameba_mtd_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                                size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct ameba_mtd_dev_s *priv = (FAR struct ameba_mtd_dev_s *)dev;
  uint32_t addr = priv->base +
                  (uint32_t)startblock * AMEBA_FLASH_PAGE_SIZE;
  uint32_t len  = (uint32_t)nblocks * AMEBA_FLASH_PAGE_SIZE;

  if (FLASH_WriteStream(addr, len, (uint8_t *)buffer) != 1)
    {
      return -EIO;
    }

  return (ssize_t)nblocks;
}

/****************************************************************************
 * Name: ameba_mtd_read
 ****************************************************************************/

static ssize_t ameba_mtd_read(FAR struct mtd_dev_s *dev, off_t offset,
                              size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct ameba_mtd_dev_s *priv = (FAR struct ameba_mtd_dev_s *)dev;

  if (offset + (off_t)nbytes >
      (off_t)(priv->nsectors * AMEBA_FLASH_SECTOR_SIZE))
    {
      return -EINVAL;
    }

  if (FLASH_ReadStream(priv->base + (uint32_t)offset, (uint32_t)nbytes,
                       buffer) != 1)
    {
      return -EIO;
    }

  return (ssize_t)nbytes;
}

/****************************************************************************
 * Name: ameba_mtd_write
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t ameba_mtd_write(FAR struct mtd_dev_s *dev, off_t offset,
                               size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct ameba_mtd_dev_s *priv = (FAR struct ameba_mtd_dev_s *)dev;

  if (offset + (off_t)nbytes >
      (off_t)(priv->nsectors * AMEBA_FLASH_SECTOR_SIZE))
    {
      return -EINVAL;
    }

  if (FLASH_WriteStream(priv->base + (uint32_t)offset, (uint32_t)nbytes,
                        (uint8_t *)buffer) != 1)
    {
      return -EIO;
    }

  return (ssize_t)nbytes;
}
#endif

/****************************************************************************
 * Name: ameba_mtd_ioctl
 ****************************************************************************/

static int ameba_mtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                           unsigned long arg)
{
  FAR struct ameba_mtd_dev_s *priv = (FAR struct ameba_mtd_dev_s *)dev;
  int ret = -ENOTTY;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo == NULL)
            {
              return -EINVAL;
            }

          memset(geo, 0, sizeof(*geo));
          geo->blocksize    = AMEBA_FLASH_PAGE_SIZE;
          geo->erasesize    = AMEBA_FLASH_SECTOR_SIZE;
          geo->neraseblocks = priv->nsectors;
          strncpy(geo->model, "ameba-nor", sizeof(geo->model) - 1);
          ret = OK;
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)((uintptr_t)arg);

          if (info == NULL)
            {
              return -EINVAL;
            }

          info->numsectors  = priv->nsectors *
                              (AMEBA_FLASH_SECTOR_SIZE /
                               AMEBA_FLASH_PAGE_SIZE);
          info->sectorsize  = AMEBA_FLASH_PAGE_SIZE;
          info->startsector = priv->base / AMEBA_FLASH_PAGE_SIZE;
          strncpy(info->parent, "", sizeof(info->parent));
          ret = OK;
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *state = (FAR uint8_t *)((uintptr_t)arg);

          *state = AMEBA_FLASH_ERASED_STATE;
          ret = OK;
        }
        break;

      case MTDIOC_BULKERASE:
        ret = ameba_mtd_erase(dev, 0, priv->nsectors);
        ret = (ret < 0) ? ret : OK;
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_flash_mtd_initialize
 ****************************************************************************/

FAR struct mtd_dev_s *ameba_flash_mtd_initialize(uint32_t offset,
                                                 uint32_t nbytes)
{
  FAR struct ameba_mtd_dev_s *priv;

  if ((offset % AMEBA_FLASH_SECTOR_SIZE) != 0 ||
      (nbytes % AMEBA_FLASH_SECTOR_SIZE) != 0 || nbytes == 0)
    {
      ferr("ERROR: misaligned partition off=0x%08lx size=0x%08lx\n",
           (unsigned long)offset, (unsigned long)nbytes);
      return NULL;
    }

  priv = (FAR struct ameba_mtd_dev_s *)kmm_zalloc(sizeof(*priv));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->base          = offset;
  priv->nsectors      = nbytes / AMEBA_FLASH_SECTOR_SIZE;

  priv->mtd.erase     = ameba_mtd_erase;
  priv->mtd.bread     = ameba_mtd_bread;
  priv->mtd.bwrite    = ameba_mtd_bwrite;
  priv->mtd.read      = ameba_mtd_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write     = ameba_mtd_write;
#endif
  priv->mtd.ioctl     = ameba_mtd_ioctl;
  priv->mtd.name      = "ameba-nor";

  return &priv->mtd;
}

/****************************************************************************
 * Name: ameba_flash_fs_initialize
 ****************************************************************************/

int ameba_flash_fs_initialize(void)
{
  FAR struct mtd_dev_s *mtd;
  int ret;

  mtd = ameba_flash_mtd_initialize(AMEBA_FLASH_VFS1_OFFSET,
                                   AMEBA_FLASH_VFS1_SIZE);
  if (mtd == NULL)
    {
      ferr("ERROR: ameba_flash_mtd_initialize failed\n");
      return -ENODEV;
    }

  ret = register_mtddriver(AMEBA_FLASH_FS_DEVPATH, mtd, 0755, NULL);
  if (ret < 0)
    {
      ferr("ERROR: register_mtddriver(%s) failed: %d\n",
           AMEBA_FLASH_FS_DEVPATH, ret);
      return ret;
    }

  /* Mount littlefs.  On a blank/never-formatted partition the first mount
   * fails; retry once with "forceformat" to lay down a fresh filesystem.
   */

  ret = nx_mount(AMEBA_FLASH_FS_DEVPATH, AMEBA_FLASH_FS_MOUNTPT,
                 "littlefs", 0, NULL);
  if (ret < 0)
    {
      finfo("littlefs mount failed (%d), formatting %s\n", ret,
            AMEBA_FLASH_FS_MOUNTPT);
      ret = nx_mount(AMEBA_FLASH_FS_DEVPATH, AMEBA_FLASH_FS_MOUNTPT,
                     "littlefs", 0, "forceformat");
      if (ret < 0)
        {
          ferr("ERROR: littlefs format/mount failed: %d\n", ret);
          return ret;
        }
    }

  return OK;
}
