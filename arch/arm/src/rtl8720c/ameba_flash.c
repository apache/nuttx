/****************************************************************************
 * arch/arm/src/rtl8720c/ameba_flash.c
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
#include <nuttx/arch.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ameba_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Identifies the flash sector and block size */

#define AMEBA_SECTOR_SHIFT           (12)
#define AMEBA_SECTOR_SIZE            (1 << 12)     /* Sector size 1 << 12 = 4KB */
#define AMEBA_PAGE_SHIFT             (8)
#define AMEBA_PAGE_SIZE              (1 << 8)      /* Block size 1 << 8 = 256B */

/* Flash Layout */

#define AMEBA_SECTOR_TOTAL_SIZE      (2048 * 1024) /* Total flash size */
#define AMEBA_SECTOR_SYSTEM_SIZE     (16   * 1024)
#define AMEBA_SECTOR_BOOT_SIZE       (32   * 1024)
#define AMEBA_SECTOR_FIRMWARE1_SIZE  (864  * 1024)
#define AMEBA_SECTOR_FIRMWARE2_SIZE  (864  * 1024)
#define AMEBA_SECTOR_OTA_SIZE        (236  * 1024)
#define AMEBA_SECTOR_DATA_SIZE       (24   * 1024)
#define AMEBA_SECTOR_BLUETOOTH_SIZE  (12   * 1024)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum
{
  SPICONEIOMODE = 0,       /* !< Define One IO mode, 1-1-1 */
  SPICDUALOUTPUTMODE = 1,  /* !< Define Dual Output mode, 1-1-2 */
  SPICDUALIOMODE = 2,      /* !< Define Dual IO mode, 1-2-2 */
  SPICQUADOUTPUTMODE = 3,  /* !< Define Quad Output mode, 1-1-4 */
  SPICQUADIOMODE = 4,      /* !< Define Quad IO mode, 1-4-4 */
  SPICQPIMODE = 5,         /* !< Define QPI mode, 4-4-4 */
};

struct ameba_flash_dev_s
{
  struct mtd_dev_s  mtd;
  void              *adaptor;
  uint32_t baseaddr;
  uint16_t nsectors;
};

extern const hal_flash_func_stubs_t hal_flash_stubs;
extern hal_spic_adaptor_t           hal_spic_adaptor;
extern hal_spic_adaptor_t           *pglob_spic_adaptor;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct partition_s ptable[5] =
{
  {
    .name       = "fw1",
    .firstblock = AMEBA_SECTOR_SYSTEM_SIZE +
    AMEBA_SECTOR_BOOT_SIZE,
    .nblocks    = AMEBA_SECTOR_FIRMWARE1_SIZE,
  },
  {
    .name       = "fw2",
    .firstblock = AMEBA_SECTOR_SYSTEM_SIZE +
    AMEBA_SECTOR_BOOT_SIZE +
    AMEBA_SECTOR_FIRMWARE1_SIZE,
    .nblocks    = AMEBA_SECTOR_FIRMWARE2_SIZE,
  },
  {
    .name       = "ota",
    .firstblock = AMEBA_SECTOR_SYSTEM_SIZE +
    AMEBA_SECTOR_BOOT_SIZE +
    AMEBA_SECTOR_FIRMWARE1_SIZE +
    AMEBA_SECTOR_FIRMWARE2_SIZE,
    .nblocks    = AMEBA_SECTOR_OTA_SIZE,
  },
  {
    .name       = "data",
    .firstblock = AMEBA_SECTOR_SYSTEM_SIZE +
    AMEBA_SECTOR_BOOT_SIZE +
    AMEBA_SECTOR_FIRMWARE1_SIZE +
    AMEBA_SECTOR_FIRMWARE2_SIZE +
    AMEBA_SECTOR_OTA_SIZE,
    .nblocks    = AMEBA_SECTOR_DATA_SIZE,
  },
  {
    .name       = "bt",
    .firstblock = AMEBA_SECTOR_SYSTEM_SIZE +
    AMEBA_SECTOR_BOOT_SIZE +
    AMEBA_SECTOR_FIRMWARE1_SIZE +
    AMEBA_SECTOR_FIRMWARE2_SIZE +
    AMEBA_SECTOR_OTA_SIZE +
    AMEBA_SECTOR_DATA_SIZE,
    .nblocks    = AMEBA_SECTOR_BLUETOOTH_SIZE,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static irqstate_t flash_resource_lock(void)
{
  irqstate_t state;
  state = enter_critical_section();
  icache_disable();
  dcache_disable();
  return state;
}

static void flash_resource_unlock(irqstate_t state)
{
  dcache_enable();
  icache_enable();
  icache_invalidate();
  leave_critical_section(state);
}

static int ameba_flash_erase(struct mtd_dev_s *dev,
                             off_t startblock, size_t nblocks)
{
  struct ameba_flash_dev_s *priv = (struct ameba_flash_dev_s *)dev;
  uint32_t address = priv->baseaddr + (startblock << AMEBA_SECTOR_SHIFT);
  irqstate_t state;
  state = flash_resource_lock();
  hal_flash_stubs.hal_flash_sector_erase(priv->adaptor, address);
  flash_resource_unlock(state);
  return nblocks;
}

static ssize_t ameba_flash_bread(struct mtd_dev_s *dev,
                                 off_t startblock,
                                 size_t nblocks,
                                 uint8_t *buf)
{
  struct ameba_flash_dev_s *priv = (struct ameba_flash_dev_s *)dev;
  uint32_t address = priv->baseaddr + (startblock << AMEBA_PAGE_SHIFT);
  uint32_t length = nblocks << AMEBA_PAGE_SHIFT;
  irqstate_t state;
  state = flash_resource_lock();
  dcache_invalidate_by_addr((uint32_t *)(SPI_FLASH_BASE + address), length);
  hal_flash_stubs.hal_flash_stream_read(priv->adaptor, length, address, buf);
  flash_resource_unlock(state);
  return nblocks;
}

static ssize_t ameba_flash_bwrite(struct mtd_dev_s *dev,
                                  off_t startblock,
                                  size_t nblocks, const uint8_t *buf)
{
  struct ameba_flash_dev_s *priv = (struct ameba_flash_dev_s *)dev;
  uint32_t address = priv->baseaddr + (startblock << AMEBA_PAGE_SHIFT);
  uint32_t length = nblocks << AMEBA_PAGE_SHIFT;
  irqstate_t state;
  state = flash_resource_lock();
  hal_flash_stubs.hal_flash_burst_write(priv->adaptor,
                                        length, address, (uint8_t *)buf);
  flash_resource_unlock(state);
  return nblocks;
}

static int ameba_flash_spic_init(struct ameba_flash_dev_s *priv)
{
  hal_status_t status;
  irqstate_t state;
  if (pglob_spic_adaptor == NULL)
    {
      status = spic_init(&hal_spic_adaptor, SPICDUALIOMODE,
                         &(hal_spic_adaptor.flash_pin_sel));
      if (status != HAL_OK)
        {
          DBG_SPIF_ERR("flash_init err(%d)\r\n", status);
        }

      return status;
    }

  priv->adaptor = pglob_spic_adaptor;
  state = flash_resource_lock();
  hal_flash_stubs.hal_flash_read_id(pglob_spic_adaptor);
  flash_resource_unlock(state);
  if ((pglob_spic_adaptor->flash_id[0] == 0x0)
      || (pglob_spic_adaptor->flash_id[0] == 0xff))
    {
      return -EPERM;
    }

  return 0;
}

static int ameba_flash_ioctl(struct mtd_dev_s *dev,
                             int cmd, unsigned long arg)
{
  struct ameba_flash_dev_s *priv = (struct ameba_flash_dev_s *)dev;
  irqstate_t state;
  int ret = OK;
  switch (cmd)
    {
    case MTDIOC_GEOMETRY:
    {
      struct mtd_geometry_s *geo =
        (struct mtd_geometry_s *)((uintptr_t)arg);
      if (geo)
        {
          memset(geo, 0, sizeof(*geo));

          geo->blocksize    = AMEBA_PAGE_SIZE;
          geo->erasesize    = AMEBA_SECTOR_SIZE;
          geo->neraseblocks = priv->nsectors;
          finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                geo->blocksize, geo->erasesize, geo->neraseblocks);
        }
    }

    break;
    case BIOC_PARTINFO:
    {
      struct partition_info_s *info =
        (struct partition_info_s *)arg;
      if (info != NULL)
        {
          info->numsectors  = priv->nsectors *
                              AMEBA_SECTOR_SIZE / AMEBA_PAGE_SIZE;
          info->sectorsize  = AMEBA_PAGE_SIZE;
          info->startsector = 0;
          info->parent[0]   = '\0';
        }
    }

    break;
    case MTDIOC_BULKERASE:
    {
      state = flash_resource_lock();
      hal_flash_stubs.hal_flash_64k_block_erase(priv->adaptor,
                                                priv->baseaddr);
      flash_resource_unlock(state);
    }

    break;
    default:
    {
      ret = -ENOTTY;
    }

    break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ameba_flash_initialize
 *
 * Description:
 *  Create and initialize an ameba spic flash MTD device instance
 *  that can be used to access the userdata memory.
 *
 ****************************************************************************/

static struct mtd_dev_s *ameba_flash_initialize(void)
{
  struct ameba_flash_dev_s *priv;
  priv = kmm_zalloc(sizeof(struct ameba_flash_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->mtd.erase    = ameba_flash_erase;
  priv->mtd.bread    = ameba_flash_bread;
  priv->mtd.bwrite   = ameba_flash_bwrite;
  priv->mtd.ioctl    = ameba_flash_ioctl;
  priv->mtd.name     = "ameba_flash";
  priv->baseaddr     = 0;
  priv->nsectors     = AMEBA_SECTOR_TOTAL_SIZE /
                       AMEBA_SECTOR_SIZE;
  if (ameba_flash_spic_init(priv) < 0)
    {
      ferr("ERROR: Flash Type Unrecognized\n");
      kmm_free(priv);
      priv = NULL;
    }

  return (struct mtd_dev_s *)priv;
}

static void ameba_partition_init(const struct partition_s *part,
                                 const void *path)
{
  char dev[32];
  snprintf(dev, sizeof(dev), "/dev/%s", part->name);
  register_mtdpartition(dev, 0, path,
                        part->firstblock / AMEBA_PAGE_SIZE,
                        part->nblocks / AMEBA_PAGE_SIZE);
}

void ameba_flash_init(void)
{
  const struct partition_s *table;
  char *path = "/dev/ameba_flash";
  struct mtd_dev_s *mtd;
  mtd = ameba_flash_initialize();
  if (mtd == NULL)
    {
      return;
    }

  register_mtddriver(path, mtd, 0, mtd);
  for (table = &ptable[0]; table->nblocks; table++)
    {
      ameba_partition_init(table, path);
    }
}

