/****************************************************************************
 * arch/arm/src/ameba/ameba_flash.c
 *
 *   Copyright (C) 2019 Xiaomi Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

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

struct ameba_flash_dev_s
{
  struct mtd_dev_s  mtd;
  void              *adaptor;
  uint32_t          baseaddr;
  uint16_t          nsectors;
};

extern const hal_flash_func_stubs_t hal_flash_stubs;
extern hal_spic_adaptor_t           hal_spic_adaptor;
extern hal_spic_adaptor_t           *pglob_spic_adaptor;

/************************************************************************************
 * Private Data
 ************************************************************************************/

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

/************************************************************************************
 * Private Functions
 ************************************************************************************/

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

static int ameba_flash_erase(FAR struct mtd_dev_s *dev,
    off_t startblock, size_t nblocks)
{
  FAR struct ameba_flash_dev_s *priv = (FAR struct ameba_flash_dev_s *)dev;
  uint32_t address = priv->baseaddr + (startblock << AMEBA_SECTOR_SHIFT);
  irqstate_t state;

  state = flash_resource_lock();
  hal_flash_stubs.hal_flash_sector_erase(priv->adaptor, address);
  flash_resource_unlock(state);

  return nblocks;
}

static ssize_t ameba_flash_bread(FAR struct mtd_dev_s *dev,
    off_t startblock, size_t nblocks, FAR uint8_t *buf)
{
  FAR struct ameba_flash_dev_s *priv = (FAR struct ameba_flash_dev_s *)dev;
  uint32_t address = priv->baseaddr + (startblock << AMEBA_PAGE_SHIFT);
  uint32_t length = nblocks << AMEBA_PAGE_SHIFT;
  irqstate_t state;

  state = flash_resource_lock();
  dcache_invalidate_by_addr((uint32_t *)(SPI_FLASH_BASE + address), length);
  hal_flash_stubs.hal_flash_stream_read(priv->adaptor, length, address, buf);
  flash_resource_unlock(state);

  return nblocks;
}

static ssize_t ameba_flash_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
    size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct ameba_flash_dev_s *priv = (FAR struct ameba_flash_dev_s *)dev;
  uint32_t address = priv->baseaddr + (startblock << AMEBA_PAGE_SHIFT);
  uint32_t length = nblocks << AMEBA_PAGE_SHIFT;
  irqstate_t state;

  state = flash_resource_lock();
  hal_flash_stubs.hal_flash_burst_write(priv->adaptor, length, address, (uint8_t *)buf);
  flash_resource_unlock(state);

  return nblocks;
}

static int ameba_flash_spic_init(struct ameba_flash_dev_s *priv)
{
  hal_status_t status;
  irqstate_t state;

  if (pglob_spic_adaptor == NULL)
    {
      status = spic_init(&hal_spic_adaptor, SpicDualIOMode,
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
      || (pglob_spic_adaptor->flash_id[0] == 0xFF))
    {
      return -EPERM;
    }

  return 0;
}

static int ameba_flash_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct ameba_flash_dev_s *priv = (FAR struct ameba_flash_dev_s *)dev;
  irqstate_t state;
  int ret = OK;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              geo->blocksize    = AMEBA_PAGE_SIZE;
              geo->erasesize    = AMEBA_SECTOR_SIZE;
              geo->neraseblocks = priv->nsectors;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;
      case MTDIOC_BULKERASE:
        {
          state = flash_resource_lock();
          hal_flash_stubs.hal_flash_64k_block_erase(priv->adaptor, priv->baseaddr);
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

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/****************************************************************************
 * Name: ameba_flash_initialize
 *
 * Description:
 *  Create and initialize an ameba spic flash MTD device instance
 *  that can be used to access the userdata memory.
 *
 ****************************************************************************/

static FAR struct mtd_dev_s *ameba_flash_initialize(void)
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

  return (FAR struct mtd_dev_s *)priv;
}

static void ameba_partition_init(FAR const struct partition_s *part,
                                 FAR const void *path)
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
  FAR struct mtd_dev_s *mtd;

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
