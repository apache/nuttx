/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_sfc.c
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

#include <inttypes.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

/* Prototypes for Remote API */

int fw_fm_rawwrite(uint32_t offset, const void *buf, uint32_t size);
int fw_fm_rawverifywrite(uint32_t offset, const void *buf, uint32_t size);
int fw_fm_rawread(uint32_t offset, void *buf, uint32_t size);
int fw_fm_rawerasesector(uint32_t sector);

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_CXD56_SPIFLASHSIZE
#  define CONFIG_CXD56_SPIFLASHSIZE (16 * 1024 * 1024)
#endif

#define SECTOR_SHIFT 12
#define SECTOR_SIZE (1 << SECTOR_SHIFT)

#ifdef CONFIG_CXD56_SFC_PAGE_SHIFT_SIZE
#  define PAGE_SHIFT CONFIG_CXD56_SFC_PAGE_SHIFT_SIZE
#else
#  define PAGE_SHIFT 12
#endif
#define PAGE_SIZE (1 << PAGE_SHIFT)

/* Flash device information */

struct flash_controller_s
{
  struct mtd_dev_s mtd; /* MTD interface */
  uint32_t density;
};

static struct flash_controller_s g_sfc;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cxd56_erase
 ****************************************************************************/

static int cxd56_erase(struct mtd_dev_s *dev, off_t startblock,
                       size_t nblocks)
{
  int ret;
  size_t i;

  finfo("erase: %" PRIxOFF " (%u blocks)\n",
        startblock << PAGE_SHIFT, nblocks);

  for (i = 0; i < nblocks; i++)
    {
      ret = fw_fm_rawerasesector(startblock + i);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

static ssize_t cxd56_bread(struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, uint8_t *buffer)
{
  int ret;

  finfo("bread: %" PRIxOFF " (%u blocks)\n",
        startblock << PAGE_SHIFT, nblocks);

  ret = fw_fm_rawread(startblock << PAGE_SHIFT, buffer,
                      nblocks << PAGE_SHIFT);
  if (ret < 0)
    {
      return ret;
    }

  return nblocks;
}

static ssize_t cxd56_bwrite(struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, const uint8_t *buffer)
{
  int ret;

  finfo("bwrite: %" PRIxOFF " (%u blocks)\n",
        startblock << PAGE_SHIFT, nblocks);

#ifdef CONFIG_CXD56_SFC_VERIFY_WRITE
  ret = fw_fm_rawverifywrite(startblock << PAGE_SHIFT, buffer,
                          nblocks << PAGE_SHIFT);
#else
  ret = fw_fm_rawwrite(startblock << PAGE_SHIFT, buffer,
                    nblocks << PAGE_SHIFT);
#endif
  if (ret < 0)
    {
      return ret;
    }

  return nblocks;
}

static ssize_t cxd56_read(struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, uint8_t *buffer)
{
  int ret;

  finfo("read: %" PRIxOFF " (%u bytes)\n", offset, nbytes);

  ret = fw_fm_rawread(offset, buffer, nbytes);
  if (ret < 0)
    {
      return ret;
    }

  return nbytes;
}

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t cxd56_write(struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, const uint8_t *buffer)
{
  int ret;

  finfo("write: %" PRIxOFF " (%u bytes)\n", offset, nbytes);

#ifdef CONFIG_CXD56_SFC_VERIFY_WRITE
  ret = fw_fm_rawverifywrite(offset, buffer, nbytes);
#else
  ret = fw_fm_rawwrite(offset, buffer, nbytes);
#endif
  if (ret < 0)
    {
      return ret;
    }

  return nbytes;
}
#endif

static int cxd56_ioctl(struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  struct flash_controller_s *priv = (struct flash_controller_s *)dev;
  int ret                         = OK;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo =
            (struct mtd_geometry_s *)((uintptr_t)arg);
          finfo("cmd: GEOM\n");
          if (geo)
            {
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just
               * an array of fixed size blocks.
               * That is most likely not true, but the client will expect
               * the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize    = PAGE_SIZE;
              geo->erasesize    = SECTOR_SIZE;
              geo->neraseblocks = priv->density >> SECTOR_SHIFT;
              ret               = OK;

              finfo("blocksize: %" PRId32 " erasesize: %" PRId32
                    " neraseblocks: %" PRId32 "\n",
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
              info->numsectors  = priv->density / PAGE_SIZE;
              info->sectorsize  = PAGE_SIZE;
              info->startsector = 0;
              info->parent[0]   = '\0';
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          uint32_t sec  = 0;
          uint32_t last = priv->density >> SECTOR_SHIFT;

          finfo("cmd: ERASE\n");

          while (sec < last)
            {
              fw_fm_rawerasesector(sec);
              sec++;
            }
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

struct mtd_dev_s *cxd56_sfc_initialize(void)
{
  struct flash_controller_s *priv = &g_sfc;

  priv->mtd.erase  = cxd56_erase;
  priv->mtd.bread  = cxd56_bread;
  priv->mtd.bwrite = cxd56_bwrite;
  priv->mtd.read   = cxd56_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = cxd56_write;
#endif
  priv->mtd.ioctl  = cxd56_ioctl;

  /* TODO: Flash reserved area should be configurable dynamically. */

  priv->density = CONFIG_CXD56_SPIFLASHSIZE;

#ifdef CONFIG_SFC_SECTOR512

  /* Allocate a buffer for the erase block cache */

  priv->cache = (uint8_t *)kmm_malloc(SPIFI_BLKSIZE);
  if (!priv->cache)
    {
      /* Allocation failed! */

      /* Discard all of that work we just did and return NULL */

      ferr("ERROR: Allocation failed\n");
      return NULL;
    }
#endif

  return &priv->mtd;
}
