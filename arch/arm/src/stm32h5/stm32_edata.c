/****************************************************************************
 * arch/arm/src/stm32h5/stm32_edata.c
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

/* MTD driver for the flash high-cycle data (EDATA) area.  The low-level
 * access, including handling of the ECC errors raised when reading blank
 * EDATA, is in stm32h563xx_flash.c.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>
#include <inttypes.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include "stm32_edata.h"
#include "stm32_flash.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define EDATA_BLOCK_SIZE  2
#define EDATA_ERASESTATE  0xff

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* struct mtd_dev_s must be first so that the two can be cast freely */

struct stm32_edata_dev_s
{
  struct mtd_dev_s mtd;

  int          bank;      /* Physical flash bank (1 or 2) */
  unsigned int first;     /* First EDATA sector used (0..7) */
  unsigned int nsectors;  /* Number of EDATA sectors */
  uintptr_t    base;      /* Address of the first sector */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     edata_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks);
static ssize_t edata_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buf);
static ssize_t edata_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buf);
static ssize_t edata_read(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t edata_write(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR const uint8_t *buf);
#endif
static int     edata_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                           unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: edata_size
 ****************************************************************************/

static inline size_t edata_size(FAR struct stm32_edata_dev_s *priv)
{
  return priv->nsectors * STM32_EDATA_SECTOR_SIZE;
}

/****************************************************************************
 * Name: edata_erase
 ****************************************************************************/

static int edata_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                       size_t nblocks)
{
  FAR struct stm32_edata_dev_s *priv = (FAR struct stm32_edata_dev_s *)dev;
  size_t i;
  int ret;

  if (startblock < 0 || startblock + nblocks > priv->nsectors)
    {
      return -EINVAL;
    }

  for (i = 0; i < nblocks; i++)
    {
      ret = stm32_flash_edata_erase(priv->bank,
                                    priv->first + startblock + i);
      if (ret < 0)
        {
          return ret;
        }
    }

  return nblocks;
}

/****************************************************************************
 * Name: edata_bread
 ****************************************************************************/

static ssize_t edata_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR uint8_t *buf)
{
  FAR struct stm32_edata_dev_s *priv = (FAR struct stm32_edata_dev_s *)dev;
  off_t offset = startblock * EDATA_BLOCK_SIZE;
  size_t nbytes = nblocks * EDATA_BLOCK_SIZE;
  ssize_t ret;

  if (startblock < 0 || offset + nbytes > edata_size(priv))
    {
      return -EINVAL;
    }

  ret = stm32_flash_edata_read(priv->base + offset, buf, nbytes);
  return ret < 0 ? ret : nblocks;
}

/****************************************************************************
 * Name: edata_bwrite
 ****************************************************************************/

static ssize_t edata_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                            size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct stm32_edata_dev_s *priv = (FAR struct stm32_edata_dev_s *)dev;
  off_t offset = startblock * EDATA_BLOCK_SIZE;
  size_t nbytes = nblocks * EDATA_BLOCK_SIZE;
  ssize_t ret;

  if (startblock < 0 || offset + nbytes > edata_size(priv))
    {
      return -EINVAL;
    }

  ret = stm32_flash_edata_write(priv->base + offset, buf, nbytes);
  return ret < 0 ? ret : nblocks;
}

/****************************************************************************
 * Name: edata_read
 *
 * Description:
 *   Byte-oriented read.  EDATA can only be read in half-words, so an odd
 *   leading or trailing byte is read through a bounce buffer.
 *
 ****************************************************************************/

static ssize_t edata_read(FAR struct mtd_dev_s *dev, off_t offset,
                          size_t nbytes, FAR uint8_t *buf)
{
  FAR struct stm32_edata_dev_s *priv = (FAR struct stm32_edata_dev_s *)dev;
  uint8_t hword[EDATA_BLOCK_SIZE];
  size_t remaining = nbytes;
  size_t len;
  ssize_t ret;

  if (offset < 0 || offset + nbytes > edata_size(priv))
    {
      return -EINVAL;
    }

  while (remaining > 0)
    {
      if ((offset & 1) || remaining == 1)
        {
          ret = stm32_flash_edata_read(priv->base + (offset & ~1), hword,
                                       sizeof(hword));
          if (ret < 0)
            {
              return ret;
            }

          *buf = hword[offset & 1];
          len  = 1;
        }
      else
        {
          len = remaining & ~1;
          ret = stm32_flash_edata_read(priv->base + offset, buf, len);
          if (ret < 0)
            {
              return ret;
            }
        }

      offset    += len;
      buf       += len;
      remaining -= len;
    }

  return nbytes;
}

/****************************************************************************
 * Name: edata_write
 *
 * Description:
 *   Byte-oriented write.  EDATA is programmed in half-words, each only once
 *   per erase, so only half-word aligned writes are supported.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t edata_write(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR const uint8_t *buf)
{
  FAR struct stm32_edata_dev_s *priv = (FAR struct stm32_edata_dev_s *)dev;

  if (offset < 0 || offset + nbytes > edata_size(priv) ||
      ((offset | nbytes) & 1))
    {
      return -EINVAL;
    }

  return stm32_flash_edata_write(priv->base + offset, buf, nbytes);
}
#endif

/****************************************************************************
 * Name: edata_ioctl
 ****************************************************************************/

static int edata_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                       unsigned long arg)
{
  FAR struct stm32_edata_dev_s *priv = (FAR struct stm32_edata_dev_s *)dev;
  int ret = -EINVAL;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)arg;

          if (geo != NULL)
            {
              memset(geo, 0, sizeof(*geo));
              geo->blocksize    = EDATA_BLOCK_SIZE;
              geo->erasesize    = STM32_EDATA_SECTOR_SIZE;
              geo->neraseblocks = priv->nsectors;
              strlcpy(geo->model, "stm32h5-edata", sizeof(geo->model));
              ret = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        ret = edata_erase(dev, 0, priv->nsectors);
        if (ret > 0)
          {
            ret = OK;
          }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;

          if (result != NULL)
            {
              *result = EDATA_ERASESTATE;
              ret = OK;
            }
        }
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
 * Name: stm32_edata_initialize
 *
 * Description:
 *   Create an MTD device on the flash high-cycle data (EDATA) area of a
 *   physical flash bank.  See stm32_edata.h.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *stm32_edata_initialize(int bank,
                                             unsigned int nsectors)
{
  FAR struct stm32_edata_dev_s *priv;
  int ret;

  if ((bank != 1 && bank != 2) || nsectors < 1 ||
      nsectors > STM32_EDATA_BANK_NSECTORS)
    {
      ferr("ERROR: Invalid EDATA bank %d or sector count %u\n",
           bank, nsectors);
      return NULL;
    }

  ret = stm32_flash_edata_configure(bank, nsectors);
  if (ret < 0)
    {
      ferr("ERROR: Failed to configure EDATA in bank %d: %d\n", bank, ret);
      return NULL;
    }

  priv = kmm_zalloc(sizeof(struct stm32_edata_dev_s));
  if (priv == NULL)
    {
      return NULL;
    }

  priv->mtd.erase  = edata_erase;
  priv->mtd.bread  = edata_bread;
  priv->mtd.bwrite = edata_bwrite;
  priv->mtd.read   = edata_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = edata_write;
#endif
  priv->mtd.ioctl  = edata_ioctl;
  priv->mtd.name   = "edata";

  priv->bank     = bank;
  priv->first    = STM32_EDATA_BANK_NSECTORS - nsectors;
  priv->nsectors = nsectors;
  priv->base     = stm32_flash_edata_address(bank, priv->first);

  finfo("EDATA bank %d: %u sectors at %08" PRIxPTR "\n",
        bank, nsectors, priv->base);

  return &priv->mtd;
}
