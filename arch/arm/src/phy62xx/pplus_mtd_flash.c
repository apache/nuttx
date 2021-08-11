/****************************************************************************
 * arch/arm/src/phy62xx/pplus_mtd_flash.c
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

#include <sys/types.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>
#include "pplus_mtd_flash.h"
#include "flash.h"

#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MTD_ERASED_STATE            (0xff)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.
 * The struct mtd_dev_s must appear at the beginning of the definition
 * so that you can freely cast between pointers to struct mtd_dev_s and
 * struct pplus_fls_dev_s.
 */

struct pplus_fls_dev_s
{
  struct mtd_dev_s      mtd;          /* MTD interface */
  uint32_t              offset;       /* offset from flash start address */
  uint32_t              size;         /* avaliable size for MTD */
  uint16_t              nsectors;     /* Number of erase sectors */
  uint8_t               sectorshift;  /* Log2 of sector size */
  uint8_t               pageshift;    /* Log2 of page size */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  pplus_fls_readid(struct pplus_fls_dev_s *priv);
static int  pplus_fls_erase_sector(struct pplus_fls_dev_s *priv,
                                   off_t offset);
static int  pplus_fls_erase_chip(struct pplus_fls_dev_s *priv);

/* MTD driver methods */

static int  pplus_fls_erase(struct mtd_dev_s *dev,
                            off_t startblock,
                            size_t nblocks);
static ssize_t pplus_fls_bread(struct mtd_dev_s *dev,
                               off_t startblock,
                               size_t nblocks,
                               uint8_t *buf);
static ssize_t pplus_fls_bwrite(struct mtd_dev_s *dev,
                                off_t startblock,
                                size_t nblocks,
                                const uint8_t *buf);
static ssize_t pplus_fls_read(struct mtd_dev_s *dev,
                              off_t offset,
                              size_t nbytes,
                              uint8_t *buffer);
static int  pplus_fls_ioctl(struct mtd_dev_s *dev,
                            int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pplus_fls_readid
 ****************************************************************************/

static inline int pplus_fls_readid(struct pplus_fls_dev_s *priv)
{
  /* fixed size and flash type
   * 256KB
   * priv->sectorshift = 12;
   * priv->pageshift   = 8;
   * priv->nsectors    = 64;
   */

  return OK;
}

/****************************************************************************
 * Name:  pplus_fls_erase_sector
 ****************************************************************************/

static int pplus_fls_erase_sector(struct pplus_fls_dev_s *priv, off_t sector)
{
  off_t address;

  finfo("sector: %08lx\n", (unsigned long)sector);

  /* Get the address associated with the sector */

  address = (off_t)((sector << priv->sectorshift) + priv->offset);
  _HAL_CS_ALLOC_();
  HAL_ENTER_CRITICAL_SECTION();

  hal_flash_erase_sector(address);
  HAL_EXIT_CRITICAL_SECTION();
  return OK;
}

/****************************************************************************
 * Name:  pplus_fls_erase_chip
 ****************************************************************************/

static int pplus_fls_erase_chip(struct pplus_fls_dev_s *priv)
{
  off_t address = priv->offset;
  int i;

  /* Erase the whole chip */

  for (i = 0; i < priv->nsectors; i++)
    {
      _HAL_CS_ALLOC_();
      HAL_ENTER_CRITICAL_SECTION();
      hal_flash_erase_sector(address);
      HAL_EXIT_CRITICAL_SECTION();
      address += (1ul << priv->sectorshift);
    }

  return OK;
}

/****************************************************************************
 * Name: pplus_fls_erase
 ****************************************************************************/

static int pplus_fls_erase(struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks)
{
  struct pplus_fls_dev_s *priv = (struct pplus_fls_dev_s *)dev;
  size_t blocksleft = nblocks;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);
  while (blocksleft-- > 0)
    {
      /* Erase each sector */

      pplus_fls_erase_sector(priv, startblock);
      startblock++;
    }

  return (int)nblocks;
}

/****************************************************************************
 * Name: pplus_fls_bread
 ****************************************************************************/

static ssize_t pplus_fls_bread(struct mtd_dev_s *dev, off_t startblock,
                               size_t nblocks, uint8_t *buffer)
{
  struct pplus_fls_dev_s *priv = (struct pplus_fls_dev_s *)dev;
  int ret;
  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  /* On this device, we can handle the block read just like the byte-oriented
   * read
   */

  ret = hal_flash_read(priv->offset + (startblock << priv->pageshift),
      buffer, nblocks << priv->pageshift);

  if (ret < 0)
    {
      ferr("ERROR: pplus_fls_read_byte returned: %d\n", ret);

      /* return (ssize_t)ret; */
    }

  return nblocks;
}

/****************************************************************************
 * Name: pplus_fls_bwrite
 ****************************************************************************/

static ssize_t pplus_fls_bwrite(struct mtd_dev_s *dev, off_t startblock,
                                size_t nblocks, const uint8_t *buffer)
{
  struct pplus_fls_dev_s *priv = (struct pplus_fls_dev_s *)dev;

  finfo("startblock: %08lx nblocks: %d\n", (long)startblock, (int)nblocks);

  _HAL_CS_ALLOC_();
  HAL_ENTER_CRITICAL_SECTION();
  int ret = hal_flash_write(priv->offset + (startblock << priv->pageshift),
      (uint8_t *)buffer, nblocks << priv->pageshift);

  HAL_EXIT_CRITICAL_SECTION();
  if (ret)
    {
      ferr("ERROR: spif_write failed: %d\n", ret);
      return -ret;
    }

  return nblocks;
}

/****************************************************************************
 * Name: pplus_fls_read
 ****************************************************************************/

static ssize_t pplus_fls_read(struct mtd_dev_s *dev,
                              off_t offset,
                              size_t nbytes,
                              uint8_t *buffer)
{
  struct pplus_fls_dev_s *priv = (struct pplus_fls_dev_s *)dev;
  int ret;

  finfo("offset: %08lx nbytes: %d\n", (long)offset, (int)nbytes);

  ret = hal_flash_read(priv->offset + offset, buffer, nbytes);

  if (ret < 0)
    {
      ferr("ERROR: pplus_fls_read_byte returned: %d\n", ret);
      return (ssize_t)ret;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return (ssize_t)nbytes;
}

/****************************************************************************
 * Name: pplus_fls_ioctl
 ****************************************************************************/

static int pplus_fls_ioctl(struct mtd_dev_s *dev,
                           int cmd,
                           unsigned long arg)
{
  struct pplus_fls_dev_s *priv = (struct pplus_fls_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          struct mtd_geometry_s *geo =
              (struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but
               * the client will expect the device logic to do whatever is
               * necessary to make it appear so.
               */

              geo->blocksize    = (1 << priv->pageshift);
              geo->erasesize    = (1 << priv->sectorshift);
              geo->neraseblocks = priv->nsectors;
              ret               = OK;

              finfo("blocksize: %" PRId32 " erasesize: %" PRId32
              " neraseblocks: %" PRId32 "\n",
              geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
          break;
        }

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = pplus_fls_erase_chip(priv);
          break;
        }

      case MTDIOC_ERASESTATE:
        {
          uint8_t *result = (uint8_t *)arg;
          *result = MTD_ERASED_STATE;
          ret = OK;
        }
        break;

      default:
          ret = -ENOTTY; /* Bad/unsupported command */
          break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pplus_fls_initialize
 *
 * Description:
 *   Create an initialize MTD device instance for the internal FLASH.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 * Parameter:
 *  offset: offset from 0 of internal flash
 *  size:   avaiable size for NVM
 ****************************************************************************/

struct mtd_dev_s *pplus_fls_initialize(uint32_t offset, uint32_t size)
{
  struct pplus_fls_dev_s *priv;

  /* int ret; */

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per QuadSPI
   * device (only because of the QSPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same QuadSPI
   * bus.
   */

  priv = (struct pplus_fls_dev_s *)
          kmm_zalloc(sizeof(struct pplus_fls_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure (unsupported methods were
       * nullified by kmm_zalloc).
       */

      priv->mtd.erase  = pplus_fls_erase;
      priv->mtd.bread  = pplus_fls_bread;
      priv->mtd.bwrite = pplus_fls_bwrite;
      priv->mtd.read   = pplus_fls_read;
      priv->mtd.ioctl  = pplus_fls_ioctl;
      priv->mtd.name   = "pplus_nvm";

      priv->offset      = offset;
      priv->size        = size;
      priv->sectorshift = 12;
      priv->pageshift   = 8;
      priv->nsectors    = 32;

      /* Identify the FLASH chip and get its capacity */

      /* ret = pplus_fls_readid(priv); */

      pplus_fls_readid(priv);
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (struct mtd_dev_s *)priv;

  /* errout_with_priv: */

  /* kmm_free(priv); */

  /* return NULL; */
}
