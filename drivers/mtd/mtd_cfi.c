/****************************************************************************
 * drivers/mtd/mtd_cfi.c
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
#include <stdint.h>
#include <errno.h>
#include <stdio.h>
#include <debug.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mtd/mtd.h>

#include "cfi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int     cfi_mtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks);
static ssize_t cfi_mtd_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buf);
static ssize_t cfi_mtd_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buf);
static ssize_t cfi_mtd_read(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t cfi_mtd_write(FAR struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, FAR const uint8_t *buffer);
#endif
static int     cfi_mtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cfi_mtd_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported.
 *
 ****************************************************************************/

static int cfi_mtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks)
{
  FAR struct cfi_dev_s *priv = (FAR struct cfi_dev_s *)dev;
  size_t endblock;

  /* The interface definition assumes that all erase blocks are the
   * same size. If that is not true for this particular device, then
   * transform the start block and nblocks as necessary.
   */

  endblock = cfi_find_block(priv, (startblock + nblocks) *
                            cfi_get_blocksize(priv, 0));
  startblock = cfi_find_block(priv, startblock *
                              cfi_get_blocksize(priv, 0));

  /* Erase the specified blocks and return status (OK or a negated errno) */

  return cfi_erase(priv, startblock, endblock - startblock);
}

/****************************************************************************
 * Name: cfi_mtd_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t cfi_mtd_bread(FAR struct mtd_dev_s *dev, off_t startpage,
                             size_t npages, FAR uint8_t *buf)
{
  FAR struct cfi_dev_s *priv = (FAR struct cfi_dev_s *)dev;
  off_t offset = startpage * priv->page_size;
  int ret;

  ret = cfi_read(priv, offset, npages * priv->page_size, buf);
  return ret < 0 ? ret : npages;
}

/****************************************************************************
 * Name: cfi_mtd_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t cfi_mtd_bwrite(FAR struct mtd_dev_s *dev, off_t startpage,
                              size_t npages, FAR const uint8_t *buf)
{
  FAR struct cfi_dev_s *priv = (FAR struct cfi_dev_s *)dev;
  off_t offset = startpage * priv->page_size;
  int ret;

  ret = cfi_write(priv, offset, npages * priv->page_size, buf);
  return ret < 0 ? ret : npages;
}

/****************************************************************************
 * Name: cfi_mtd_read
 *
 * Description:
 *   Read the specified number of bytes to the user provided buffer.
 *
 ****************************************************************************/

static ssize_t cfi_mtd_read(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct cfi_dev_s *priv = (FAR struct cfi_dev_s *)dev;
  int ret;

  ret = cfi_read(priv, offset, nbytes, buffer);
  return ret < 0 ? ret : nbytes;
}

/****************************************************************************
 * Name: cfi_mtd_write
 *
 * Description:
 *   Some FLASH parts have the ability to write an arbitrary number of
 *   bytes to an arbitrary offset on the device.  This method should be
 *   implement only for devices that support such access.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t cfi_mtd_write(FAR struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct cfi_dev_s *priv = (FAR struct cfi_dev_s *)dev;
  int ret;

  ret = cfi_write(priv, offset, nbytes, buffer);
  return ret < 0 ? ret : nbytes;
}
#endif

/****************************************************************************
 * Name: cfi_mtd_ioctl
 ****************************************************************************/

static int cfi_mtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct cfi_dev_s *priv = (FAR struct cfi_dev_s *)dev;
  int ret = OK;

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)arg;

          DEBUGASSERT(geo != NULL);
          memset(geo, 0, sizeof(*geo));
          geo->blocksize    = priv->page_size;
          geo->erasesize    = cfi_get_blocksize(priv, 0);
          geo->neraseblocks = cfi_get_total_blocknum(priv);
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;

          DEBUGASSERT(info != NULL);
          info->numsectors  = cfi_get_total_blocknum(priv);
          info->sectorsize  = cfi_get_blocksize(priv, 0);
          info->startsector = 0;
          info->parent[0]   = '\0';
        }
        break;

      case BIOC_XIPBASE:
        {
          FAR void **ppv = (FAR void**)arg;

          DEBUGASSERT(ppv != NULL);
          *ppv = (FAR void *)priv->base_addr;
        }
        break;

      case MTDIOC_BULKERASE:
        {
          ret = cfi_mtd_erase(dev, 0, cfi_get_total_blocknum(priv));
        }
        break;

      case MTDIOC_ERASESECTORS:
        {
          FAR struct mtd_erase_s *erase = (FAR struct mtd_erase_s *)arg;
          ret = cfi_mtd_erase(dev, erase->startblock, erase->nblocks);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = 0xff;
          break;
        }

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Name: cfi_initialize
 *
 * Description:
 *   Create and initialize an MTD device instance.  MTD devices are not
 *   registered in the file system, but are created as instances that can
 *   be bound to other functions (such as a block or character driver front
 *   end).
 *
 ****************************************************************************/

static FAR struct mtd_dev_s *
cfi_initialize(uintptr_t addr_base, uintptr_t addr_end, uint32_t bankwidth)
{
  FAR struct cfi_dev_s *cfi;

  /* Create an instance of the CFI MTD device state structure  */

  cfi = kmm_zalloc(sizeof(struct cfi_dev_s));
  if (cfi == NULL)
    {
      ferr("ERROR: Failed to allocate the CFI MTD state structure\n");
      return NULL;
    }

  cfi->mtd.erase = cfi_mtd_erase;
  cfi->mtd.bread = cfi_mtd_bread;
  cfi->mtd.bwrite = cfi_mtd_bwrite;
  cfi->mtd.read = cfi_mtd_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  cfi->mtd.write  = cfi_mtd_write;
#endif
  cfi->mtd.ioctl = cfi_mtd_ioctl;
  cfi->mtd.name = "cfi-flash";

  cfi->base_addr = addr_base;
  cfi->end_addr = addr_end;
  cfi->bankwidth = bankwidth;

  /* Check the CFI, and set other parameters */

  if (cfi_check(cfi) == OK)
    {
      return (FAR struct mtd_dev_s *)cfi;
    }

  ferr("ERROR: Not a CFI device!\n");
  kmm_free(cfi);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: register_cfi_driver
 *
 * Description:
 *   Initialize cfi MTD device instance and register it to vfs.
 *
 ****************************************************************************/

int register_cfi_driver(uintptr_t addr_base, uintptr_t addr_end,
                        uint32_t bankwidth, int id)
{
  int ret = -ENXIO;
  FAR struct mtd_dev_s *mtd;

  mtd = cfi_initialize(addr_base, addr_end, bankwidth);
  if (mtd != NULL)
    {
      char devname[32];

      snprintf(devname, sizeof(devname), "/dev/cfi-flash%d", id);
      ret = register_mtddriver(devname, mtd, 0777, NULL);
      if (ret < 0)
        {
          kmm_free(mtd);
        }
    }

  return ret;
}
