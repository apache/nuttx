/****************************************************************************
 * drivers/mtd/at25ee.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/eeprom/eeprom.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* For applications where a file system is used on the AT25EE, the tiny page
 * sizes will result in very inefficient EEPROM usage.  In such cases, it is
 * better if blocks are comprised of "clusters" of pages so that the file
 * system block size is, say, 256 or 512 bytes.
 * In any event, the block size *must* be an even multiple of the pages.
 */

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct at25ee_dev_s.
 */

struct at25ee_dev_s
{
  struct mtd_dev_s         mtd;    /* MTD interface            */
  FAR struct eeprom_dev_s *eepdev; /* eeprom/spi_xx25xx device */

  size_t    size;      /* Size in bytes, expanded from geometry             */
  blksize_t pgsize;    /* Write block size (in bytes), from geometry        */
  blksize_t sectsize;  /* Sector size (in bytes), expanded from geometry    */
  uint16_t  blocksize; /* Block size to report                              */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_AT25EE_ENABLE_BLOCK_ERASE
static int at25ee_erasepages(FAR struct at25ee_dev_s *priv, off_t startpage,
                             blkcnt_t npages);
#endif

/* MTD driver methods */

static int at25ee_erase(FAR struct mtd_dev_s *dev,
                        off_t startblock,
                        size_t nblocks);
static ssize_t at25ee_bread(FAR struct mtd_dev_s *dev,
                            off_t startblock,
                            size_t nblocks, FAR uint8_t *buf);
static ssize_t at25ee_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buf);
static ssize_t at25ee_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t at25ee_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buf);
#endif
static int at25ee_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                        unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at25ee_erasepages
 *
 * Description:
 *   Erase a number of pages of data
 *
 * Remark;
 *   The startpage and npages parameter are assumed to be valid. If they are
 *   not, the underlying EEPROM character driver will return an error.
 *
 * Input Parameters:
 *   priv       - a reference to the device structure
 *   startpage  - start page of the erase
 *   npages     - number of pages to erase
 *
 * Returned Value:
 *   Success (OK), or fail (negated error code)
 *
 ****************************************************************************/

#ifdef CONFIG_AT25EE_ENABLE_BLOCK_ERASE
static int at25ee_erasepages(FAR struct at25ee_dev_s *priv, off_t startpage,
                             blkcnt_t npages)
{
  blkcnt_t i;
  int      ret = OK;

  DEBUGASSERT(priv);

  for (i = 0; i < npages; ++i)
    {
      const unsigned long page = (unsigned long)(startpage + i);
      ret = priv->eepdev->ioctl(priv->eepdev, EEPIOC_PAGEERASE, page);
      if (ret < 0)
        {
          ferr("ERROR while erasing page #%ld (ret = %d)\n", page, ret);
          break;
        }
    }

  return ret;
}
#endif /* CONFIG_AT25EE_ENABLE_BLOCK_ERASE */

/****************************************************************************
 * Name: at25ee_erase
 *
 * Description:
 *   Erase a number of blocks of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   startblock - start block of the erase
 *   nblocks    - nblocks to erase
 *
 * Returned Value:
 *   Number of blocks erased, or fail (negated error code)
 *
 ****************************************************************************/

static int at25ee_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                        size_t nblocks)
{
#ifndef CONFIG_AT25EE_ENABLE_BLOCK_ERASE
  return (int)nblocks;
#else
  FAR struct at25ee_dev_s *priv    = (FAR struct at25ee_dev_s *)dev;
  blkcnt_t                 npages  = (blkcnt_t)nblocks;
  off_t                    startpg = startblock;

  DEBUGASSERT(dev);

  if ((startblock == 0) && ((nblocks * priv->blocksize) >= priv->size))
    {
      return priv->eepdev->ioctl(priv->eepdev, EEPIOC_CHIPERASE, 0);
    }

  if (priv->blocksize > priv->pgsize)
    {
      startpg *= (priv->blocksize / priv->pgsize);
      npages *= (priv->blocksize / priv->pgsize);
    }

  if ((startpg * priv->pgsize) >= priv->size)
    {
      return -EFAULT;
    }

  if (((startpg + npages) * priv->pgsize) > priv->size)
    {
      npages = (blkcnt_t)(priv->size / (size_t)priv->pgsize) - startpg;
    }

  finfo("startpg: %08lx npages: %d\n", (long)startpg, (int)npages);

  if (priv->pgsize != priv->sectsize)
    {
      const blkcnt_t pgpersect = priv->sectsize / priv->pgsize;
      const off_t    startsect = (startpg + (pgpersect - 1)) / pgpersect;
      const off_t    endsect   = (startpg + npages) / pgpersect;
      int            i;
      int            ret;

      /* Erase the pages not aligned to a sector */

      if ((startpg % pgpersect) != 0)
        {
          const blkcnt_t pgcnt = pgpersect - (startpg % pgpersect);
          ret                  = at25ee_erasepages(priv, startpg, pgcnt);
          if (ret < 0)
            {
              return ret;
            }
        }

      /* Erase by sectors */

      for (i = 0; i < (endsect - startsect); ++i)
        {
          const unsigned long sectorno = (unsigned long)(startsect + i);
          ret = priv->eepdev->ioctl(priv->eepdev, EEPIOC_SECTORERASE,
                                    sectorno);
          if (ret < 0)
            {
              ferr("ERROR while erasing sector #%ld (ret = %d)\n",
                   sectorno, ret);
              return ret;
            }
        }

      /* Erase the remaining pages */

      if ((endsect * priv->sectsize) < ((startpg + npages) * priv->pgsize))
        {
          const off_t    start = endsect * pgpersect;
          const blkcnt_t pgcnt = npages - (start - startpg);
          ret = at25ee_erasepages(priv, endsect * pgpersect, pgcnt);
          if (ret < 0)
            {
              return ret;
            }
        }
    }

  else
    {
      const int ret = at25ee_erasepages(priv, startpg, npages);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (priv->blocksize > priv->pgsize)
    {
      return (int)((npages * priv->pgsize) / priv->blocksize);
    }
  else
    {
      return (int)nblocks;
    }
#endif /* CONFIG_AT25EE_ENABLE_BLOCK_ERASE */
}

/****************************************************************************
 * Name: at25ee_read
 *
 * Description:
 *   Read a number of bytes of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   offset     - start of the memory to read
 *   nbytes     - number of bytes to read
 *   buffer     - pointer to variable to store the read data
 *
 * Returned Value:
 *   Size of the data read on success, a negated error code on failure
 *
 ****************************************************************************/

static ssize_t at25ee_read(FAR struct mtd_dev_s *dev, off_t offset,
                           size_t nbytes, FAR uint8_t *buf)
{
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;

  DEBUGASSERT(dev);
  DEBUGASSERT(buf);

  return priv->eepdev->read(priv->eepdev, offset, nbytes, buf);
}

/****************************************************************************
 * Name: at25ee_write
 *
 * Description:
 *   Write a number of bytes of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   offset     - start of the memory to write
 *   nbytes     - number of bytes to write
 *   buf        - pointer to buffer of data to write
 *
 * Returned Value:
 *   Size of the data written on success, a negated error code on failure
 *
 ****************************************************************************/

static ssize_t at25ee_write(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR const uint8_t *buf)
{
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;

  DEBUGASSERT(dev);
  DEBUGASSERT(buf);

  return priv->eepdev->write(priv->eepdev, offset, nbytes, buf);
}

/****************************************************************************
 * Name: at25ee_bread
 *
 * Description:
 *   Read a number of blocks of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   startblock - start block of the read
 *   nblocks    - nblocks to read
 *   buf        - pointer to variable to store the read data
 *
 * Returned Value:
 *   Number of blocks read on success, a negated error code on failure
 *
 ****************************************************************************/

static ssize_t at25ee_bread(FAR struct mtd_dev_s *dev,
                           off_t startblock,
                           size_t nblocks, FAR uint8_t *buf)
{
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;
  off_t                    offset;
  ssize_t                  nbytes;
  ssize_t                  ret;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv->blocksize > 0);

  offset = startblock * priv->blocksize;
  nbytes = nblocks * priv->blocksize;

  ret = at25ee_read(dev, offset, nbytes, buf);
  if (ret > 0)
    {
      ret /= priv->blocksize;
    }

  return ret;
}

/****************************************************************************
 * Name: at25ee_bwrite
 *
 * Description:
 *   Write a number of blocks of data.
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   startblock - starting block to write to
 *   nblocks    - nblocks to write
 *   buf        - pointer to data buffer to write
 *
 * Returned Value:
 *   Number of blocks written on success, a negated error code on failure
 *
 ****************************************************************************/

static ssize_t at25ee_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;
  off_t                    offset;
  ssize_t                  nbytes;
  ssize_t                  ret;

  DEBUGASSERT(dev);
  DEBUGASSERT(priv->blocksize > 0);

  offset = startblock * priv->blocksize;
  nbytes = nblocks * priv->blocksize;

  ret = at25ee_write(dev, offset, nbytes, buf);
  if (ret > 0)
    {
      ret /= priv->blocksize;
    }

  return ret;
}

/****************************************************************************
 * Name: at25ee_ioctl
 *
 * Description:
 *   IOCTLS relating to the EEPROM mtd device
 *
 * Input Parameters:
 *   dev        - a reference to the device structure
 *   cmd        - ioctl command
 *   arg        - ioctl argument
 *
 * Returned Value:
 *   Success (OK) or fail (negated error code)
 *
 ****************************************************************************/

static int at25ee_ioctl(FAR struct mtd_dev_s *dev,
                       int cmd,
                       unsigned long arg)
{
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)dev;
  int                      ret  = -EINVAL;

  DEBUGASSERT(dev);

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
         FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)
                                          ((uintptr_t)arg);
          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an array
               * of fixed size blocks.
               * That is most likely not true, but the client will expect the
               * device logic to do whatever is necessary to make it appear
               * so.
               *
               * blocksize:
               *   May be user defined.
               *   The block size for the at24XX devices may be larger than
               *   the page size in order to better support file systems.
               *   The read and write functions translate BLOCKS to pages
               *   for the small flash devices
               * erasesize:
               *   It has to be at least as big as the blocksize, bigger
               *   serves no purpose.
               * neraseblocks
               *   Note that the device size is in kilobits and must be
               *   scaled by 1024 / 8
               */

              geo->blocksize    = priv->blocksize;
              geo->erasesize    = priv->blocksize;
              geo->neraseblocks = priv->size / priv->blocksize;

              ret = OK;

              finfo("blocksize: %" PRId32 " erasesize: %" PRId32
                    " neraseblocks: %" PRId32 "\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
              (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = priv->size / priv->blocksize;
              info->sectorsize  = priv->blocksize;

              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          ret = priv->eepdev->ioctl(priv->eepdev, EEPIOC_CHIPERASE, 0);
        }
        break;

      case MTDIOC_SETSPEED:
        {
          ret = priv->eepdev->ioctl(priv->eepdev, EEPIOC_SETSPEED, arg);
        }
        break;

      default:
        {
          /* Tunnel the command to the EEPROM driver */

          ret = priv->eepdev->ioctl(priv->eepdev, cmd, arg);
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: at25ee_initialize
 *
 * Description:
 *   Create an initialized MTD device instance for an AT25 SPI EEPROM
 *   MTD devices are not registered in the file system, but are created
 *   as instances that can be bound to other functions
 *   (such as a block or character driver front end).
 *
 * Input Parameters:
 *   dev        - a reference to the spi device structure
 *   spi_devid  - SPI device ID to manage CS lines in board
 *   devtype    - device type, see eeprom/eeprom.h
 *   readonly   - sets block driver to be readonly
 *
 * Returned Value:
 *   Initialized device instance (success) or NULL (fail)
 *
 ****************************************************************************/

FAR struct mtd_dev_s *at25ee_initialize(FAR struct spi_dev_s *dev,
                                        uint16_t spi_devid,
                                        enum eeprom_25xx_e devtype,
                                        int readonly)
{
  FAR struct at25ee_dev_s *priv;
  struct eeprom_geometry_s geo;
  int                      ret;

  /* Initialize the EEPROM driver */

  FAR struct eeprom_dev_s *eepdev =
    ee25xx_initialize_as_eeprom_dev(dev, spi_devid, devtype, readonly);

  if (eepdev == NULL)
    {
      ferr("ERROR: Failed to initialize the EEPROM driver\n");
      return NULL;
    }

  /* Allocate the device structure */

  priv = (FAR struct at25ee_dev_s *)kmm_zalloc(sizeof(struct at25ee_dev_s));
  if (priv == NULL)
    {
      ferr("ERROR: Failed to allocate device structure\n");
      return NULL;
    }

  /* Retrieve the geometry */

  ret = eepdev->ioctl(eepdev, EEPIOC_GEOMETRY,
                      (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      ferr("ERROR retrieving the EEPROM geometry (ret = %d)\n", ret);
      goto cleanup;
    }

  /* Initialize the allocated structure */

  priv->eepdev   = eepdev;
  priv->size     = geo.pagesize * geo.npages;
  priv->pgsize   = geo.pagesize;
  priv->sectsize = geo.sectsize;

#ifdef CONFIG_USE_NATIVE_AT25EE_BLOCK_SIZE
  priv->blocksize = geo.pagesize;
#else
  if ((CONFIG_MANUAL_AT25EE_BLOCK_SIZE % priv->pgsize) ||
      (CONFIG_MANUAL_AT25EE_BLOCK_SIZE > priv->size))
    {
      ferr("ERROR: Configured block size is incorrect!\n");
      DEBUGASSERT(0);
      priv->blocksize = geo.pagesize;
    }
  else
    {
      priv->blocksize = CONFIG_MANUAL_AT25EE_BLOCK_SIZE;
    }

#endif

  finfo("MTD wrapper around EEPROM device: %zd bytes, %" PRIu16
        " per page, %" PRIu16 " per block, readonly %s\n",
        priv->size, (uint16_t)priv->pgsize, priv->blocksize,
        readonly ? "ON" : "OFF");

  priv->mtd.erase  = at25ee_erase;
  priv->mtd.bread  = at25ee_bread;
  priv->mtd.bwrite = at25ee_bwrite;
  priv->mtd.read   = at25ee_read;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = at25ee_write;
#endif
  priv->mtd.ioctl  = at25ee_ioctl;
  priv->mtd.name   = "at25ee";

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return (FAR struct mtd_dev_s *)priv;

cleanup:
  ee25xx_teardown(eepdev);
  kmm_free(priv);
  return NULL;
}

/****************************************************************************
 * Name: at25ee_teardown
 *
 * Description:
 *   Teardown a previously created at25ee device.
 *
 * Input Parameters:
 *   mtd - Pointer to the mtd driver instance.
 *
 ****************************************************************************/

void at25ee_teardown(FAR struct mtd_dev_s *mtd)
{
  FAR struct at25ee_dev_s *priv = (FAR struct at25ee_dev_s *)mtd;

  if (priv != NULL)
    {
      /* Destroy the underlying EEPROM driver */

      ee25xx_teardown(priv->eepdev);

      /* Free the memory */

      kmm_free(priv);
    }
}

