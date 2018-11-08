/************************************************************************************
 * drivers/mtd/mtd_rwbuffer.c
 * MTD driver that contains another MTD driver and provides read-ahead and/or write
 * buffering.
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 ************************************************************************************/

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/drivers/rwbuffer.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#if defined(CONFIG_DRVR_WRITEBUFFER) || defined(CONFIG_DRVR_READAHEAD)

/************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************/

#ifndef CONFIG_DRVR_INVALIDATE
#  error This driver requires CONFIG_DRVR_INVALIDATE
#endif

#ifndef CONFIG_DRVR_READBYTES
#  error This driver requires CONFIG_DRVR_READBYTES
#endif

#ifndef CONFIG_MTD_NWRBLOCKS
#  define CONFIG_MTD_NWRBLOCKS 4
#endif

#ifndef CONFIG_MTD_NRDBLOCKS
#  define CONFIG_MTD_NRDBLOCKS 4
#endif

/************************************************************************************
 * Private Types
 ************************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s must
 * appear at the beginning of the definition so that you can freely cast between
 * pointers to struct mtd_dev_s and struct mtd_rwbuffer_s.
 */

struct mtd_rwbuffer_s
{
  struct mtd_dev_s          mtd;  /* Our exported MTD interface */
  FAR struct mtd_dev_s     *dev;  /* Saved lower level MTD interface instance */
  struct rwbuffer_s         rwb;  /* The rwbuffer state structure */
  uint16_t                  spb;  /* Number of sectors per block */
};

/************************************************************************************
 * Private Function Prototypes
 ************************************************************************************/

/* rwbuffer callouts */

static ssize_t mtd_reload(FAR void *dev, FAR uint8_t *buffer, off_t startblock,
                          size_t nblocks);
static ssize_t mtd_flush(FAR void *dev, FAR const uint8_t *buffer, off_t startblock,
                         size_t nblocks);

/* MTD driver methods */

static int mtd_erase(FAR struct mtd_dev_s *dev, off_t block, size_t nsectors);
static ssize_t mtd_bread(FAR struct mtd_dev_s *dev, off_t block,
                         size_t nsectors, FAR uint8_t *buf);
static ssize_t mtd_bwrite(FAR struct mtd_dev_s *dev, off_t block,
                          size_t nsectors, FAR const uint8_t *buf);
static ssize_t mtd_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                        FAR uint8_t *buffer);
static int mtd_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Name: mtd_reload
 *
 * Description:
 *   Reload the read-ahead buffer
 *
 ************************************************************************************/

static ssize_t mtd_reload(FAR void *dev, FAR uint8_t *buffer, off_t startblock,
                          size_t nblocks)
{
  FAR struct mtd_rwbuffer_s *priv = (FAR struct mtd_rwbuffer_s *)dev;
  DEBUGASSERT(priv && priv->dev);

  /* This is just a pass-through to the contained MTD */

  return priv->dev->bread(priv->dev, startblock, nblocks, buffer);
}

/************************************************************************************
 * Name: mtd_flush
 *
 * Description:
 *   Flush the write buffer to hardware
 *
 ************************************************************************************/

static ssize_t mtd_flush(FAR void *dev, FAR const uint8_t *buffer, off_t startblock,
                         size_t nblocks)
{
  FAR struct mtd_rwbuffer_s *priv = (FAR struct mtd_rwbuffer_s *)dev;
  DEBUGASSERT(priv && priv->dev);

  /* This is just a pass-through to the contained MTD */

  return priv->dev->bwrite(priv->dev, startblock, nblocks, buffer);
}

/************************************************************************************
 * Name: mtd_erase
 ************************************************************************************/

static int mtd_erase(FAR struct mtd_dev_s *dev, off_t block, size_t nblocks)
{
  FAR struct mtd_rwbuffer_s *priv = (FAR struct mtd_rwbuffer_s *)dev;
  off_t sector;
  size_t nsectors;
  int ret;

  finfo("block: %08lx nsectors: %lu\n",
        (unsigned long)block, (unsigned int)nsectors);

  /* Convert to logical sectors and sector numbers */

  sector   = block * priv->spb;
  nsectors = nblocks * priv->spb;

  /* Then invalidate in cached data */

  ret = rwb_invalidate(&priv->rwb, sector, nsectors);
  if (ret < 0)
    {
      ferr("ERROR: rwb_invalidate failed: %d\n", ret);
      return ret;
    }

  /* Then let the lower level MTD driver do the real erase */

  return priv->dev->erase(priv->dev, block, nblocks);
}

/************************************************************************************
 * Name: mtd_bread
 ************************************************************************************/

static ssize_t mtd_bread(FAR struct mtd_dev_s *dev, off_t sector,
                          size_t nsectors, FAR uint8_t *buffer)
{
  FAR struct mtd_rwbuffer_s *priv = (FAR struct mtd_rwbuffer_s *)dev;

  /* Let the rwbuffer logic do it real work.  It will call out to mtd_reload if is
   * needs to read any data.
   */

  return rwb_read(&priv->rwb, sector, nsectors, buffer);
}

/************************************************************************************
 * Name: mtd_bwrite
 ************************************************************************************/

static ssize_t mtd_bwrite(FAR struct mtd_dev_s *dev, off_t block, size_t nsectors,
                            FAR const uint8_t *buffer)
{
  FAR struct mtd_rwbuffer_s *priv = (FAR struct mtd_rwbuffer_s *)dev;

  /* Let the rwbuffer logic do it real work.  It will call out to wrb_reload it is
   * needs to read any data.
   */

  return rwb_write(&priv->rwb, block, nsectors, buffer);
}

/************************************************************************************
 * Name: mtd_read
 ************************************************************************************/

static ssize_t mtd_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct mtd_rwbuffer_s *priv = (FAR struct mtd_rwbuffer_s *)dev;

  /* Let the rwbuffer logic do it real work.  It will call out to mtd_reload it is
   * needs to read any data.
   */

  return rwb_readbytes(&priv->rwb, offset, nbytes, buffer);
}

/************************************************************************************
 * Name: mtd_ioctl
 ************************************************************************************/

static int mtd_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct mtd_rwbuffer_s *priv = (FAR struct mtd_rwbuffer_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d \n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to know
               * the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an array
               * of fixed size blocks.  That is most likely not true, but the client
               * will expect the device logic to do whatever is necessary to make it
               * appear so.
               */

              geo->blocksize    = priv->rwb.blocksize;
              geo->erasesize    = priv->rwb.blocksize * priv->spb;
              geo->neraseblocks = priv->rwb.nblocks / priv->spb;
              ret               = OK;

              finfo("blocksize: %d erasesize: %d neraseblocks: %d\n",
                    geo->blocksize, geo->erasesize, geo->neraseblocks);
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = priv->dev->ioctl(priv->dev, MTDIOC_BULKERASE, 0);
          if (ret >= 0)
            {
              ferr("ERROR: Device ioctl failed: %d\n", ret);
              break;
            }

          /* Then invalidate in cached data */

         ret = rwb_invalidate(&priv->rwb, 0, priv->rwb.nblocks);
         if (ret < 0)
           {
              ferr("ERROR: rwb_invalidate failed: %d\n", ret);
           }
        }
        break;

      case MTDIOC_XIPBASE:
      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/************************************************************************************
 * Public Functions
 ************************************************************************************/

/************************************************************************************
 * Name: mtd_rwb_initialize
 *
 * Description:
 *   Create an initialized MTD device instance.  This MTD driver contains another
 *   MTD driver and converts a larger sector size to a standard 512 byte sector
 *   size.
 *
 *   MTD devices are not registered in the file system, but are created as instances
 *   that can be bound to other functions (such as a block or character driver front
 *   end).
 *
 ************************************************************************************/

FAR struct mtd_dev_s *mtd_rwb_initialize(FAR struct mtd_dev_s *mtd)
{
  FAR struct mtd_rwbuffer_s *priv;
  struct mtd_geometry_s geo;
  int ret;

  finfo("mtd: %p\n", mtd);
  DEBUGASSERT(mtd && mtd->ioctl);

  /* Get the device geometry */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      ferr("ERROR: MTDIOC_GEOMETRY ioctl failed: %d\n", ret);
      return NULL;
    }

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would have
   * to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct mtd_rwbuffer_s *)kmm_zalloc(sizeof(struct mtd_rwbuffer_s));
  if (!priv)
    {
      ferr("ERROR: Failed to allocate mtd_rwbuffer\n");
      return NULL;
    }

  /* Initialize the allocated structure. (unsupported methods/fields
   * were already nullified by kmm_zalloc).
   */

  priv->mtd.erase    = mtd_erase;  /* Our MTD erase method */
  priv->mtd.bread    = mtd_bread;  /* Our MTD bread method */
  priv->mtd.bwrite   = mtd_bwrite; /* Our MTD bwrite method */
  priv->mtd.read     = mtd_read;   /* Our MTD read method */
  priv->mtd.ioctl    = mtd_ioctl;  /* Our MTD ioctl method */
  priv->mtd.name     = "rwbuffer";

  priv->dev          = mtd;        /* The contained MTD instance */

  /* Sectors per block.  The erase block size must be an even multiple
   * of the sector size.
   */

  priv->spb          = geo.erasesize / geo.blocksize;
  DEBUGASSERT((size_t)priv->spb * geo.blocksize == geo.erasesize);

  /* Values must be provided to rwb_initialize() */
  /* Supported geometry */

  priv->rwb.blocksize = geo.blocksize;
  priv->rwb.nblocks   = geo.neraseblocks * priv->spb;

  /* Buffer setup */

#ifdef CONFIG_DRVR_WRITEBUFFER
  priv->rwb.wrmaxblocks = CONFIG_MTD_NWRBLOCKS;
#endif
#ifdef CONFIG_DRVR_READAHEAD
  priv->rwb.rhmaxblocks = CONFIG_MTD_NRDBLOCKS;
#endif

  /* Callouts */

  priv->rwb.dev       = priv;             /* Device state passed to callouts */
  priv->rwb.wrflush   = mtd_flush;        /* Callout to flush buffer */
  priv->rwb.rhreload  = mtd_reload;       /* Callout to reload buffer */

  /* Initialize read-ahead/write buffering */

  ret = rwb_initialize(&priv->rwb);
  if (ret < 0)
    {
      ferr("ERROR: rwb_initialize failed: %d\n", ret);
      kmm_free(priv);
      return NULL;
    }

  /* Return the implementation-specific state structure as the MTD device */

  return &priv->mtd;
}

#endif /* CONFIG_DRVR_WRITEBUFFER || CONFIG_DRVR_READAHEAD */
