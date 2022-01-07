/****************************************************************************
 * drivers/mtd/sector512.c
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

/* MTD driver that contains another MTD driver and converts a larger sector
 * size to a standard 512 byte sector size.
 */

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

#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration */

#ifndef CONFIG_MTD_SECT512_ERASED_STATE
#  define CONFIG_MTD_SECT512_ERASED_STATE 0xff
#endif

/* 512-byte sector constants */

#define SECTOR_512              512
#define SHIFT_512               9
#define MASK_512                511

/* Cache flags */

#define SST25_CACHE_VALID       (1 << 0)    /* 1=Cache has valid data */
#define SST25_CACHE_DIRTY       (1 << 1)    /* 1=Cache is dirty */
#define SST25_CACHE_ERASED      (1 << 2)    /* 1=Backing FLASH is erased */

#define IS_VALID(p)             ((((p)->flags) & SST25_CACHE_VALID) != 0)
#define IS_DIRTY(p)             ((((p)->flags) & SST25_CACHE_DIRTY) != 0)
#define IS_ERASED(p)            ((((p)->flags) & SST25_CACHE_DIRTY) != 0)

#define SET_VALID(p)            do { (p)->flags |= SST25_CACHE_VALID; } while (0)
#define SET_DIRTY(p)            do { (p)->flags |= SST25_CACHE_DIRTY; } while (0)
#define SET_ERASED(p)           do { (p)->flags |= SST25_CACHE_DIRTY; } while (0)

#define CLR_VALID(p)            do { (p)->flags &= ~SST25_CACHE_VALID; } while (0)
#define CLR_DIRTY(p)            do { (p)->flags &= ~SST25_CACHE_DIRTY; } while (0)
#define CLR_ERASED(p)           do { (p)->flags &= ~SST25_CACHE_DIRTY; } while (0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device. The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct s512_dev_s.
 */

struct s512_dev_s
{
  struct mtd_dev_s      mtd;          /* MTD interface */
  FAR struct mtd_dev_s *dev;          /* Saved lower level MTD interface instance */
  uint32_t              eblocksize;   /* Size of one erase block */
  size_t                neblocks;     /* Number of erase blocks */
  size_t                sectperblock; /* Number of read/write sectors per erase block */
  uint16_t              stdperblock;  /* Number of 512 byte sectors in one erase block */
  uint8_t               flags;        /* Buffered sector flags */
  uint32_t              eblockno;     /* Erase sector number in the cache */
  FAR uint8_t          *eblock;       /* Allocated erase block */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static FAR uint8_t *s512_cacheread(struct s512_dev_s *priv, off_t sector);
#ifndef CONFIG_MTD_SECT512_READONLY
static void s512_cacheflush(struct s512_dev_s *priv);
#endif

/* MTD driver methods */

static int s512_erase(FAR struct mtd_dev_s *dev,
                      off_t sector512,
                      size_t nsectors);
static ssize_t s512_bread(FAR struct mtd_dev_s *dev,
                          off_t sector512,
                          size_t nsectors,
                          FAR uint8_t *buf);
static ssize_t s512_bwrite(FAR struct mtd_dev_s *dev,
                           off_t sector512,
                           size_t nsectors,
                           FAR const uint8_t *buf);
static ssize_t s512_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer);
static int s512_ioctl(FAR struct mtd_dev_s *dev,
                      int cmd,
                      unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s512_cacheread
 ****************************************************************************/

static FAR uint8_t *s512_cacheread(struct s512_dev_s *priv, off_t sector512)
{
  off_t eblockno;
  off_t sector;
  ssize_t result;
  int index;

  /* Get the erase block containing this sector */

  eblockno = sector512 / priv->stdperblock;
  finfo("sector512: %lu eblockno: %lu\n",
        (unsigned long)sector512, (unsigned long)eblockno);

  /* Check if the requested erase block is already in the cache */

  if (!IS_VALID(priv) || eblockno != priv->eblockno)
    {
      /* No.. Flush any dirty erase block currently in the cache */

      s512_cacheflush(priv);

      /* Read the erase block into the cache */

      sector = eblockno * priv->sectperblock;
      result = priv->dev->bread(priv->dev, sector, priv->sectperblock,
                                priv->eblock);
      if (result < 0)
        {
          ferr("ERROR: bread(%lu, %lu) returned %ld\n",
               (unsigned long)sector, (unsigned long)priv->eblocksize,
               (long)result);

          return NULL;
        }

      /* Mark the sector as cached */

      priv->eblockno = eblockno;

      SET_VALID(priv);          /* The data in the cache is valid */
      CLR_DIRTY(priv);          /* It should match the FLASH contents */
      CLR_ERASED(priv);         /* The underlying FLASH has not been erased */
    }

  /* Get the index to the 512 sector in the erase block that holds the
   * argument
   */

  index = sector512 % priv->stdperblock;

  /* Return the address in the cache that holds this sector */

  return &priv->eblock[index << SHIFT_512];
}

/****************************************************************************
 * Name: s512_cacheflush
 ****************************************************************************/

#if !defined(CONFIG_MTD_SECT512_READONLY)
static void s512_cacheflush(struct s512_dev_s *priv)
{
  off_t sector;
  ssize_t result;

  /* If the cached is dirty (meaning that it no longer matches the old FLASH
   * contents) or was erased (with the cache containing the correct FLASH
   * contents), then write the cached erase block to FLASH.
   */

  if (IS_DIRTY(priv) || IS_ERASED(priv))
    {
      /* Write entire erase block to FLASH */

      sector = priv->eblockno * priv->sectperblock;
      result = priv->dev->bwrite(priv->dev, sector, priv->sectperblock,
                                 priv->eblock);
      if (result < 0)
        {
          ferr("ERROR: bwrite(%lu, %lu) returned %ld\n",
               (unsigned long)sector, (unsigned long)priv->eblocksize,
               (long)result);

          return;
        }

      /* The cache is no long dirty and the FLASH is no longer erased */

      CLR_DIRTY(priv);
      CLR_ERASED(priv);
    }
}
#endif

/****************************************************************************
 * Name: s512_erase
 ****************************************************************************/

static int s512_erase(FAR struct mtd_dev_s *dev,
                      off_t sector512,
                      size_t nsectors)
{
#ifdef CONFIG_MTD_SECT512_READONLY
  return -EACESS
#else
  FAR struct s512_dev_s *priv = (FAR struct s512_dev_s *)dev;
  FAR uint8_t *dest;
  size_t sectorsleft = nsectors;
  size_t eblockno;
  int ret;

  finfo("sector512: %08jx nsectors: %zu\n", (intmax_t)sector512, nsectors);

  while (sectorsleft-- > 0)
    {
      /* Erase each sector. First, make sure that the erase block containing
       * the 512 byte sector is in the cache.
       */

      dest = s512_cacheread(priv, sector512);
      if (!dest)
        {
          ferr("ERROR: s512_cacheread(%lu) failed\n",
               (unsigned long)sector512);
          DEBUGPANIC();
          return -EIO;
        }

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicator will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the block.
       */

      if (!IS_ERASED(priv))
        {
          eblockno  = sector512 / priv->stdperblock;
          finfo("sector512: %lu eblockno: %lu\n",
                (unsigned long)sector512, (unsigned long)eblockno);

          ret = priv->dev->erase(priv->dev, eblockno, 1);
          if (ret < 0)
            {
              ferr("ERROR: Failed to erase block %lu: %d\n",
                   (unsigned long)eblockno, ret);
              return ret;
            }

          SET_ERASED(priv);
        }

      /* Put the cached sector data into the erase state and mark the cache
       * as dirty (but don't update the FLASH yet.  The caller will do that
       * at a more optimal time).
       */

      memset(dest, CONFIG_MTD_SECT512_ERASED_STATE, SECTOR_512);
      SET_DIRTY(priv);
      sector512++;
    }

  /* Flush the last erase block left in the cache */

  s512_cacheflush(priv);

  return (int)nsectors;
#endif
}

/****************************************************************************
 * Name: s512_bread
 ****************************************************************************/

static ssize_t s512_bread(FAR struct mtd_dev_s *dev, off_t sector512,
                          size_t nsectors, FAR uint8_t *buffer)
{
  FAR struct s512_dev_s *priv = (FAR struct s512_dev_s *)dev;
  FAR uint8_t *src;
  ssize_t remaining;
  ssize_t result = nsectors;

  finfo("sector512: %08lx nsectors: %d\n", (long)sector512, (int)nsectors);

  /* Read each 512 byte sector from the block via the erase block cache */

  for (remaining = nsectors; remaining; remaining--)
    {
      /* Make sure that the next sector is in the erase block cache */

      src = s512_cacheread(priv, sector512);
      if (!src)
        {
          ferr("ERROR: s512_cacheread(%lu) failed\n",
               (unsigned long)sector512);
          DEBUGPANIC();

          result = (ssize_t)nsectors - remaining;
          if (result <= 0)
            {
              result = -EIO;
            }

          break;
        }

      /* Copy the sector data from the erase block cache into the user
       * buffer
       */

      memcpy(buffer, src, SECTOR_512);

      buffer += SECTOR_512;
      sector512++;
    }

  return result;
}

/****************************************************************************
 * Name: s512_bwrite
 ****************************************************************************/

static ssize_t s512_bwrite(FAR struct mtd_dev_s *dev, off_t sector512,
                           size_t nsectors,
                           FAR const uint8_t *buffer)
{
#ifdef CONFIG_MTD_SECT512_READONLY
  return -EACCESS;
#else
  FAR struct s512_dev_s *priv = (FAR struct s512_dev_s *)dev;
  ssize_t remaining;
  ssize_t result;
  off_t eblockno;

  finfo("sector512: %08lx nsectors: %d\n", (long)sector512, (int)nsectors);

  FAR uint8_t *dest;

  for (remaining = nsectors; remaining > 0; remaining--)
    {
      /* First, make sure that the erase block containing 512 byte sector is
       * in memory.
       */

      dest = s512_cacheread(priv, sector512);
      if (!dest)
        {
          result = (ssize_t)nsectors - remaining;
          if (result <= 0)
            {
              result = -EIO;
            }

          return result;
        }

      /* Erase the block containing this sector if it is not already erased.
       * The erased indicated will be cleared when the data from the erase
       * sector is read into the cache and set here when we erase the sector.
       */

      if (!IS_ERASED(priv))
        {
          eblockno  = sector512 / priv->stdperblock;
          finfo("sector512: %lu eblockno: %lu\n",
                (unsigned long)sector512, (unsigned long)eblockno);

          result = priv->dev->erase(priv->dev, eblockno, 1);
          if (result < 0)
            {
              ferr("ERROR: Failed to erase block %lu: %ld\n",
                   (unsigned long)eblockno, (long)result);
              return result;
            }

          SET_ERASED(priv);
        }

      /* Copy the new sector data into cached erase block */

      memcpy(dest, buffer, SECTOR_512);
      SET_DIRTY(priv);

      /* Set up for the next 512 byte sector */

      buffer += SECTOR_512;
      sector512++;
    }

  /* Flush the last erase block left in the cache */

  s512_cacheflush(priv);
  return nsectors;
#endif
}

/****************************************************************************
 * Name: s512_read
 ****************************************************************************/

static ssize_t s512_read(FAR struct mtd_dev_s *dev,
                         off_t offset,
                         size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct s512_dev_s *priv = (FAR struct s512_dev_s *)dev;
  FAR uint8_t *src;
  ssize_t remaining;
  ssize_t xfrsize;
  off_t sectoffset;
  off_t sector;

  finfo("offset: %08lx nbytes: %lu\n",
        (unsigned long)offset, (unsigned long)nbytes);

  /* Convert the offset into 512 byte sector address and a byte offset */

  sectoffset = offset & MASK_512;
  sector     = offset >> SHIFT_512;

  for (remaining = nbytes; remaining > 0; remaining -= xfrsize)
    {
      /* Read the erase block into the cache and get the address of the
       * beginning of the 512 byte block in the cached erase block.
       */

      src = s512_cacheread(priv, sector);
      if (!src)
        {
          int result;

          ferr("ERROR: s512_cacheread(%lu) failed\n", (unsigned long)sector);
          DEBUGPANIC();

          result = (ssize_t)nbytes - remaining;
          if (result <= 0)
            {
              result = -EIO;
            }

          return result;
        }

      /* Then copy the requested bytes from the cached erase block */

      xfrsize = remaining;
      if (sectoffset + xfrsize > SECTOR_512)
        {
          xfrsize = SECTOR_512 - sectoffset;
        }

      memcpy(buffer, src + sectoffset, xfrsize);
      buffer += xfrsize;
    }

  finfo("return nbytes: %d\n", (int)nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: s512_ioctl
 ****************************************************************************/

static int s512_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct s512_dev_s *priv = (FAR struct s512_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  finfo("cmd: %d\n", cmd);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)
                                           ((uintptr_t)arg);
          if (geo)
            {
              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               *
               * NOTE:
               * that the device is treated as though it where just an
               * array of fixed size blocks. That is most likely not true,
               * but the client will expect the device logic to do whatever
               * is necessary to make it appear so.
               */

              geo->blocksize    = SECTOR_512;
              geo->erasesize    = SECTOR_512;
              geo->neraseblocks = priv->neblocks * priv->stdperblock;
              ret               = OK;

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
              info->numsectors  = priv->neblocks * priv->stdperblock;
              info->sectorsize  = SECTOR_512;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = priv->dev->ioctl(priv->dev, MTDIOC_BULKERASE, 0);
          if (ret >= 0)
            {
              priv->flags    = 0;      /* Buffered sector flags */
              priv->eblockno = 0;      /* Erase sector number in the cache */
              priv->eblock   = NULL;   /* Allocated erase block */
            }
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = CONFIG_MTD_SECT512_ERASED_STATE;

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  finfo("return %d\n", ret);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: s512_initialize
 *
 * Description:
 *   Create an initialized MTD device instance. This MTD driver contains
 *   another MTD driver and converts a larger sector size to a standard 512
 *   byte sector size.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *s512_initialize(FAR struct mtd_dev_s *mtd)
{
  FAR struct s512_dev_s *priv;
  FAR struct mtd_geometry_s geo;
  int ret;

  finfo("mtd: %p\n", mtd);

  /* Get the device geometry */

  DEBUGASSERT(mtd && mtd->ioctl);
  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));

  /* We expect that the block size will be >512 and an even multiple of 512 */

  if (ret < 0 || geo.erasesize <= SECTOR_512 ||
     (geo.erasesize & ~MASK_512) != geo.erasesize)
    {
      ferr(
        "ERROR: MTDIOC_GEOMETRY ioctl returned %d, eraseize=%" PRId32 "\n",
           ret, geo.erasesize);
      DEBUGPANIC();
      return NULL;
    }

  /* Allocate a state structure (we allocate the structure instead of using
   * a fixed, static allocation so that we can handle multiple FLASH devices.
   * The current implementation would handle only one FLASH part per SPI
   * device (only because of the SPIDEV_FLASH(0) definition) and so would
   * have to be extended to handle multiple FLASH parts on the same SPI bus.
   */

  priv = (FAR struct s512_dev_s *)kmm_zalloc(sizeof(struct s512_dev_s));
  if (priv)
    {
      /* Initialize the allocated structure. (unsupported methods/fields
       * were already nullified by kmm_zalloc).
       */

      priv->mtd.erase    = s512_erase;
      priv->mtd.bread    = s512_bread;
      priv->mtd.bwrite   = s512_bwrite;
      priv->mtd.read     = s512_read;
      priv->mtd.ioctl    = s512_ioctl;
      priv->mtd.name     = "sector512";

      priv->dev          = mtd;
      priv->eblocksize   = geo.erasesize;
      priv->neblocks     = geo.neraseblocks;
      priv->sectperblock = geo.erasesize / geo.blocksize;
      priv->stdperblock  = geo.erasesize >> 9;

      /* Allocate a buffer for the erase block cache */

      priv->eblock = (FAR uint8_t *)kmm_malloc(priv->eblocksize);
      if (!priv->eblock)
        {
          /* Allocation failed! Discard all of that work we just did and
           * return NULL
           */

          ferr("ERROR: Allocation failed\n");
          kmm_free(priv);
          priv = NULL;
        }
    }

  /* Return the implementation-specific state structure as the MTD device */

  finfo("Return %p\n", priv);
  return &priv->mtd;
}
