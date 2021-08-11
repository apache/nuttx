/****************************************************************************
 * drivers/mtd/nullmtd.c
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

/* Configuration ************************************************************/

#ifndef CONFIG_NULLMTD_BLOCKSIZE
#  define CONFIG_NULLMTD_BLOCKSIZE 512
#endif

#ifndef CONFIG_NULLMTD_ERASESIZE
#  define CONFIG_NULLMTD_ERASESIZE 4096
#endif

#ifndef CONFIG_NULLMTD_ERASESTATE
#  define CONFIG_NULLMTD_ERASESTATE 0xff
#endif

#if CONFIG_NULLMTD_ERASESTATE != 0xff && CONFIG_NULLMTD_ERASESTATE != 0x00
#  error "Unsupported value for CONFIG_NULLMTD_ERASESTATE"
#endif

#if CONFIG_NULLMTD_BLOCKSIZE > CONFIG_NULLMTD_ERASESIZE
#  error "Must have CONFIG_NULLMTD_BLOCKSIZE <= CONFIG_NULLMTD_ERASESIZE"
#endif

#undef  NULLMTD_BLKPER
#define NULLMTD_BLKPER (CONFIG_NULLMTD_ERASESIZE/CONFIG_NULLMTD_BLOCKSIZE)

#if NULLMTD_BLKPER*CONFIG_NULLMTD_BLOCKSIZE != CONFIG_NULLMTD_ERASESIZE
#  error "CONFIG_NULLMTD_ERASESIZE must be an even multiple of CONFIG_NULLMTD_BLOCKSIZE"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct null_dev_s.
 */

struct null_dev_s
{
  struct mtd_dev_s mtd;        /* MTD device */
  size_t           nblocks;    /* Number of erase blocks */
  size_t           erasesize;  /* Offset from start of file */
  size_t           blocksize;  /* Offset from start of file */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#define nullmtd_read(dest, len) memset(dest, CONFIG_NULLMTD_ERASESTATE, len)

/* MTD driver methods */

static int     nullmtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks);
static ssize_t nullmtd_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buf);
static ssize_t nullmtd_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buf);
static ssize_t nullmtd_byteread(FAR struct mtd_dev_s *dev, off_t offset,
                                size_t nbytes, FAR uint8_t *buf);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t nullmtd_bytewrite(FAR struct mtd_dev_s *dev, off_t offset,
                                 size_t nbytes, FAR const uint8_t *buf);
#endif
static int     nullmtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                             unsigned long arg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nullmtd_erase
 ****************************************************************************/

static int nullmtd_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks)
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;

  DEBUGASSERT(dev);

  /* Don't let the erase exceed the configured size of the device */

  if (startblock >= priv->nblocks)
    {
      return 0;
    }

  return OK;
}

/****************************************************************************
 * Name: nullmtd_bread
 ****************************************************************************/

static ssize_t nullmtd_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buf)
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;
  off_t maxblock;
  size_t nbytes;

  DEBUGASSERT(dev && buf);

  /* Don't let the read exceed the configured size of the device */

  maxblock = priv->nblocks * (priv->erasesize / priv->blocksize);
  if (startblock >= maxblock)
    {
      return 0;
    }

  if (startblock + nblocks > maxblock)
    {
      nblocks = maxblock - startblock;
    }

  /* Get the size corresponding to the number of blocks.
   */

  nbytes = nblocks * priv->blocksize;

  /* Then read the data from the file */

  nullmtd_read(buf, nbytes);
  return nblocks;
}

/****************************************************************************
 * Name: nullmtd_bwrite
 ****************************************************************************/

static ssize_t nullmtd_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;
  off_t maxblock;

  DEBUGASSERT(dev && buf);

  /* Don't let the write exceed the configured size of the device */

  maxblock = priv->nblocks * (priv->erasesize / priv->blocksize);
  if (startblock >= maxblock)
    {
      return 0;
    }

  if (startblock + nblocks > maxblock)
    {
      nblocks = maxblock - startblock;
    }

  return nblocks;
}

/****************************************************************************
 * Name: nullmtd_byteread
 ****************************************************************************/

static ssize_t nullmtd_byteread(FAR struct mtd_dev_s *dev, off_t offset,
                                size_t nbytes, FAR uint8_t *buf)
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;
  off_t maxoffset;

  DEBUGASSERT(dev && buf);

  /* Don't let the read exceed the configured size of the device */

  maxoffset = priv->nblocks * priv->erasesize;
  if (offset >= maxoffset)
    {
      return 0;
    }

  if (offset + nbytes > maxoffset)
    {
      nbytes = maxoffset - offset;
    }

  nullmtd_read(buf, nbytes);
  return nbytes;
}

/****************************************************************************
 * Name: nullmtd_bytewrite
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t nullmtd_bytewrite(FAR struct mtd_dev_s *dev, off_t offset,
                                 size_t nbytes, FAR const uint8_t *buf)
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;
  off_t maxoffset;

  DEBUGASSERT(dev && buf);

  /* Don't let the write exceed the configured size of the device */

  maxoffset = priv->nblocks * priv->erasesize;
  if (offset >= maxoffset)
    {
      return 0;
    }

  if (offset + nbytes > maxoffset)
    {
      nbytes = maxoffset - offset;
    }

  return nbytes;
}
#endif

/****************************************************************************
 * Name: nullmtd_ioctl
 ****************************************************************************/

static int nullmtd_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct null_dev_s *priv = (FAR struct null_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo =
            (FAR struct mtd_geometry_s *)((uintptr_t)arg);

          if (geo)
            {
              memset(geo, 0, sizeof(*geo));

              /* Populate the geometry structure with information need to
               * know the capacity and how to access the device.
               */

              geo->blocksize    = priv->blocksize;
              geo->erasesize    = priv->erasesize;
              geo->neraseblocks = priv->nblocks;
              ret               = OK;
            }
        }
        break;

      case BIOC_PARTINFO:
        {
          FAR struct partition_info_s *info =
            (FAR struct partition_info_s *)arg;
          if (info != NULL)
            {
              info->numsectors  = priv->nblocks *
                                  priv->erasesize / priv->blocksize;
              info->sectorsize  = priv->blocksize;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire device */

          ret = nullmtd_erase(dev, 0, priv->nblocks);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = CONFIG_NULLMTD_ERASESTATE;

          ret = OK;
        }
        break;

      default:
        ret = -ENOTTY; /* Bad command */
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nullmtd_initialize
 *
 * Description:
 *   Create and initialize a MTD null device instance.
 *
 * Input Parameters:
 *   mtdlen - total size of MTD device
 *   sectsize
 *
 ****************************************************************************/

FAR struct mtd_dev_s *nullmtd_initialize(size_t mtdlen, int16_t sectsize,
                                         int32_t erasesize)
{
  FAR struct null_dev_s *priv;
  size_t nblocks;

  /* Create an instance of the RAM MTD device state structure */

  priv = (FAR struct null_dev_s *)kmm_zalloc(sizeof(struct null_dev_s));
  if (!priv)
    {
      ferr("ERROR: Failed to allocate the RAM MTD state structure\n");
      return NULL;
    }

  /* Set the block size based on the provided sectsize parameter */

  if (sectsize <= 0)
    {
      priv->blocksize = CONFIG_NULLMTD_BLOCKSIZE;
    }
  else
    {
      priv->blocksize = sectsize;
    }

  /* Set the erase size based on the provided erasesize parameter */

  if (erasesize <= 0)
    {
      priv->erasesize = CONFIG_NULLMTD_ERASESIZE;
    }
  else
    {
      priv->erasesize = erasesize;
    }

  /* Force the size to be an even number of the erase block size */

  nblocks = mtdlen / priv->erasesize;
  if (nblocks < 1)
    {
      ferr("ERROR: Need to provide at least one full erase block\n");
      kmm_free(priv);
      return NULL;
    }

  /* Perform initialization as necessary. (unsupported methods were
   * nullified by kmm_zalloc).
   */

  priv->mtd.erase  = nullmtd_erase;
  priv->mtd.bread  = nullmtd_bread;
  priv->mtd.bwrite = nullmtd_bwrite;
  priv->mtd.read   = nullmtd_byteread;
#ifdef CONFIG_MTD_BYTE_WRITE
  priv->mtd.write  = nullmtd_bytewrite;
#endif
  priv->mtd.ioctl  = nullmtd_ioctl;
  priv->mtd.name   = "nullmtd";
  priv->nblocks    = nblocks;

  return &priv->mtd;
}
