/****************************************************************************
 * drivers/mtd/mtd_progmem.c
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
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <nuttx/progmem.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_MTD_PROGMEM

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct progmem_dev_s.
 */

struct progmem_dev_s
{
  /* Publicly visible representation of the interface */

  struct mtd_dev_s mtd;

  /* Fields unique to the progmem MTD driver */

  bool    initialized;      /* True: Already initialized */
  uint8_t blkshift;         /* Log2 of the flash read/write block size */
  uint8_t ersshift;         /* Log2 of the flash erase block size */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Internal helper functions */

static int32_t progmem_log2(size_t blocksize);

/* MTD driver methods */

static int     progmem_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks);
static ssize_t progmem_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, FAR uint8_t *buf);
static ssize_t progmem_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                 size_t nblocks, FAR const uint8_t *buf);
static ssize_t progmem_read(FAR struct mtd_dev_s *dev, off_t offset,
                 size_t nbytes, FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t progmem_write(FAR struct mtd_dev_s *dev, off_t offset,
                 size_t nbytes, FAR const uint8_t *buffer);
#endif
static int     progmem_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                 unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This structure holds the state of the MTD driver */

static struct progmem_dev_s g_progmem =
{
  {
    progmem_erase,
    progmem_bread,
    progmem_bwrite,
    progmem_read,
#ifdef CONFIG_MTD_BYTE_WRITE
    progmem_write,
#endif
    progmem_ioctl,
    "progmem",
  }
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: progmem_log2
 *
 * Description:
 *   Check block size is exact powers of two.
 *
 ****************************************************************************/

static int32_t progmem_log2(size_t blocksize)
{
  uint32_t log2 = 0;

  /* Search every bit in the blocksize from bit zero to bit 30 (omitting bit
   * 31 which is the sign bit on return)
   */

  for (log2 = 0; log2 < 31; log2++, blocksize >>= 1)
    {
      /* Is bit zero set? */

      if ((blocksize & 1) != 0)
        {
          /* Yes... the value should be exactly one.  We do not support
           * block sizes that are not exact powers of two.
           */

          return blocksize == 1 ? log2 : -ENOSYS;
        }
    }

  return blocksize == 0 ? -EINVAL : -E2BIG;
}

/****************************************************************************
 * Name: progmem_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported.
 *
 ****************************************************************************/

static int progmem_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                         size_t nblocks)
{
  ssize_t result;

  /* Erase the specified blocks and return status (OK or a negated errno) */

  while (nblocks > 0)
    {
      result = up_progmem_eraseblock(startblock);
      if (result < 0)
        {
          return (int)result;
        }

      /* Setup for the next pass through the loop */

      startblock++;
      nblocks--;
    }

  return OK;
}

/****************************************************************************
 * Name: progmem_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t progmem_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buffer)
{
  FAR struct progmem_dev_s *priv = (FAR struct progmem_dev_s *)dev;
#ifdef CONFIG_ARCH_HAVE_PROGMEM_READ
  ssize_t result;

  result = up_progmem_read(up_progmem_getaddress(startblock), buffer,
                           nblocks << priv->blkshift);
  return result < 0 ? result : nblocks;
#else
  FAR const uint8_t *src;

  /* Read the specified blocks into the provided user buffer and return
   * status (The positive, number of blocks actually read or a negated
   * errno).
   */

  src = (FAR const uint8_t *)up_progmem_getaddress(startblock);
  memcpy(buffer, src, nblocks << priv->blkshift);
  return nblocks;
#endif
}

/****************************************************************************
 * Name: progmem_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t progmem_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                              size_t nblocks, FAR const uint8_t *buffer)
{
  FAR struct progmem_dev_s *priv = (FAR struct progmem_dev_s *)dev;
  ssize_t result;

  /* Write the specified blocks from the provided user buffer and return
   * status (The positive, number of blocks actually written or a negated
   * errno)
   */

  result = up_progmem_write(up_progmem_getaddress(startblock), buffer,
                            nblocks << priv->blkshift);
  return result < 0 ? result : nblocks;
}

/****************************************************************************
 * Name: progmem_read
 *
 * Description:
 *   Read the specified number of bytes to the user provided buffer.
 *
 ****************************************************************************/

static ssize_t progmem_read(FAR struct mtd_dev_s *dev, off_t offset,
                            size_t nbytes, FAR uint8_t *buffer)
{
  FAR struct progmem_dev_s *priv = (FAR struct progmem_dev_s *)dev;
  off_t startblock = offset >> priv->blkshift;
#ifdef CONFIG_ARCH_HAVE_PROGMEM_READ
  ssize_t result;

  result = up_progmem_read(up_progmem_getaddress(startblock) +
           (offset & ((1 << priv->blkshift) - 1)), buffer, nbytes);
  return result < 0 ? result : nbytes;
#else
  FAR const uint8_t *src;

  /* Read the specified bytes into the provided user buffer and return
   * status (The positive, number of bytes actually read or a negated
   * errno)
   */

  src = (FAR const uint8_t *)up_progmem_getaddress(startblock) +
                             (offset & ((1 << priv->blkshift) - 1));
  memcpy(buffer, src, nbytes);
  return nbytes;
#endif
}

/****************************************************************************
 * Name: progmem_write
 *
 * Description:
 *   Some FLASH parts have the ability to write an arbitrary number of
 *   bytes to an arbitrary offset on the device.
 *
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t progmem_write(FAR struct mtd_dev_s *dev, off_t offset,
                             size_t nbytes, FAR const uint8_t *buffer)
{
  FAR struct progmem_dev_s *priv = (FAR struct progmem_dev_s *)dev;
  off_t startblock;
  ssize_t result;

  /* Write the specified blocks from the provided user buffer and return
   * status (The positive number of blocks actually written or a negated
   * errno).
   */

  startblock = offset >> priv->blkshift;
  result = up_progmem_write(up_progmem_getaddress(startblock) +
           (offset & ((1 << priv->blkshift) - 1)), buffer, nbytes);
  return result < 0 ? result : nbytes;
}
#endif

/****************************************************************************
 * Name: progmem_ioctl
 ****************************************************************************/

static int progmem_ioctl(FAR struct mtd_dev_s *dev, int cmd,
                         unsigned long arg)
{
  FAR struct progmem_dev_s *priv = (FAR struct progmem_dev_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information needed to
               * know the capacity and how to access the device.
               *
               * NOTE: that the device is treated as though it where just an
               * array of fixed size blocks.  That is most likely not true,
               * but the client will expect the device logic to do whatever
               * is necessary to make it appear so.
               */

              geo->blocksize    = (1 << priv->blkshift);     /* Size of one read/write block */
              geo->erasesize    = (1 << priv->ersshift);     /* Size of one erase block */
              geo->neraseblocks = up_progmem_neraseblocks(); /* Number of erase blocks */
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
              info->numsectors  = up_progmem_neraseblocks() <<
                                  (priv->ersshift - priv->blkshift);
              info->sectorsize  = 1 << priv->blkshift;
              info->startsector = 0;
              info->parent[0]   = '\0';
              ret               = OK;
            }
        }
        break;

      case BIOC_XIPBASE:
        {
          FAR void **ppv = (FAR void**)arg;

          if (ppv)
            {
              /* Return (void*) base address of FLASH memory. */

              *ppv = (FAR void *)up_progmem_getaddress(0);
              ret  = OK;
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          size_t nblocks = up_progmem_neraseblocks();

          /* Erase the entire device */

          ret = progmem_erase(dev, 0, nblocks);
        }
        break;

      case MTDIOC_ERASESTATE:
        {
          FAR uint8_t *result = (FAR uint8_t *)arg;
          *result = up_progmem_erasestate();

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
 * Name: progmem_initialize
 *
 * Description:
 *   Create and initialize an MTD device instance that can be used to access
 *   on-chip program memory.
 *
 *   MTD devices are not registered in the file system, but are created as
 *   instances that can be bound to other functions (such as a block or
 *   character driver front end).
 *
 ****************************************************************************/

FAR struct mtd_dev_s *progmem_initialize(void)
{
  FAR struct progmem_dev_s *priv = (FAR struct progmem_dev_s *)&g_progmem;
  int32_t blkshift;
  int32_t ersshift;

  /* Perform initialization if necessary */

  if (!g_progmem.initialized)
    {
      /* Get the size of one block.  Here we assume that the block size is
       * uniform and that the size of block0 is the same as the size of any
       * other block.
       */

      size_t blocksize = up_progmem_pagesize(0);

      /* Calculate Log2 of the flash read/write block size */

      blkshift = progmem_log2(blocksize);
      if (blkshift < 0)
        {
          return NULL;
        }

      /* Calculate Log2 of the flash erase block size */

      blocksize = up_progmem_erasesize(0);

      ersshift = progmem_log2(blocksize);
      if (ersshift < 0)
        {
          return NULL;
        }

      /* Save the configuration data */

      g_progmem.blkshift    = blkshift;
      g_progmem.ersshift    = ersshift;
      g_progmem.initialized = true;
    }

  /* Return the implementation-specific state structure as the MTD device */

  return (FAR struct mtd_dev_s *)priv;
}

#endif /* CONFIG_MTD_PROGMEM */
