/****************************************************************************
 * drivers/child/skeleton.c
 *
 *   Copyright (C) 2013 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/mtd.h>
#include <nuttx/kmalloc.h>
#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This type represents the state of the MTD device.  The struct mtd_dev_s
 * must appear at the beginning of the definition so that you can freely
 * cast between pointers to struct mtd_dev_s and struct mtd_partition_s.
 */

struct mtd_partition_s
{
  /* This structure must reside at the beginning so that we can simply cast
   * from struct mtd_dev_s * to struct mtd_partition_s *
   */

  struct mtd_dev_s child;       /* The "child" MTD vtable that manages the
                                 * sub-region */
  /* Other implementation specific data may follow here */

  FAR struct mtd_dev_s *parent; /* The "parent" MTD driver that manages the
                                 * entire FLASH */
  off_t firstblock;             /* Offset to the first block of the managed
                                 * sub-region */
  off_t neraseblocks;           /* The number of erase blocks in the managed
                                 * sub-region */
  off_t blocksize;              /* The size of one read/write block */
  uint16_t blkpererase;         /* Number of R/W blocks in one erase block */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* MTD driver methods */

static int part_erase(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks);
static ssize_t part_bread(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                          FAR uint8_t *buf);
static ssize_t part_bwrite(FAR struct mtd_dev_s *dev, off_t startblock, size_t nblocks,
                           FAR const uint8_t *buf);
static ssize_t part_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer);
#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t part_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR const uint8_t *buffer);
#endif
static int part_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Name: part_blockcheck
 *
 * Description:
 *   Check if the provided block offset lies within the partition
 *
 ****************************************************************************/

static bool part_blockcheck(FAR struct mtd_partition_s *priv, off_t block)
{
  off_t partsize;

  partsize = priv->neraseblocks * priv->blkpererase;
  return block < partsize;
}

/****************************************************************************
 * Name: part_bytecheck
 *
 * Description:
 *   Check if the provided byte offset lies within the partition
 *
 ****************************************************************************/

static bool part_bytecheck(FAR struct mtd_partition_s *priv, off_t byoff)
{
  off_t erasesize;
  off_t readend;

  erasesize = priv->blocksize * priv->blkpererase;
  readend   = (byoff + erasesize - 1) / erasesize;
  return readend <= priv->neraseblocks;
}

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: part_erase
 *
 * Description:
 *   Erase several blocks, each of the size previously reported.
 *
 ****************************************************************************/

static int part_erase(FAR struct mtd_dev_s *dev, off_t startblock,
                      size_t nblocks)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;
  off_t eoffset;

  DEBUGASSERT(priv);

  /* Make sure that erase would not extend past the end of the partition */

  if (!part_blockcheck(priv, startblock + nblocks - 1))
    {
      fdbg("ERROR: Read beyond the end of the partition\n");
      return -ENXIO;
    }

  /* Just add the partition offset to the requested block and let the
   * underlying MTD driver perform the erase.
   *
   * NOTE: the offset here is in units of erase blocks.
   */

  eoffset = priv->firstblock / priv->blkpererase;
  DEBUGASSERT(eoffset * priv->blkpererase == priv->firstblock);

  return priv->parent->erase(priv->parent, startblock + eoffset, nblocks);
}

/****************************************************************************
 * Name: part_bread
 *
 * Description:
 *   Read the specified number of blocks into the user provided buffer.
 *
 ****************************************************************************/

static ssize_t part_bread(FAR struct mtd_dev_s *dev, off_t startblock,
                          size_t nblocks, FAR uint8_t *buf)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;

  DEBUGASSERT(priv && (buf || nblocks == 0));

  /* Make sure that read would not extend past the end of the partition */

  if (!part_blockcheck(priv, startblock + nblocks - 1))
    {
      fdbg("ERROR: Read beyond the end of the partition\n");
      return -ENXIO;
    }

  /* Just add the partition offset to the requested block and let the
   * underlying MTD driver perform the read.
   */

  return priv->parent->bread(priv->parent, startblock + priv->firstblock,
                             nblocks, buf);
}

/****************************************************************************
 * Name: part_bwrite
 *
 * Description:
 *   Write the specified number of blocks from the user provided buffer.
 *
 ****************************************************************************/

static ssize_t part_bwrite(FAR struct mtd_dev_s *dev, off_t startblock,
                           size_t nblocks, FAR const uint8_t *buf)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;

  DEBUGASSERT(priv && (buf || nblocks == 0));

  /* Make sure that write would not extend past the end of the partition */

  if (!part_blockcheck(priv, startblock + nblocks - 1))
    {
      fdbg("ERROR: Write beyond the end of the partition\n");
      return -ENXIO;
    }

  /* Just add the partition offset to the requested block and let the
   * underlying MTD driver perform the write.
   */

  return priv->parent->bwrite(priv->parent, startblock + priv->firstblock,
                              nblocks, buf);
}

/****************************************************************************
 * Name: part_read
 *
 * Description:
 *   Read the specified number of bytes to the user provided buffer.
 *
 ****************************************************************************/

static ssize_t part_read(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                         FAR uint8_t *buffer)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;
  off_t newoffset;

  DEBUGASSERT(priv && (buffer || nbytes == 0));

  /* Does the underlying MTD device support the read method? */

  if (priv->parent->read)
    {
      /* Make sure that read would not extend past the end of the partition */

      if (!part_bytecheck(priv, offset + nbytes - 1))
        {
          fdbg("ERROR: Read beyond the end of the partition\n");
          return -ENXIO;
        }

      /* Just add the partition offset to the requested block and let the
       * underlying MTD driver perform the read.
       */

      newoffset = offset + priv->firstblock * priv->blocksize;
      return priv->parent->read(priv->parent, newoffset, nbytes, buffer);
    }

  /* The underlying MTD driver does not support the read() method */

  return -ENOSYS;
}

/****************************************************************************
 * Name: part_write
 ****************************************************************************/

#ifdef CONFIG_MTD_BYTE_WRITE
static ssize_t part_write(FAR struct mtd_dev_s *dev, off_t offset, size_t nbytes,
                          FAR const uint8_t *buffer)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;
  off_t newoffset;

  DEBUGASSERT(priv && (buffer || nbytes == 0));

  /* Does the underlying MTD device support the write method? */

  if (priv->parent->write)
    {
      /* Make sure that write would not extend past the end of the partition */

      if (!part_bytecheck(priv, offset + nbytes - 1))
        {
          fdbg("ERROR: Write beyond the end of the partition\n");
          return -ENXIO;
        }

      /* Just add the partition offset to the requested block and let the
       * underlying MTD driver perform the write.
       */

      newoffset = offset + priv->firstblock * priv->blocksize;
      return priv->parent->write(priv->parent, newoffset, nbytes, buffer);
    }

  /* The underlying MTD driver does not support the write() method */

  return -ENOSYS;
}
#endif

/****************************************************************************
 * Name: part_ioctl
 ****************************************************************************/

static int part_ioctl(FAR struct mtd_dev_s *dev, int cmd, unsigned long arg)
{
  FAR struct mtd_partition_s *priv = (FAR struct mtd_partition_s *)dev;
  int ret = -EINVAL; /* Assume good command with bad parameters */

  DEBUGASSERT(priv);

  switch (cmd)
    {
      case MTDIOC_GEOMETRY:
        {
          FAR struct mtd_geometry_s *geo = (FAR struct mtd_geometry_s *)arg;
          if (geo)
            {
              /* Populate the geometry structure with information needed to know
               * the capacity and how to access the device.
               */

              geo->blocksize    = priv->blocksize;
              geo->erasesize    = priv->blocksize * priv->blkpererase;
              geo->neraseblocks = priv->neraseblocks;
              ret               = OK;
          }
        }
        break;

      case MTDIOC_XIPBASE:
        {
          FAR void **ppv = (FAR void**)arg;
          unsigned long base;

          if (ppv)
            {
              /* Get hte XIP base of the entire FLASH */

              ret = priv->parent->ioctl(priv->parent, MTDIOC_XIPBASE,
                                        (unsigned long)((uintptr_t)&base));
              if (ret == OK)
                {
                  /* Add the offset of this partion to the XIP base and
                   * return the sum to the caller.
                   */

                  *ppv = (FAR void *)(base + priv->firstblock * priv->blocksize);
                }
            }
        }
        break;

      case MTDIOC_BULKERASE:
        {
          /* Erase the entire partition */

          ret = priv->parent->erase(priv->parent, priv->firstblock,
                                    priv->neraseblocks * priv->blkpererase);
        }
        break;

      default:
        {
          /* Pass any unhandled ioctl() calls to the underlying driver */

          ret = priv->parent->ioctl(priv->parent, cmd, arg);
        }
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mtd_partition
 *
 * Description:
 *   Give an instance of an MTD driver, create a flash partition, ie.,
 *   another MTD driver instance that only operates with a sub-region of
 *   FLASH media.  That sub-region is defined by a sector offsetset and a
 *   sector count (where the size of a sector is provided the by parent MTD
 *   driver).
 *
 *   NOTE: Since there may be a number of MTD partition drivers operating on
 *   the same, underlying FLASH driver, that FLASH driver must be capable
 *   of enforcing mutually exclusive access to the FLASH device.  Without
 *   partitions, that mutual exclusion would be provided by the file system
 *   above the FLASH driver.
 *
 ****************************************************************************/

FAR struct mtd_dev_s *mtd_partition(FAR struct mtd_dev_s *mtd, off_t firstblock,
                                    off_t nblocks)
{
  FAR struct mtd_partition_s *part;
  FAR struct mtd_geometry_s geo;
  unsigned int blkpererase;
  off_t erasestart;
  off_t eraseend;
  off_t devblocks;
  int ret;

  DEBUGASSERT(mtd);

  /* Get the geometry of the FLASH device */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      fdbg("ERROR: mtd->ioctl failed: %d\n", ret);
      return NULL;
    }

  /* Get the number of blocks per erase.  There must be an even number of
   * blocks in one erase blocks.
   */

  blkpererase = geo.erasesize / geo.blocksize;
  DEBUGASSERT(blkpererase * geo.blocksize == geo.erasesize);

  /* Adjust the offset and size if necessary so that they are multiples of
   * the erase block size (making sure that we do not go outside of the
   * requested sub-region).  NOTE that eraseend is the first erase block
   * beyond the sub-region.
   */

  erasestart = (firstblock + blkpererase - 1) / blkpererase;
  eraseend   = (firstblock + nblocks) / blkpererase;

  if (erasestart >= eraseend)
    {
      fdbg("ERROR: sub-region too small\n");
      return NULL;
    }
  
  /* Verify that the sub-region is valid for this geometry */

  devblocks = blkpererase * geo.neraseblocks;
  if (eraseend > devblocks)
    {
      fdbg("ERROR: sub-region too big\n");
      return NULL;
    }

  /* Allocate a partition device structure */

  part = (FAR struct mtd_partition_s *)kzalloc(sizeof(struct mtd_partition_s));
  if (!part)
    {
      fdbg("ERROR: Failed to allocate memory for the partition device\n");
      return NULL;      
    }

  /* Initialize the partition device structure. (unsupported methods were
   * nullified by kzalloc).
   */

  part->child.erase  = part_erase;
  part->child.bread  = part_bread;
  part->child.bwrite = part_bwrite;
  part->child.read   = mtd->read ? part_read : NULL;
  part->child.ioctl  = part_ioctl;
#ifdef CONFIG_MTD_BYTE_WRITE
  part->child.write  = mtd->write ? part_write : NULL;
#endif

  part->parent       = mtd;
  part->firstblock   = erasestart * blkpererase;
  part->neraseblocks = eraseend - erasestart;
  part->blocksize    = geo.blocksize;
  part->blkpererase  = blkpererase;

  /* Return the implementation-specific state structure as the MTD device */

  return &part->child;
}

