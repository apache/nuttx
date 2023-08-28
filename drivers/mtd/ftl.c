/****************************************************************************
 * drivers/mtd/ftl.c
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

#include <sys/param.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/drivers/rwbuffer.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Check if read/write buffer support is needed */

#if defined(CONFIG_FTL_READAHEAD) || defined(CONFIG_FTL_WRITEBUFFER)
#  define FTL_HAVE_RWBUFFER 1
#endif

/* The maximum length of the device name paths is the maximum length of a
 * name plus 5 for the the length of "/dev/" and a NUL terminator.
 */

#define DEV_NAME_MAX    (NAME_MAX + 5)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct ftl_struct_s
{
  FAR struct mtd_dev_s *mtd;      /* Contained MTD interface */
  struct mtd_geometry_s geo;      /* Device geometry */
#ifdef FTL_HAVE_RWBUFFER
  struct rwbuffer_s     rwb;      /* Read-ahead/write buffer support */
#endif
  uint16_t              blkper;   /* R/W blocks per erase block */
  uint16_t              refs;     /* Number of references */
  bool                  unlinked; /* The driver has been unlinked */
  FAR uint8_t          *eblock;   /* One, in-memory erase block */

  /* The nand block map between logic block and physical block */

  FAR off_t            *lptable;
  off_t                 lpcount;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     ftl_open(FAR struct inode *inode);
static int     ftl_close(FAR struct inode *inode);
static ssize_t ftl_reload(FAR void *priv, FAR uint8_t *buffer,
                 off_t startblock, size_t nblocks);
static ssize_t ftl_read(FAR struct inode *inode, FAR unsigned char *buffer,
                 blkcnt_t start_sector, unsigned int nsectors);
static ssize_t ftl_flush(FAR void *priv, FAR const uint8_t *buffer,
                 off_t startblock, size_t nblocks);
static ssize_t ftl_write(FAR struct inode *inode,
                 FAR const unsigned char *buffer, blkcnt_t start_sector,
                 unsigned int nsectors);
static int     ftl_geometry(FAR struct inode *inode,
                 FAR struct geometry *geometry);
static int     ftl_ioctl(FAR struct inode *inode, int cmd,
                 unsigned long arg);
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int     ftl_unlink(FAR struct inode *inode);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  ftl_open,     /* open     */
  ftl_close,    /* close    */
  ftl_read,     /* read     */
  ftl_write,    /* write    */
  ftl_geometry, /* geometry */
  ftl_ioctl     /* ioctl    */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , ftl_unlink  /* unlink   */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftl_init_map
 *
 * Description: Allocate logical block and physical block mapping table
 *              space, and scan the entire nand flash device to establish
 *              the mapping relationship between logical block and physical
 *              good block.
 *
 ****************************************************************************/

static int ftl_init_map(FAR struct ftl_struct_s *dev)
{
  int j = 0;
  int i;

  if (dev->lptable == NULL)
    {
      dev->lptable = kmm_malloc(dev->geo.neraseblocks * sizeof(off_t));
      if (dev->lptable == NULL)
        {
          return -ENOMEM;
        }
    }

  for (i = 0; i < dev->geo.neraseblocks; i++)
    {
      if (!MTD_ISBAD(dev->mtd, i))
        {
          dev->lptable[j++] = i;
        }
    }

  dev->lpcount = j;
  return 0;
}

/****************************************************************************
 * Name: ftl_update_map
 *
 * Description: Update the lptable from the specified location, remap the
 *              relationship between logical blocks and physical good blocks.
 *
 ****************************************************************************/

static void ftl_update_map(FAR struct ftl_struct_s *dev, off_t start)
{
  DEBUGASSERT(start < dev->lpcount);
  memmove(&dev->lptable[start], &dev->lptable[start + 1],
          (--dev->lpcount - start) * sizeof(dev->lptable[0]));
}

/****************************************************************************
 * Name: ftl_get_cblock
 *
 * Description: Get the number of consecutive eraseblocks from lptable.
 *
 ****************************************************************************/

static size_t ftl_get_cblock(FAR struct ftl_struct_s *dev, off_t start,
                             size_t count)
{
  off_t i;

  count = MIN(count, dev->lpcount - start);
  for (i = start; i < start + count - 1; i++)
    {
      if (dev->lptable[i + 1] - dev->lptable[i] != 1)
        {
          return i - start + 1;
        }
    }

  return count;
}

/****************************************************************************
 * Name: ftl_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int ftl_open(FAR struct inode *inode)
{
  FAR struct ftl_struct_s *dev;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  dev->refs++;
  return OK;
}

/****************************************************************************
 * Name: ftl_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int ftl_close(FAR struct inode *inode)
{
  FAR struct ftl_struct_s *dev;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

#ifdef CONFIG_FTL_WRITEBUFFER
  rwb_flush(&dev->rwb);
#endif

  if (--dev->refs == 0 && dev->unlinked)
    {
#ifdef FTL_HAVE_RWBUFFER
      rwb_uninitialize(&dev->rwb);
#endif
      if (dev->eblock)
        {
          kmm_free(dev->eblock);
        }

      kmm_free(dev);
    }

  return OK;
}

/****************************************************************************
 * Name: ftl_mtd_bread
 *
 * Description:
 *   Read the specified number of sectors. If mtd device is nor flash, it
 *   can be read once time. If mtd device is nand flash, it can be read one
 *   block every time and need to skip bad block until the specified number
 *   of sectors finish.
 *
 ****************************************************************************/

static ssize_t ftl_mtd_bread(FAR struct ftl_struct_s *dev, off_t startblock,
                             size_t nblocks, FAR uint8_t *buffer)
{
  off_t mask = dev->blkper - 1;
  size_t nread = nblocks;
  ssize_t ret = OK;

  if (dev->lptable == NULL)
    {
      ret = MTD_BREAD(dev->mtd, startblock, nblocks, buffer);
      if (ret != nblocks)
        {
          ferr("ERROR: Read %zu blocks starting at block %" PRIdOFF
               " failed: %zd\n", nblocks, startblock, ret);
        }

      return ret;
    }

  while (nblocks > 0)
    {
      off_t startphysicalblock;
      off_t starteraseblock;
      size_t count;

      starteraseblock = startblock / dev->blkper;
      if (starteraseblock >= dev->lpcount)
        {
          ret = -ENOSPC;
          break;
        }

      count = ftl_get_cblock(dev, starteraseblock,
                             (nblocks + mask) / dev->blkper);
      count = MIN(count * dev->blkper, nblocks);
      startphysicalblock = dev->lptable[starteraseblock] *
                           dev->blkper + (startblock & mask);
      ret = MTD_BREAD(dev->mtd, startphysicalblock, count, buffer);
      if (ret == count || ret == -EUCLEAN)
        {
          nblocks -= count;
          startblock += count;
          buffer += count * dev->geo.blocksize;
        }
      else
        {
          ftl_update_map(dev, starteraseblock);
          break;
        }
    }

  return nblocks != nread ? nread - nblocks : ret;
}

/****************************************************************************
 * Name: ftl_mtd_bwrite
 *
 * Description:
 *   Write the specified eraseblocks. If mtd device is nor flash, it
 *   can be written once time. If mtd device is nand flash, it can be write
 *   one block every time and need to skip bad block until writing success.
 *
 ****************************************************************************/

static ssize_t ftl_mtd_bwrite(FAR struct ftl_struct_s *dev, off_t startblock,
                              FAR const uint8_t *buffer)
{
  off_t starteraseblock;
  ssize_t ret;

  if (dev->lptable == NULL)
    {
      ret = MTD_BWRITE(dev->mtd, startblock, dev->blkper, buffer);
      if (ret != dev->blkper)
        {
          ferr("ERROR: Write block %" PRIdOFF " failed: %zd\n",
               startblock, ret);
        }

      return ret;
    }

  starteraseblock = startblock / dev->blkper;
  while (1)
    {
      if (starteraseblock >= dev->lpcount)
        {
          return -ENOSPC;
        }

      ret = MTD_BWRITE(dev->mtd, dev->lptable[starteraseblock] * dev->blkper,
                       dev->blkper, buffer);
      if (ret == dev->blkper)
        {
          return ret;
        }

      MTD_MARKBAD(dev->mtd, dev->lptable[starteraseblock]);
      ftl_update_map(dev, starteraseblock);
    }
}

/****************************************************************************
 * Name: ftl_mtd_erase
 *
 * Description:
 *   Erase the specified number of sectors. If mtd device is nor flash, it
 *   can be erased once time. If mtd device is nand flash, it can be erased
 *   one block every time and need to skip bad block until the specified
 *   number of sectors finish.
 *
 ****************************************************************************/

static ssize_t ftl_mtd_erase(FAR struct ftl_struct_s *dev, off_t startblock)
{
  ssize_t ret;

  if (dev->lptable == NULL)
    {
      ret = MTD_ERASE(dev->mtd, startblock, 1);
      if (ret < 0)
        {
          ferr("ERROR: Erase block %" PRIdOFF " failed: %zd\n",
               startblock, ret);
        }

      return ret;
    }

  while (1)
    {
      if (startblock >= dev->lpcount)
        {
          return -ENOSPC;
        }

      ret = MTD_ERASE(dev->mtd, dev->lptable[startblock], 1);
      if (ret == 1)
        {
          return ret;
        }

      MTD_MARKBAD(dev->mtd, dev->lptable[startblock]);
      ftl_update_map(dev, startblock);
    }
}

/****************************************************************************
 * Name: ftl_reload
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t ftl_reload(FAR void *priv, FAR uint8_t *buffer,
                          off_t startblock, size_t nblocks)
{
  struct ftl_struct_s *dev = (struct ftl_struct_s *)priv;

  /* Read the full erase block into the buffer */

  return ftl_mtd_bread(dev, startblock, nblocks, buffer);
}

/****************************************************************************
 * Name: ftl_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t ftl_read(FAR struct inode *inode, unsigned char *buffer,
                        blkcnt_t start_sector, unsigned int nsectors)
{
  FAR struct ftl_struct_s *dev;

  finfo("sector: %" PRIuOFF " nsectors: %u\n", start_sector, nsectors);

  DEBUGASSERT(inode->i_private);

  dev = inode->i_private;
#ifdef FTL_HAVE_RWBUFFER
  return rwb_read(&dev->rwb, start_sector, nsectors, buffer);
#else
  return ftl_reload(dev, buffer, start_sector, nsectors);
#endif
}

/****************************************************************************
 * Name: ftl_flush
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

static int ftl_alloc_eblock(FAR struct ftl_struct_s *dev)
{
  if (dev->eblock == NULL)
    {
      /* Allocate one, in-memory erase block buffer */

      dev->eblock = kmm_malloc(dev->geo.erasesize);
    }

  return dev->eblock != NULL ? OK : -ENOMEM;
}

static ssize_t ftl_flush(FAR void *priv, FAR const uint8_t *buffer,
                         off_t startblock, size_t nblocks)
{
  struct ftl_struct_s *dev = (struct ftl_struct_s *)priv;
  off_t  alignedblock;
  off_t  mask;
  off_t  rwblock;
  off_t  eraseblock;
  off_t  offset;
  size_t remaining;
  size_t nxfrd;
  int    nbytes;
  int    ret;

  /* Get the aligned block.  Here is is assumed: (1) The number of R/W blocks
   * per erase block is a power of 2, and (2) the erase begins with that same
   * alignment.
   */

  mask         = dev->blkper - 1;
  alignedblock = (startblock + mask) & ~mask;

  /* Handle partial erase blocks before the first unaligned block */

  remaining = nblocks;
  if (alignedblock > startblock)
    {
      /* Check if the write is shorter than to the end of the erase block */

      bool short_write = (remaining < (alignedblock - startblock));

      ret = ftl_alloc_eblock(dev);
      if (ret < 0)
        {
          ferr("ERROR: Failed to allocate an erase block buffer\n");
          return ret;
        }

      /* Read the full erase block into the buffer */

      rwblock = startblock & ~mask;
      nxfrd   = ftl_mtd_bread(dev, rwblock, dev->blkper, dev->eblock);
      if (nxfrd != dev->blkper)
        {
          return -EIO;
        }

      /* Then erase the erase block */

      eraseblock = rwblock / dev->blkper;
      ret        = ftl_mtd_erase(dev, eraseblock);
      if (ret < 0)
        {
          return ret;
        }

      /* Copy the user data at the end of the buffered erase block */

      offset = (startblock & mask) * dev->geo.blocksize;

      if (short_write)
        {
          nbytes = remaining * dev->geo.blocksize;
        }
      else
        {
          nbytes = dev->geo.erasesize - offset;
        }

      finfo("Copy %d bytes into erase block=%" PRIdOFF
            " at offset=%" PRIdOFF "\n", nbytes, eraseblock, offset);

      memcpy(dev->eblock + offset, buffer, nbytes);

      /* And write the erase block back to flash */

      nxfrd = ftl_mtd_bwrite(dev, rwblock, dev->eblock);
      if (nxfrd != dev->blkper)
        {
          return -EIO;
        }

      /* Then update for amount written */

      if (short_write)
        {
          remaining = 0;
        }
      else
        {
          remaining -= dev->blkper - (startblock & mask);
        }

      buffer += nbytes;
    }

  /* How handle full erase pages in the middle */

  while (remaining >= dev->blkper)
    {
      /* Erase the erase block */

      eraseblock = alignedblock / dev->blkper;
      ret        = ftl_mtd_erase(dev, eraseblock);
      if (ret < 0)
        {
          return ret;
        }

      /* Write a full erase back to flash */

      finfo("Write %" PRId32 " bytes into erase block=%" PRIdOFF
            " at offset=0\n", dev->geo.erasesize, alignedblock);

      nxfrd = ftl_mtd_bwrite(dev, alignedblock, buffer);
      if (nxfrd != dev->blkper)
        {
          return -EIO;
        }

      /* Then update for amount written */

      alignedblock += dev->blkper;
      remaining    -= dev->blkper;
      buffer       += dev->geo.erasesize;
    }

  /* Finally, handle any partial blocks after the last full erase block */

  if (remaining > 0)
    {
      ret = ftl_alloc_eblock(dev);
      if (ret < 0)
        {
          ferr("ERROR: Failed to allocate an erase block buffer\n");
          return ret;
        }

      /* Read the full erase block into the buffer */

      nxfrd = ftl_mtd_bread(dev, alignedblock, dev->blkper, dev->eblock);
      if (nxfrd != dev->blkper)
        {
          return -EIO;
        }

      /* Then erase the erase block */

      eraseblock = alignedblock / dev->blkper;
      ret        = ftl_mtd_erase(dev, eraseblock);
      if (ret < 0)
        {
          return ret;
        }

      /* Copy the user data at the beginning the buffered erase block */

      nbytes = remaining * dev->geo.blocksize;
      finfo("Copy %d bytes into erase block=%" PRIdOFF " at offset=0\n",
             nbytes, alignedblock);
      memcpy(dev->eblock, buffer, nbytes);

      /* And write the erase back to flash */

      nxfrd = ftl_mtd_bwrite(dev, alignedblock, dev->eblock);
      if (nxfrd != dev->blkper)
        {
          return -EIO;
        }
    }

  return nblocks;
}

/****************************************************************************
 * Name: ftl_write
 *
 * Description: Write (or buffer) the specified number of sectors
 *
 ****************************************************************************/

static ssize_t ftl_write(FAR struct inode *inode,
                         FAR const unsigned char *buffer,
                         blkcnt_t start_sector, unsigned int nsectors)
{
  struct ftl_struct_s *dev;

  finfo("sector: %" PRIuOFF " nsectors: %u\n", start_sector, nsectors);

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;
#ifdef FTL_HAVE_RWBUFFER
  return rwb_write(&dev->rwb, start_sector, nsectors, buffer);
#else
  return ftl_flush(dev, buffer, start_sector, nsectors);
#endif
}

/****************************************************************************
 * Name: ftl_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int ftl_geometry(FAR struct inode *inode,
                        FAR struct geometry *geometry)
{
  FAR struct ftl_struct_s *dev;

  finfo("Entry\n");

  if (geometry)
    {
      dev = inode->i_private;
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
      geometry->geo_writeenabled  = true;
      geometry->geo_nsectors      = dev->geo.neraseblocks * dev->blkper;
      geometry->geo_sectorsize    = dev->geo.blocksize;

      strlcpy(geometry->geo_model, dev->geo.model,
              sizeof(geometry->geo_model));

      finfo("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");
      finfo("nsectors: %" PRIuOFF " sectorsize: %u\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: ftl_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int ftl_ioctl(FAR struct inode *inode, int cmd, unsigned long arg)
{
  FAR struct ftl_struct_s *dev;
  int ret;

  finfo("Entry\n");
  DEBUGASSERT(inode->i_private);

  dev = inode->i_private;

  if (cmd == BIOC_FLUSH)
    {
#ifdef CONFIG_FTL_WRITEBUFFER
      rwb_flush(&dev->rwb);
#endif
    }

  /* No other block driver ioctl commands are not recognized by this
   * driver.  Other possible MTD driver ioctl commands are passed through
   * to the MTD driver (unchanged).
   */

  ret = MTD_IOCTL(dev->mtd, cmd, arg);
  if (ret < 0 && ret != -ENOTTY)
    {
      ferr("ERROR: MTD ioctl(%04x) failed: %d\n", cmd, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: ftl_unlink
 *
 * Description: Unlink the device
 *
 ****************************************************************************/

#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
static int ftl_unlink(FAR struct inode *inode)
{
  FAR struct ftl_struct_s *dev;

  DEBUGASSERT(inode->i_private);
  dev = inode->i_private;

  dev->unlinked = true;
  if (dev->refs == 0)
    {
#ifdef FTL_HAVE_RWBUFFER
      rwb_uninitialize(&dev->rwb);
#endif
      if (dev->eblock)
        {
          kmm_free(dev->eblock);
        }

      kmm_free(dev);
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ftl_initialize_by_path
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   path - The block device path.
 *   mtd  - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int ftl_initialize_by_path(FAR const char *path, FAR struct mtd_dev_s *mtd)
{
  struct ftl_struct_s *dev;
  int ret = -ENOMEM;

  /* Sanity check */

  if (path == NULL || mtd == NULL)
    {
      return -EINVAL;
    }

  finfo("path=\"%s\"\n", path);

  /* Allocate a FTL device structure */

  dev = kmm_zalloc(sizeof(struct ftl_struct_s));
  if (dev)
    {
      /* Initialize the FTL device structure */

      dev->mtd = mtd;

      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY,
                      (unsigned long)((uintptr_t)&dev->geo));
      if (ret < 0)
        {
          ferr("ERROR: MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          kmm_free(dev);
          return ret;
        }

      /* Get the number of R/W blocks per erase block */

      dev->blkper = dev->geo.erasesize / dev->geo.blocksize;
      DEBUGASSERT(dev->blkper * dev->geo.blocksize == dev->geo.erasesize);

      /* Configure read-ahead/write buffering */

#ifdef FTL_HAVE_RWBUFFER
      dev->rwb.blocksize     = dev->geo.blocksize;
      dev->rwb.nblocks       = dev->geo.neraseblocks * dev->blkper;
      dev->rwb.dev           = (FAR void *)dev;
      dev->rwb.wrflush       = ftl_flush;
      dev->rwb.rhreload      = ftl_reload;

#if defined(CONFIG_FTL_WRITEBUFFER)
      dev->rwb.wrmaxblocks   = dev->blkper;
      dev->rwb.wralignblocks = dev->blkper;
#endif

#ifdef CONFIG_FTL_READAHEAD
      dev->rwb.rhmaxblocks   = dev->blkper;
#endif

      ret = rwb_initialize(&dev->rwb);
      if (ret < 0)
        {
          ferr("ERROR: rwb_initialize failed: %d\n", ret);
          kmm_free(dev);
          return ret;
        }
#endif

      if (MTD_ISBAD(dev->mtd, 0) != -ENOSYS)
        {
          ret = ftl_init_map(dev);
          if (ret < 0)
            {
              goto out;
            }
        }

      /* Inode private data is a reference to the FTL device structure */

      ret = register_blockdriver(path, &g_bops, 0, dev);
      if (ret < 0)
        {
          ferr("ERROR: register_blockdriver failed: %d\n", -ret);
          kmm_free(dev->lptable);
out:
#ifdef FTL_HAVE_RWBUFFER
          rwb_uninitialize(&dev->rwb);
#endif
          kmm_free(dev);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: ftl_initialize
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   minor - The minor device number.  The MTD block device will be
 *           registered as as /dev/mtdblockN where N is the minor number.
 *   mtd   - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int ftl_initialize(int minor, FAR struct mtd_dev_s *mtd)
{
  char path[DEV_NAME_MAX];

#ifdef CONFIG_DEBUG_FEATURES
  /* Sanity check */

  if (minor < 0 || minor > 255)
    {
      return -EINVAL;
    }
#endif

  /* Do the real work by ftl_initialize_by_path */

  snprintf(path, DEV_NAME_MAX, "/dev/mtdblock%d", minor);
  return ftl_initialize_by_path(path, mtd);
}
