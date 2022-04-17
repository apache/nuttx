/****************************************************************************
 * arch/arm/src/lc823450/lc823450_mmcl.c
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
#include <sys/ioctl.h>
#include <inttypes.h>
#include <stdio.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mtd/mtd.h>

#include "lc823450_mmcl.h"

/****************************************************************************
 * Private Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mmcl_dev_s
{
#ifdef CONFIG_FS_EVFAT
  uint32_t              channel; /* 0: eMMC, 1: SD */
#endif
  struct mtd_dev_s     *mtd;     /* Contained MTD interface */
  struct mtd_geometry_s geo;     /* Device geometry */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     mmcl_open(struct inode *inode);
static int     mmcl_close(struct inode *inode);
static ssize_t mmcl_read(struct inode *inode, unsigned char *buffer,
                         blkcnt_t start_sector, unsigned int nsectors);
static ssize_t mmcl_write(struct inode *inode,
                          const unsigned char *buffer, blkcnt_t start_sector,
                          unsigned int nsectors);
static int     mmcl_geometry(struct inode *inode,
                             struct geometry *geometry);
static int     mmcl_ioctl(struct inode *inode, int cmd,
                          unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct block_operations g_bops =
{
  mmcl_open,      /* open     */
  mmcl_close,     /* close    */
  mmcl_read,      /* read     */
  mmcl_write,     /* write    */
  mmcl_geometry,  /* geometry */
  mmcl_ioctl      /* ioctl    */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcl_open
 *
 * Description: Open the block device
 *
 ****************************************************************************/

static int mmcl_open(struct inode *inode)
{
  finfo("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: mmcl_close
 *
 * Description: close the block device
 *
 ****************************************************************************/

static int mmcl_close(struct inode *inode)
{
  finfo("Entry\n");
  return OK;
}

/****************************************************************************
 * Name: mmcl_read
 *
 * Description:  Read the specified number of sectors
 *
 ****************************************************************************/

static ssize_t mmcl_read(struct inode *inode, unsigned char *buffer,
  blkcnt_t start_sector, unsigned int nsectors)
{
  ssize_t nread;
  struct mmcl_dev_s *dev;

  finfo("sector: %" PRIuOFF " nsectors: %u\n", start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct mmcl_dev_s *)inode->i_private;

  nread = MTD_BREAD(dev->mtd, start_sector, nsectors, buffer);
  if (nread != nsectors)
    {
      finfo("Read %u blocks starting at block %" PRIuOFF " failed: %d\n",
            nsectors, start_sector, nread);
      return -EIO;
    }

  return nread;
}

/****************************************************************************
 * Name: mmcl_write
 *
 * Description: Write the specified number of sectors
 *
 ****************************************************************************/

static ssize_t mmcl_write(struct inode *inode,
                          const unsigned char *buffer, blkcnt_t start_sector,
                          unsigned int nsectors)
{
  ssize_t nwrite;
  struct mmcl_dev_s *dev;

  finfo("sector: %" PRIuOFF " nsectors: %u\n", start_sector, nsectors);

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct mmcl_dev_s *)inode->i_private;

  nwrite = MTD_BWRITE(dev->mtd, start_sector, nsectors, buffer);
  if (nwrite != nsectors)
    {
      finfo("Write %u blocks starting at block %" PRIuOFF " failed: %d\n",
            nsectors, start_sector, nwrite);
      return -EIO;
    }

  return nwrite;
}

/****************************************************************************
 * Name: mmcl_geometry
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int mmcl_geometry(struct inode *inode, struct geometry *geometry)
{
  struct mmcl_dev_s *dev;

  finfo("Entry\n");

  DEBUGASSERT(inode);
  if (geometry)
    {
      dev = (struct mmcl_dev_s *)inode->i_private;
      geometry->geo_available     = true;
      geometry->geo_mediachanged  = false;
      geometry->geo_writeenabled  = true;
      geometry->geo_nsectors      = dev->geo.neraseblocks;
      geometry->geo_sectorsize    = dev->geo.blocksize;

      finfo("available: true mediachanged: false writeenabled: %s\n",
            geometry->geo_writeenabled ? "true" : "false");

      finfo("nsectors: %" PRIuOFF " sectorsize: %" PRIi16 "\n",
            geometry->geo_nsectors, geometry->geo_sectorsize);

      return OK;
    }

  return -EINVAL;
}

/****************************************************************************
 * Name: mmcl_ioctl
 *
 * Description: Return device geometry
 *
 ****************************************************************************/

static int mmcl_ioctl(struct inode *inode, int cmd, unsigned long arg)
{
  struct mmcl_dev_s *dev ;
  int ret;
  finfo("Entry\n");

  DEBUGASSERT(inode && inode->i_private);
  dev = (struct mmcl_dev_s *)inode->i_private;

  ret = MTD_IOCTL(dev->mtd, cmd, arg);
  if (ret < 0)
    {
      finfo("ERROR: MTD ioctl(%04x) failed: %d\n", cmd, ret);
    }

  return ret;
}

/****************************************************************************
 * Name: mmcl_allocdev
 ****************************************************************************/

static struct mmcl_dev_s *mmcl_allocdev(int number,
                                        struct mtd_dev_s *mtd)
{
  struct mmcl_dev_s *dev;
  int ret;

  /* Allocate a MMCL device structure */

  dev = (struct mmcl_dev_s *)kmm_malloc(sizeof(struct mmcl_dev_s));
  if (dev)
    {
      /* Initialize the MMCL device structure */

      dev->mtd = mtd;

      /* Get the device geometry. (casting to uintptr_t first eliminates
       * complaints on some architectures where the sizeof long is different
       * from the size of a pointer).
       */

      ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY,
                      (unsigned long)((uintptr_t)&dev->geo));
      if (ret < 0)
        {
          finfo("MTD ioctl(MTDIOC_GEOMETRY) failed: %d\n", ret);
          kmm_free(dev);
          return NULL;
        }
    }

  return dev;
}

/****************************************************************************
 * Name: mmcl_freedev
 ****************************************************************************/

static void mmcl_freedev(struct mmcl_dev_s *dev)
{
  DEBUGASSERT(dev);
  kmm_free(dev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmcl_initialize
 *
 * Description:
 *   Initialize to provide a block driver wrapper around an MTD interface
 *
 * Input Parameters:
 *   minor - The minor device number.  The MTD block device will be
 *      registered as as /dev/mtdblockN where N is the minor number.
 *   mtd - The MTD device that supports the FLASH interface.
 *
 ****************************************************************************/

int mmcl_initialize(int minor, struct mtd_dev_s *mtd)
{
  struct mmcl_dev_s *dev;
  const char *devname[CONFIG_MTD_DEV_MAX] =
  {
    CONFIG_MTD_DEVPATH0,
#if CONFIG_MTD_DEV_MAX > 1
    CONFIG_MTD_DEVPATH1,
#endif
  };

  int ret = -ENOMEM;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (minor < 0 || minor > 255 || !mtd)
    {
      return -EINVAL;
    }
#endif

  DEBUGASSERT(minor < CONFIG_MTD_DEV_MAX);

  /* Allocate a MTD block device structure */

  dev = mmcl_allocdev(minor, mtd);
  DEBUGASSERT(dev);

#ifdef CONFIG_FS_EVFAT
  dev->channel = minor;
#endif

  /* Create a MTD block device name */

  /* Inode private data is a reference to the MMCL device structure */

  ret = register_blockdriver(devname[minor], &g_bops, 0, dev);
  if (ret < 0)
    {
      finfo("register_blockdriver failed: %d\n", -ret);
      kmm_free(dev);
    }

  return ret;
}

/****************************************************************************
 * Name: mmcl_uninitialize
 ****************************************************************************/

int mmcl_uninitialize(const char *devname)
{
  int ret;
  struct inode *inode;
  struct mmcl_dev_s *dev;

  DEBUGASSERT(devname);

  ret = open_blockdriver(devname, 0, &inode);
  if (ret != OK)
    {
      finfo("open_blockdriver failed: %d\n", ret);
      return ret;
    }

  dev = inode->i_private;
  DEBUGASSERT(dev);

  ret = close_blockdriver(inode);
  if (ret != OK)
    {
      finfo("close_blockdriver failed: %d\n", ret);
    }

  ret = unregister_blockdriver(devname);
  if (ret != OK)
    {
      finfo("unregister_blockdriver failed: %d\n", ret);
    }

  mmcl_freedev(dev);

  return ret;
}

/****************************************************************************
 * Name: mmcl_createpartition
 ****************************************************************************/

int mmcl_createpartition(int minor, int number, struct mtd_dev_s *mtd)
{
  struct mmcl_dev_s *dev;
  char devname[32];
  int ret = -ENOMEM;

  /* Sanity check */

#ifdef CONFIG_DEBUG
  if (minor < 0 || minor > 255 || !mtd)
    {
      return -EINVAL;
    }

  if (number < 0 || number > 255 || !mtd)
    {
      return -EINVAL;
    }
#endif

  /* Allocate a MMCL device structure */

  dev = mmcl_allocdev(minor, mtd);
  DEBUGASSERT(dev);

#ifdef CONFIG_FS_EVFAT
  dev->channel = minor;
#endif

  /* Create a MTD block device name */

  snprintf(devname, sizeof(devname), "/dev/mtdblock%dp%d", minor, number);

  /* Inode private data is a reference to the MMCL device structure */

  ret = register_blockdriver(devname, &g_bops, 0, dev);
  if (ret < 0)
    {
      finfo("register_blockdriver failed: %d\n", -ret);
      kmm_free(dev);
    }

  return ret;
}
