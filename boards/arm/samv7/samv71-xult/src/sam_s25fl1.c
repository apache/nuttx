/****************************************************************************
 * boards/arm/samv7/samv71-xult/src/sam_s25fl1.c
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

#include <stdio.h>
#include <stdbool.h>
#include <syslog.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/drivers/drivers.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/spi/qspi.h>
#include <nuttx/net/usrsock.h>

#include <nuttx/fs/ioctl.h>
#include <nuttx/fs/smart.h>
#include <nuttx/fs/nxffs.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "sam_qspi.h"
#include "samv71-xult.h"
#include "board_progmem.h" /* For struct mtd_partition_s definition */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define QSPI_PORTNO 0

typedef enum
  {
    MTD_TYPE_BCH,
    MTD_TYPE_SMARTFS,
    MTD_TYPE_NXFFS,
    MTD_TYPE_MTD,
  } mtd_types;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mtd_partition_s g_mtd_partition_table[] =
{
  {
    .size    = 0x4000, /* 16 KiB */
    .devpath = "/nxffs",
    .type    = MTD_TYPE_NXFFS,
  },
  {
    .size    = 0x8000, /* 32 KiB */
    .devpath = "/dev/bch",
    .type    = MTD_TYPE_BCH,
  },
  {
    .size    = 0x8000, /* 32 KiB */
    .devpath = "/dev/mtd",
    .type    = MTD_TYPE_MTD,
  },
};

static const size_t g_mtd_partition_table_size =
    nitems(g_mtd_partition_table);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: issmartfs
 *
 * Description:
 *   Check a SMART (Sector Mapped Allocation for Really Tiny) Flash file
 *   system image on the specified block device (must be a SMART device).
 *
 * Inputs:
 *   pathname - the full path to a registered block driver
 *
 * Return:
 *   Zero (OK) on success; -1 (ERROR) on failure with errno set:
 *
 *   EINVAL - NULL block driver string
 *   ENOENT - 'pathname' does not refer to anything in the filesystem.
 *   ENOTBLK - 'pathname' does not refer to a block driver
 *   EFTYPE - the block driver hasn't been formatted yet
 *
 ****************************************************************************/

#ifdef CONFIG_FS_SMARTFS
static int issmartfs(FAR const char *pathname)
{
  struct smart_format_s fmt;
  int fd;
  int ret;

  /* Find the inode of the block driver identified by 'source' */

  fd = open(pathname, O_RDONLY);
  if (fd < 0)
    {
      return ERROR;
    }

  /* Get the format information so we know the block have been formatted */

  ret = ioctl(fd, BIOC_GETFORMAT, (unsigned long)&fmt);
  if (ret < 0)
    {
      close(fd);
      return ret;
    }

  if (!(fmt.flags & SMART_FMT_ISFORMATTED))
    {
      errno = EFTYPE;
      ret = ERROR;
    }

  close(fd);
  return ret;
}

/****************************************************************************
 * Name: mksmartfs
 *
 * Description:
 *   Make a SMART Flash file system image on the specified block device
 *
 * Inputs:
 *   pathname   - the full path to a registered block driver
 *   sectorsize - the size of logical sectors on the device from 256-16384.
 *                Setting this to zero will cause the device to be formatted
 *                using the default CONFIG_MTD_SMART_SECTOR_SIZE value.
 *   nrootdirs  - Number of root directory entries to create.
 *
 * Return:
 *   Zero (OK) on success; -1 (ERROR) on failure with errno set.
 *
 * Assumptions:
 *   - The caller must assure that the block driver is not mounted and not in
 *     use when this function is called.  The result of formatting a mounted
 *     device is indeterminate (but likely not good).
 *
 ****************************************************************************/

static int mksmartfs(FAR const char *pathname, uint16_t sectorsize)
{
  struct smart_format_s fmt;
  struct smart_read_write_s request;
  uint8_t type;
  int fd;
  int ret;

  /* Find the inode of the block driver identified by 'pathname' */

  fd = open(pathname, O_RDWR);
  if (fd < 0)
    {
      ret = -ENOENT;
      goto errout;
    }

  /* Perform a low-level SMART format */

  ret = ioctl(fd, BIOC_LLFORMAT, sectorsize << 16);
  if (ret != OK)
    {
      ret = -errno;
      goto errout_with_driver;
    }

  /* Get the format information so we know how big the sectors are */

  ret = ioctl(fd, BIOC_GETFORMAT, (unsigned long) &fmt);
  if (ret != OK)
    {
      ret = -errno;
      goto errout_with_driver;
    }

  /* Now write the filesystem to media. */

  ret = ioctl(fd, BIOC_ALLOCSECT, SMARTFS_ROOT_DIR_SECTOR);
  if (ret != SMARTFS_ROOT_DIR_SECTOR)
    {
      ret = -EIO;
      goto errout_with_driver;
    }

  /* Mark this block as a directory entry */

  type = SMARTFS_SECTOR_TYPE_DIR;
  request.offset = 0;
  request.count = 1;
  request.buffer = &type;
  request.logsector = SMARTFS_ROOT_DIR_SECTOR;

  /* Issue a write to the sector, single byte */

  ret = ioctl(fd, BIOC_WRITESECT, (unsigned long) &request);
  if (ret != OK)
    {
      ret = -EIO;
      goto errout_with_driver;
    }

errout_with_driver:

  /* Close the driver */

  close(fd);

errout:

  /* Return any reported errors */

  if (ret < 0)
    {
      errno = -ret;
      return ERROR;
    }

  return OK;
}
#endif /* CONFIG_FS_SMARTFS */

/****************************************************************************
 * Name: sam_s25fl1_alloc_mtdpart
 *
 * Description:
 *   Allocate an MTD partition from FLASH.
 *
 * Input Parameters:
 *   mtd        - The MTD device to be partitioned
 *   mtd_offset - MTD Partition offset from the base address in FLASH.
 *   mtd_size   - Size for the MTD partition.
 *
 * Returned Value:
 *   MTD partition data pointer on success, NULL on failure.
 *
 ****************************************************************************/

static struct mtd_dev_s
*sam_s25fl1_alloc_mtdpart(struct mtd_dev_s *mtd,
                             struct mtd_geometry_s *geo,
                             struct mtd_partition_s
                             *part)
{
  ASSERT((part->offset + part->size) <= geo->neraseblocks * geo->erasesize);
  ASSERT((part->offset % geo->erasesize) == 0);
  ASSERT((part->size % geo->erasesize) == 0);

  size_t startblock = part->offset / geo->blocksize;
  size_t blocks = part->size / geo->blocksize;

  return mtd_partition(mtd, startblock, blocks);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_s25fl1_init
 *
 * Description:
 *   Initialize s25fl1.
 *
 ****************************************************************************/

int sam_s25fl1_init(void)
{
  int ret = OK;
#ifdef CONFIG_FS_SMARTFS
  char  partref[4];
  char  smartfs_path[30];
#endif

  struct qspi_dev_s *qspi = sam_qspi_initialize(QSPI_PORTNO);
  if (!qspi)
    {
      syslog(LOG_ERR, "ERROR: sam_qspi_initialize failed\n");
      return ERROR;
    }

  /* Use the QSPI device instance to initialize the s25fl1 device.  */

  struct mtd_dev_s *mtd = s25fl1_initialize(qspi, true);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: s25fl1_initialize failed\n");
      return ERROR;
    }

  /* Get geometry of s25fl1 */

  struct mtd_geometry_s geometry;
  ret = MTD_IOCTL(mtd, MTDIOC_GEOMETRY, (unsigned long)&geometry);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: s25fl1 ioctl failed: %s\n",
             strerror(errno));
      return ret;
    }

  size_t i;
  size_t offset = 0;
  for (i = 0; i < g_mtd_partition_table_size; i++)
    {
      struct mtd_partition_s *part = &g_mtd_partition_table[i];
      part->offset = offset;
      if (part->size == 0)
        {
          part->size = (geometry.neraseblocks * geometry.erasesize) - offset;
        }

      part->mtd = sam_s25fl1_alloc_mtdpart(mtd, &geometry, part);
      if (!part->mtd)
        {
          syslog(LOG_ERR, "ERROR: sam_s25fl1_alloc_mtdpart failed\n");
          return ERROR;
        }

      /* Use the FTL layer to wrap the MTD driver as a block driver */

      if (part->type == MTD_TYPE_BCH)
        {
          ret = ftl_initialize(S25FL1_MTD_MINOR + i, part->mtd);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to initialize FTL layer: %d\n",
                    ret);
              return ret;
            }

          /* Now create a character device on the block device */

#if defined(CONFIG_BCH)
              char blockdev[18];
              snprintf(blockdev, sizeof(blockdev), "/dev/mtdblock%d",
                      S25FL1_MTD_MINOR + i);
              ret = bchdev_register(blockdev, part->devpath, false);
              if (ret < 0)
                {
                  syslog(LOG_ERR, "ERROR: bchdev_register %s failed: %d\n",
                        part->devpath, ret);
                  return ret;
                }
#endif /* defined(CONFIG_BCH) */
        }
      else if (part->type == MTD_TYPE_SMARTFS)
        {
#if defined (CONFIG_FS_SMARTFS)
          sprintf(partref, "p%d", i);
          ret = smart_initialize(0, part->mtd, partref);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: smart_initialize %s failed: %d\n",
                    part->devpath, ret);
              return ret;
            }

          sprintf(smartfs_path, "/dev/smart0%s", partref);
          if (issmartfs(smartfs_path) != OK)
            {
              mksmartfs(smartfs_path, 0);
            }

          ret = nx_mount(smartfs_path, part->devpath, "smartfs", 0,
                          NULL);
          if (ret < 0)
            {
              /* Mount failed. */

              syslog(LOG_ERR, "ERROR: mount %s failed: %d\n", smartfs_path,
                        ret);
              return ret;
            }
#endif
        }
      else if (part->type == MTD_TYPE_NXFFS)
        {
#if defined (HAVE_S25FL1_NXFFS)
          /* Initialize to provide NXFFS on the S25FL1 MTD interface */

          ret = nxffs_initialize(mtd);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: NXFFS initialization failed: %d\n",
                     ret);
            }

          /* Mount the file system at /mnt/s25fl1 */

          ret = nx_mount(NULL, part->devpath, "nxffs", 0, NULL);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to mount NXFFS volume: %d\n",
                     ret);
              return ret;
            }
#endif
        }
      else if (part->type == MTD_TYPE_MTD)
        {
#if defined (HAVE_S25FL1_MTDDEV)
          ret = mtd_partition_register(part->mtd, part->devpath, false);
          if (ret < 0)
            {
              syslog(LOG_ERR, "failed to register %d %d\n", ret, errno);
            }
#endif
        }
      else
        {
          syslog(LOG_ERR, "ERROR: Incorrect device type %d for %s.\n",
                 part->type, part->devpath);
        }

      offset += part->size;
    }

  return ret;
}
