/****************************************************************************
 * boards/sparc/bm3803/xx3803/src/bm3803_am29lv.c
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

#include <sys/mount.h>

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>
#include <nuttx/fs/fs.h>
#ifdef CONFIG_MTD_AM29LV
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/fs/nxffs.h>
#  include <nuttx/mtd/paradata.h>
#endif

#include "xx3803.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Can't support the W25 device if it SPI1 or W25 support is not enabled */

#define HAVE_AM29LV  1
#if !defined(CONFIG_MTD_AM29LV)
#  undef HAVE_AM29LV
#endif

/* Can't support W25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_AM29LV
#endif

/* Can't support both FAT and SMARTFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_SMARTFS)
#  warning "Can't support both FAT and SMARTFS -- using FAT"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: bm3803_w25initialize
 *
 * Description:
 *   Initialize and register the W25 FLASH file system.
 *
 ****************************************************************************/

int bm3803_am29lv_initialize(int minor)
{
  int ret;
#ifdef HAVE_AM29LV
  struct mtd_dev_s *mtd;
#if defined(CONFIG_MTD_PARTITION_NAMES)
  const char *partname = CONFIG_XX3803_FLASH_PART_NAMES;
#endif

  /* Now bind the SPI interface to the W25 SPI FLASH driver */

  mtd = am29lv_initialize();
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize AM29LV FLASH driver\n");
      return -ENODEV;
    }
#if defined(CONFIG_XX3803_FLASH_PARA)

      /* Register the MTD as the System Parameter device */

      ret = mtdpara_register(mtd);
      if (ret < 0)
        {
          _err("ERROR: Failed to register the /dev/para: %d\n", errno);
          return ret;
        }
#endif

#ifdef CONFIG_MTD_PARTITION

      mtd = mtd_partition(mtd, 48, 79);
      if (!mtd)
        {
          _err("ERROR: mtd_partition failed\n");
          return ret;
        }
#endif

#ifdef CONFIG_FS_LITTLEFS

  /* Register the MTD driver so that it can be accessed from the
   * VFS.
   */

  ret = register_mtddriver("/dev/flash", mtd, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register MTD driver: %d\n", ret);
    }

  /* mtd_dev->ioctl(mtd_dev, MTDIOC_BULKERASE, 0); */

  /* Mount the LittleFS file system */

  ret = mount("/dev/flash", "/mnt/lfs", "littlefs", 0,
                 "autoformat");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount LittleFS at /mnt/lfs: %d\n",
             ret);
    }
#endif

#endif /* HAVE_AM29LV */

  /* Create a RAM MTD device if configured */

#if defined(CONFIG_RAMMTD) && defined(CONFIG_XX3803_RAMMTD)
  uint8_t *start = (uint8_t *)kmm_malloc(CONFIG_XX3803_RAMMTD_SIZE * 1024);
  mtd = rammtd_initialize(start, CONFIG_XX3803_RAMMTD_SIZE * 1024);
  mtd->ioctl(mtd, MTDIOC_BULKERASE, 0);
#endif /* CONFIG_RAMMTD && CONFIG_XX3803_RAMMTD */

  return OK;
}
