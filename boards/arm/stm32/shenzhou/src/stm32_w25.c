/****************************************************************************
 * boards/arm/stm32/shenzhou/src/stm32_w25.c
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

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_STM32_SPI1
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/fs/fs.h>
#  include <nuttx/fs/nxffs.h>

#  include "stm32_spi.h"
#endif

#include "shenzhou.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Can't support the W25 device if it SPI1 or W25 support is not enabled */

#define HAVE_W25  1
#if !defined(CONFIG_STM32_SPI1) || !defined(CONFIG_MTD_W25)
#  undef HAVE_W25
#endif

/* Can't support W25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_W25
#endif

/* Can't support both FAT and NXFFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_NXFFS)
#  warning "Can't support both FAT and NXFFS -- using FAT"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Initialize and register the W25 FLASH file system.
 *
 ****************************************************************************/

int stm32_w25initialize(int minor)
{
#ifdef HAVE_W25
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
#ifdef CONFIG_FS_NXFFS
  char devname[12];
#endif
  int ret;

  /* Get the SPI port */

  spi = stm32_spibus_initialize(1);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize SPI port 2\n");
      return -ENODEV;
    }

  /* Now bind the SPI interface to the W25 SPI FLASH driver */

  mtd = w25_initialize(spi);
  if (!mtd)
    {
      ferr("ERROR: Failed to bind SPI port 2 to the SST 25 FLASH driver\n");
      return -ENODEV;
    }

#ifndef CONFIG_FS_NXFFS
  /* And use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(minor, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Initialize the FTL layer\n");
      return ret;
    }
#else
  /* Initialize to provide NXFFS on the MTD interface */

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      ferr("ERROR: NXFFS initialization failed: %d\n", -ret);
      return ret;
    }

  /* Mount the file system at /mnt/w25 */

  snprintf(devname, 12, "/mnt/w25%c", 'a' + minor);
  ret = nx_mount(NULL, devname, "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
      return ret;
    }
#endif
#endif

  return OK;
}
