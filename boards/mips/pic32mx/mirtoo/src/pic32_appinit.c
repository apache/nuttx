/****************************************************************************
 * boards/mips/pic32mx/mirtoo/src/pic32_appinit.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_PIC32MX_SPI2
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/fs/nxffs.h>
#endif

#include "pic32mx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Can't support the SST25 device if it SPI2/SST25 support is not enabled */

#define HAVE_SST25  1
#if !defined(CONFIG_PIC32MX_SPI2) || !defined(CONFIG_MTD_SST25)
#  undef HAVE_SST25
#endif

/* Can't support SST25 features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_SST25
#endif

/* Use minor device number 0 is not is provided */

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/* Can't support both FAT and NXFFS */

#if defined(CONFIG_FS_FAT) && defined(CONFIG_FS_NXFFS)
#  warning "Can't support both FAT and NXFFS -- using FAT"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef HAVE_SST25
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
  int ret;

  /* Get the SPI port */

  spi = pic32mx_spibus_initialize(2);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize SPI port 2\n");
      return -ENODEV;
    }

  /* Now bind the SPI interface to the SST 25 SPI FLASH driver */

  mtd = sst25_initialize(spi);
  if (!mtd)
    {
      ferr("ERROR: Failed to bind SPI port 2 to the SST 25 FLASH driver\n");
      return -ENODEV;
    }

#ifndef CONFIG_FS_NXFFS
  /* And use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(CONFIG_NSH_MMCSDMINOR, mtd);
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

  /* Mount the file system at /mnt/sst25 */

  ret = nx_mount(NULL, "/mnt/sst25", "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
      return ret;
    }
#endif
#endif

  return OK;
}
