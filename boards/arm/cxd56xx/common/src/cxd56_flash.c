/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_flash.c
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
#include <errno.h>
#include <debug.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <arch/board/board.h>
#include <nuttx/mtd/mtd.h>

#ifdef CONFIG_FS_NXFFS
#  include <nuttx/fs/nxffs.h>
#endif

#include "cxd56_sfc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SFC_DEVNO
#  define CONFIG_SFC_DEVNO 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_flash_initialize
 *
 * Description:
 *   Initialize the SPI-Flash device and mount the file system.
 *
 ****************************************************************************/

int board_flash_initialize(void)
{
  int ret;
  struct mtd_dev_s *mtd;

  mtd = cxd56_sfc_initialize();
  if (!mtd)
    {
      ferr("ERROR: Failed to initialize SFC. %d\n ", ret);
      return -ENODEV;
    }

  /* use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(CONFIG_SFC_DEVNO, mtd);
  if (ret < 0)
    {
      ferr("ERROR: Initializing the FTL layer: %d\n", ret);
      return ret;
    }

#if defined(CONFIG_FS_SMARTFS)
  /* Initialize to provide SMARTFS on the MTD interface */

  ret = smart_initialize(CONFIG_SFC_DEVNO, mtd, NULL);
  if (ret < 0)
    {
      ferr("ERROR: SmartFS initialization failed: %d\n", ret);
      return ret;
    }

  ret = nx_mount("/dev/smart0d1", "/mnt/spif", "smartfs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the SmartFS volume: %d\n", ret);
      return ret;
    }

#elif defined(CONFIG_FS_NXFFS)
  /* Initialize to provide NXFFS on the MTD interface */

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      ferr("ERROR: NXFFS initialization failed: %d\n", ret);
      return ret;
    }

  ret = nx_mount(NULL, "/mnt/spif", "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}
