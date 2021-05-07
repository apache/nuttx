/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_emmcdev.c
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
#include "cxd56_emmc.h"

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
 * Name: board_emmc_initialize
 *
 * Description:
 *   Initialize the eMMC device and mount the file system.
 *
 ****************************************************************************/

int board_emmc_initialize(void)
{
  int ret;

  /* Power on the eMMC device */

  ret = board_power_control(POWER_EMMC, true);
  if (ret)
    {
      ferr("ERROR: Failed to power on eMMC. %d\n", ret);
      return -ENODEV;
    }

  /* Initialize the eMMC deivce */

  ret = cxd56_emmcinitialize();
  if (ret < 0)
    {
      ferr("ERROR: Failed to initialize eMMC. %d\n ", ret);
      return -ENODEV;
    }

  /* Mount the eMMC deivce */

  ret = nx_mount("/dev/emmc0", "/mnt/emmc", "vfat", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the eMMC. %d\n", ret);
    }

  return ret;
}
