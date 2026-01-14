/****************************************************************************
 * boards/arm64/bcm2711/raspberrypi-4b/src/rpi4b_sdmmc.c
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

#include <debug.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mmcsd.h>

#include "bcm2711_sdio.h"

#include "rpi4b.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rpi4b_sdmmc_initialize
 *
 * Description:
 *   Bring up EMMC2 interface and mount micro-SD card as a file system.
 *
 ****************************************************************************/

int rpi4b_sdmmc_initialize(void)
{
  int err;
  struct sdio_dev_s *emmc2;

  /* Initialize EMMC2 interface */

  emmc2 = bcm2711_sdio_initialize(2);
  if (emmc2 == NULL)
    {
      syslog(LOG_ERR, "Couldn't initialize EMMC2");
      return -EIO;
    }

  /* Set up MMCSD device (uSD) on EMMC2 */

  err = mmcsd_slotinitialize(0, emmc2);
  if (err < 0)
    {
      syslog(LOG_ERR, "Couldn't bind EMMC2 interface to MMC/SD driver: %d",
             err);
      return err;
    }

  return err;
}
