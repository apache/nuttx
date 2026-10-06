/****************************************************************************
 * boards/arm/imxrt/imxrt1180-evk/src/imxrt_bringup.c
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

#include <sys/mount.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#include "imxrt_gpio.h"
#include "imxrt_usdhc.h"
#include "imxrt1180-evk.h"
#include <arch/board/board.h>  /* Must always be included last */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_IMXRT_USDHC
static int imxrt_sdmmc_initialize(void)
{
  struct sdio_dev_s *sdmmc;
  int ret;

  /* Power the slot and select 3.3 V I/O signalling. */

  imxrt_config_gpio(GPIO_SD_PWREN);
  imxrt_config_gpio(GPIO_SD1_VSELECT);

  sdmmc = imxrt_usdhc_initialize(0);
  if (sdmmc == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize USDHC1\n");
      return -ENODEV;
    }

  ret = mmcsd_slotinitialize(0, sdmmc);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind USDHC1 to the MMC/SD "
             "driver: %d\n", ret);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board resources.  For the minimal MIMXRT1180-EVK NSH
 *   configuration this only mounts /proc when procfs is enabled.
 *
 ****************************************************************************/

int imxrt_bringup(void)
{
  int ret = OK;

#if defined(CONFIG_USBDEV_DMAMEMORY) || defined(CONFIG_FAT_DMAMEMORY)
  ret = imxrt_dma_alloc_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize DMA allocator: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs: %d\n", ret);
    }
#endif

#if defined(CONFIG_INPUT_BUTTONS) && defined(CONFIG_INPUT_BUTTONS_LOWER)
  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_IMXRT_USDHC
  ret = imxrt_sdmmc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: imxrt_sdmmc_initialize() failed: %d\n", ret);
    }
#endif

  return ret;
}
