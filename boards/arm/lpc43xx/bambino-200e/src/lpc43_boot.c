/****************************************************************************
 * boards/arm/lpc43xx/bambino-200e/src/lpc43_boot.c
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

#include <debug.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#ifdef CONFIG_LPC43_SPIFI
#  include <nuttx/mtd/mtd.h>
#  include "lpc43_spifi.h"

#  ifdef CONFIG_SPFI_NXFFS
#    include <nuttx/fs/fs.h>
#    include <nuttx/fs/nxffs.h>
#  endif
#endif

#ifdef CONFIG_LPC43_SDMMC
#  include <nuttx/sdio.h>
#  include <nuttx/mmcsd.h>
#  include "lpc43_sdmmc.h"
#endif

#include "arm_internal.h"
#include "bambino-200e.h"
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SPIFI_DEVNO
#  define CONFIG_SPIFI_DEVNO 0
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum devid_e
{
  DEVID0 = 0,
  DEVID1
};

enum ssp_channel_e
{
  SSP0 = 0,
  SSP1
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_spifi_initialize
 *
 * Description:
 *   Make the SPIFI (or part of it) into a block driver that can hold a
 *   file system.
 *
 ****************************************************************************/

#ifdef CONFIG_LPC43_SPIFI
static int nsh_spifi_initialize(void)
{
  struct mtd_dev_s *mtd;
  int ret;

  /* Initialize the SPIFI interface and create the MTD driver instance */

  mtd = lpc43_spifi_initialize();
  if (!mtd)
    {
      ferr("ERROR: lpc43_spifi_initialize failed\n");
      return -ENODEV;
    }

#ifndef CONFIG_SPFI_NXFFS
  /* Register the MTD driver */

  char path[32];
  snprintf(path, sizeof(path), "/dev/mtdblock%d", CONFIG_SPIFI_DEVNO);
  ret = register_mtddriver(path, mtd, 0755, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to register the MTD driver %s, ret %d\n",
           path, ret);
      return ret;
    }
#else
  /* Initialize to provide NXFFS on the MTD interface */

  ret = nxffs_initialize(mtd);
  if (ret < 0)
    {
      ferr("ERROR: NXFFS initialization failed: %d\n", ret);
      return ret;
    }

  /* Mount the file system at /mnt/spifi */

  ret = nx_mount(NULL, "/mnt/spifi", "nxffs", 0, NULL);
  if (ret < 0)
    {
      ferr("ERROR: Failed to mount the NXFFS volume: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_boardinitialize
 *
 * Description:
 *   All LPC43xx architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void lpc43_boardinitialize(void)
{
  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

  /* Configure SSP chip selects if 1) Any SSP channel is not disabled, and 2)
   * the weak function lpc43_sspdev_initialize() has been brought into the
   * link.
   */

#if defined(CONFIG_LPC43_SSP0) || defined(CONFIG_LPC43_SSP1)
  if (lpc43_sspdev_initialize)
    {
      lpc43_sspdev_initialize();
    }
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize().
 *   board_late_initialize() will be called immediately after up_initialize()
 *   is called and just before the initial application is started.
 *   This additional initialization phase may be used, for example, to
 *   initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret = 0;
#ifdef HAVE_MMCSD
  struct sdio_dev_s *sdmmc;
#endif

  /* Configure max31855 driver for SSP0 or SSP1 */

#if defined(CONFIG_SENSORS_MAX31855)

#if defined(CONFIG_LPC43_SSP0)
  ret = lpc43_max31855initialize("/dev/temp0", SSP0, DEVID0);
  if (ret < 0)
    {
      serr("ERROR:  lpc43_max31855initialize failed: %d\n", ret);
    }

  ret = lpc43_max31855initialize("/dev/temp1", SSP0, DEVID1);
  if (ret < 0)
    {
      serr("ERROR:  lpc43_max31855initialize failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_LPC43_SSP1)
  ret = lpc43_max31855initialize("/dev/temp2", SSP1, DEVID0);
  if (ret < 0)
    {
      serr("ERROR:  lpc43_max31855initialize failed: %d\n", ret);
    }

  ret = lpc43_max31855initialize("/dev/temp3", SSP1, DEVID1);
  if (ret < 0)
    {
      serr("ERROR:  lpc43_max31855initialize failed: %d\n", ret);
    }
#endif
#endif

#ifdef CONFIG_LPC43_SPIFI
  /* Initialize the SPIFI block device */

  ret = nsh_spifi_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: nsh_spifi_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_MMCSD
  /* Get an instance of the SDIO interface */

  sdmmc = lpc43_sdmmc_initialize(0);
  if (!sdmmc)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
    }
  else
    {
      /* Bind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(MMCSD_MINOR, sdmmc);
      if (ret != OK)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
                 ret);
        }
    }
#endif

#ifdef CONFIG_TIMER
  /* Registers the timers */

  lpc43_timerinitialize();
#endif

  UNUSED(ret);
}
#endif
