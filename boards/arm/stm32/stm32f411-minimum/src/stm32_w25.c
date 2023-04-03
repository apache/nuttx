/****************************************************************************
 * boards/arm/stm32/stm32f411-minimum/src/stm32_w25.c
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
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <debug.h>

#ifdef CONFIG_STM32_SPI1
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#  include <nuttx/fs/smart.h>
#  include <nuttx/mtd/configdata.h>
#endif

#include "stm32_spi.h"

#include "stm32f411-minimum.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/* Non-standard debug that may be enabled just for testing the watchdog
 * timer
 */

#define W25_SPI_PORT 1

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
  int ret;
#ifdef HAVE_W25
  struct spi_dev_s *spi;
  struct mtd_dev_s *mtd;
  struct mtd_geometry_s geo;
#if defined(CONFIG_MTD_PARTITION_NAMES)
  const char *partname = CONFIG_STM32F411MINIMUM_FLASH_PART_NAMES;
#endif

  /* Get the SPI port */

  spi = stm32_spibus_initialize(W25_SPI_PORT);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
             W25_SPI_PORT);
      return -ENODEV;
    }

  /* Raise SPI frequency from default 400kHz to something usable
   * SPI1 uses PCLK2 of 96MHz with DIV2 = 48Mbps max
   * W25Q64 requires more dummy clocks above 26MHz
   */

  SPI_SETFREQUENCY(spi, 24000000);

  /* Now bind the SPI interface to the W25 SPI FLASH driver */

  mtd = w25_initialize(spi);
  if (!mtd)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port %d to the Winbond"
                      "W25 FLASH driver\n", W25_SPI_PORT);
      return -ENODEV;
    }

#ifndef CONFIG_FS_SMARTFS
  /* And use the FTL layer to wrap the MTD driver as a block driver */

  ret = ftl_initialize(minor, mtd);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Initialize the FTL layer\n");
      return ret;
    }
#else
  /* Initialize to provide SMARTFS on the MTD interface */

  /* Get the geometry of the FLASH device */

  ret = mtd->ioctl(mtd, MTDIOC_GEOMETRY, (unsigned long)((uintptr_t)&geo));
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mtd->ioctl failed: %d\n", ret);
      return ret;
    }

  /* Configure the device with no partition support */

  smart_initialize(CONFIG_STM32F411MINIMUM_FLASH_MINOR, mtd, NULL);

#endif /* CONFIG_FS_SMARTFS */
#endif /* HAVE_W25 */

  return OK;
}
