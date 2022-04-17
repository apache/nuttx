/****************************************************************************
 * boards/arm/stm32/shenzhou/src/stm32_mmcsd.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include "stm32_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* SPI1 connects to the SD CARD (and to the SPI FLASH) */

#define HAVE_MMCSD           1 /* Assume that we have SD support */
#define STM32_MMCSDSPIPORTNO 1 /* Port is SPI1 */
#define STM32_MMCSDSLOTNO    0 /* There is only one slot */

#ifndef CONFIG_STM32_SPI1
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if MMC/SD driver support is not selected */

#ifndef CONFIG_MMCSD
#  undef HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_MMCSD
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_sdinitialize
 *
 * Description:
 *   Initialize the SPI-based SD card.  Requires CONFIG_DISABLE_MOUNTPOINT=n
 *   and CONFIG_STM32_SPI1=y
 *
 ****************************************************************************/

int stm32_sdinitialize(int minor)
{
#ifdef HAVE_MMCSD
  struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  finfo("Initializing SPI port %d\n", STM32_MMCSDSPIPORTNO);

  spi = stm32_spibus_initialize(STM32_MMCSDSPIPORTNO);
  if (!spi)
    {
      ferr("ERROR: Failed to initialize SPI port %d\n",
           STM32_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  finfo("Successfully initialized SPI port %d\n", STM32_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  finfo("Binding SPI port %d to MMC/SD slot %d\n",
          STM32_MMCSDSPIPORTNO, STM32_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(minor, STM32_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      ferr("ERROR: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
            STM32_MMCSDSPIPORTNO, STM32_MMCSDSLOTNO, ret);
      return ret;
    }

  finfo("Successfully bound SPI port %d to MMC/SD slot %d\n",
        STM32_MMCSDSPIPORTNO, STM32_MMCSDSLOTNO);
#endif
  return OK;
}
