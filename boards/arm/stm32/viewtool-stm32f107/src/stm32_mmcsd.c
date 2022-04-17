/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/src/stm32_mmcsd.c
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

#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "stm32_sdio.h"
#include "viewtool_stm32f107.h"

/* Only the STM32F103 supports the SDIO interface */

#ifdef CONFIG_ARCH_CHIP_STM32F103VC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_MMCSD           1 /* Assume that we have SD support */
#define STM32_MMCSDSLOTNO    0 /* There is only one slot */

/* Can't support MMC/SD features if the SDIO peripheral is disabled */

#ifndef CONFIG_STM32_SDIO
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
 *   and CONFIG_STM32_SDIO=y
 *
 ****************************************************************************/

int stm32_sdinitialize(int minor)
{
#ifdef HAVE_MMCSD
  struct sdio_dev_s *sdio;
  int ret;

  /* Configure the card-detect GPIO */
#warning REVISIT: Missing logic

  /* First, get an instance of the SDIO interface */

  sdio = sdio_initialize(STM32_MMCSDSLOTNO);
  if (!sdio)
    {
      ferr("ERROR: Failed to initialize SDIO slot %d\n", STM32_MMCSDSLOTNO);
      return -ENODEV;
    }

  finfo("Initialized SDIO slot %d\n", STM32_MMCSDSLOTNO);

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(minor, sdio);
  if (ret != OK)
    {
      ferr("ERROR:");
      ferr("Failed to bind SDIO slot %d to the MMC/SD driver, minor=%d\n",
           STM32_MMCSDSLOTNO, minor);
    }

  finfo("Bound SDIO slot %d to the MMC/SD driver, minor=%d\n",
         STM32_MMCSDSLOTNO, minor);

  /* Then let's guess and say that there is a card in the slot.  I need to
   * check to see if the M3 Wildfire board supports a GPIO to detect if there
   * is a card in the slot.
   */
#warning REVISIT: Need to read the current state of the card-detect pin
#warning REVISIT: Need to support interrupts from the card-detect pin
  sdio_mediachange(sdio, true);
#endif
  return OK;
}

#endif /* CONFIG_ARCH_CHIP_STM32F103VC */
