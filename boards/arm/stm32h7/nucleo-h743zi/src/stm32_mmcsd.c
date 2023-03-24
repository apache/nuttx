/****************************************************************************
 * boards/arm/stm32h7/nucleo-h743zi/src/stm32_mmcsd.c
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
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>

#include <arch/board/board.h>

#include "stm32_spi.h"

#include "nucleo-h743zi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  error "SD driver requires CONFIG_DISABLE_MOUNTPOINT to be disabled"
#endif

#ifndef CONFIG_STM32H7_SPI3
#  error "MMC/SD requires SPI3 enabled"
#endif

#define MMCSD_SPI_PORT (3)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_mmcsd_initialize
 *
 * Description:
 *   Initialize SPI-based SD card and card detect thread.
 *
 ****************************************************************************/

int stm32_mmcsd_initialize(int minor)
{
  struct spi_dev_s *spi;
  int ret;

  mcinfo("INFO: Initializing mmcsd port %d minor %d\n",
         MMCSD_SPI_PORT, minor);

  spi = stm32_spibus_initialize(MMCSD_SPI_PORT);
  if (spi == NULL)
    {
      mcerr("ERROR: Failed to initialize SPI port %d\n", MMCSD_SPI_PORT);
      return -ENODEV;
    }

  ret = mmcsd_spislotinitialize(minor, 0, spi);
  if (ret < 0)
    {
      mcerr("ERROR: Failed to bind SPI port %d to SD slot %d\n",
            MMCSD_SPI_PORT, minor);
      return ret;
    }

  mcinfo("INFO: mmcsd card has been initialized successfully\n");
  return OK;
}
