/****************************************************************************
 * boards/arm/stm32l4/nucleo-l432kc/src/stm32_w25.c
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

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <nuttx/debug.h>

#include <nuttx/spi/spi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/smart.h>

#include <sys/mount.h>

#include "stm32l4_spi.h"
#include "nucleo-l432kc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_STM32_SPI1
#  error "W25 driver requires CONFIG_STM32_SPI1 to be enabled"
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const int W25_SPI_PORT = 1;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_w25initialize
 *
 * Description:
 *   Initialize and configure the W25 SPI Serial Flash Memory.
 *
 ****************************************************************************/

int stm32_w25initialize(int minor)
{
  FAR struct spi_dev_s *spi;
  FAR struct mtd_dev_s *mtd;
  static bool initialized = false;
  int ret = OK;

  if (!initialized)
    {
      finfo("INFO: Initializing W25\n");

      spi = stm32_spibus_initialize(W25_SPI_PORT);
      if (spi == NULL)
        {
          ferr("ERROR: Failed to initialize SPI port %d\n", W25_SPI_PORT);
          return -ENODEV;
        }

      mtd = w25_initialize(spi);
      if (mtd == NULL)
        {
          ferr("ERROR: Failed to initialize W25\n");
          return -ENODEV;
        }

#if !defined(CONFIG_DISABLE_MOUNTPOINT)
      ret = register_mtddriver("/dev/mtd0", mtd, 0777, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to register /dev/mtd0: %d\n", ret);
          return ret;
        }

      ret = ftl_initialize(0, mtd);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize FTL: %d\n", ret);
        }
#endif

#if defined(CONFIG_FS_SMARTFS)
      finfo("Initialize SMARTFS on /dev/smart0\n");
      ret = smart_initialize(0, mtd, NULL);
      if (ret < 0)
        {
          ferr("ERROR: Failed to initialize SMARTFS: %d\n", ret);
        }
      else
        {
          /* Try to mount SMARTFS if already formatted */

          mount("/dev/smart0", "/mnt/w25", "smartfs", 0, NULL);
        }
#endif

      initialized = true;
    }

  return OK;
}
