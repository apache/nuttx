/****************************************************************************
 * boards/arm/am67/t3-gem-o1/src/am67_spi.c
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
#include <inttypes.h>
#include <stdint.h>

#include <nuttx/spi/spi.h>
#include <nuttx/spi/spi_transfer.h>

#include "am67_gpio.h"
#include "am67_mcspi.h"

#ifdef CONFIG_AM67_MCSPI0

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* SPIDEV_USER(n) = SPIDEV_ID(SPIDEVTYPE_USER, n): the spi tool's -t 0
 * selects SPIDEVTYPE_USER, so devid = SPIDEV_USER(csn).
 */

#define AM67_SPIDEV_LPS22DF   SPIDEV_USER(1)   /* CS1 per Linux DTS */
#define AM67_SPIDEV_ICM20948  SPIDEV_USER(3)   /* CS3 per Linux DTS */

/* MCSPI channel numbers for each device (low byte of SPIDEV_USER(n)) */

#define AM67_CH_LPS22DF       1u
#define AM67_CH_ICM20948      3u

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_spi0select
 ****************************************************************************/

void am67_spi0select(FAR struct spi_dev_s *dev, uint32_t devid,
                     bool selected)
{
  switch (devid)
    {
      case AM67_SPIDEV_LPS22DF:
        am67_mcspi_board_select(dev, AM67_CH_LPS22DF, selected);
        break;
      case AM67_SPIDEV_ICM20948:
        am67_mcspi_board_select(dev, AM67_CH_ICM20948, selected);
        break;

      default:
        spierr("ERROR: Unsupported SPI devid: %" PRIu32 "\n", devid);
        break;
    }
}

/****************************************************************************
 * Name: am67_spi0status
 ****************************************************************************/

uint8_t am67_spi0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  switch (devid)
    {
      case AM67_SPIDEV_LPS22DF:
      case AM67_SPIDEV_ICM20948:
        return SPI_STATUS_PRESENT;

      default:
        return 0;
    }
}

/****************************************************************************
 * Name: am67_spidev_initialize
 ****************************************************************************/

void am67_spidev_initialize(void)
{
  FAR struct spi_dev_s *spi;
  int ret;

  am67_configgpio(AM67_GPIO_HAT_CS1, GPIO_OUTPUT);
  am67_configgpio(AM67_GPIO_HAT_CS2, GPIO_OUTPUT);
  am67_configgpio(AM67_GPIO_HAT_CS3, GPIO_OUTPUT);

  spi = am67_spibus_initialize(0);
  if (spi == NULL)
    {
      spierr("ERROR: Failed to initialize SPI0\n");
      return;
    }

  ret = spi_register(spi, 0);
  if (ret < 0)
    {
      spierr("ERROR: Failed to register /dev/spi0: %d\n", ret);
    }
}

#endif /* CONFIG_AM67_MCSPI0 */
