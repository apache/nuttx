/****************************************************************************
 * boards/arm/stm32/stm32butterfly2/src/stm32_spi.c
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

#include <inttypes.h>
#include <debug.h>
#include <nuttx/spi/spi.h>

#include "stm32_butterfly2.h"
#include "stm32_gpio.h"
#include "stm32_spi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spidev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins.
 *
 * Note:
 *   Here only CS pins are configured as SPI pins are configured by driver
 *   itself.
 ****************************************************************************/

void stm32_spidev_initialize(void)
{
  spiinfo("INFO: Initializing spi gpio pins\n");

  stm32_configgpio(GPIO_SD_CS);
  stm32_configgpio(GPIO_SD_CD);
}

/****************************************************************************
 * Name: stm32_spi1select
 *
 * Description:
 *   Function asserts given devid based on select
 ****************************************************************************/

void stm32_spi1select(struct spi_dev_s *dev, uint32_t devid,
                      bool select)
{
  spiinfo("INFO: Selecting spi dev: %" PRId32 ", state: %d\n",
          devid, select);

  if (devid == SPIDEV_MMCSD(0))
    {
      stm32_gpiowrite(GPIO_SD_CS, !select);
    }
}

/****************************************************************************
 * Name: stm32_spi1status
 *
 * Description:
 *   Return status of devid
 ****************************************************************************/

uint8_t stm32_spi1status(struct spi_dev_s *dev, uint32_t devid)
{
  spiinfo("INFO: Requesting info from spi dev: %" PRId32 "\n", devid);

  if (devid == SPIDEV_MMCSD(0))
    {
      if (stm32_gpioread(GPIO_SD_CD) == 0)
        {
          return SPI_STATUS_PRESENT;
        }
    }

  return 0;
}
