/****************************************************************************
 * boards/arm/lpc43xx/bambino-200e/src/lpc43_ssp.c
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

#include <errno.h>
#include <debug.h>
#include <string.h>

#include "lpc43_spi.h"
#include "lpc43_ssp.h"

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "bambino-200e.h"

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/random.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_sspdev_initialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the Bambino-200e
 *   board.
 *
 ****************************************************************************/

void weak_function lpc43_sspdev_initialize(void)
{
#ifdef CONFIG_SENSORS_MAX31855
  lpc43_pin_config(PINCONFIG_MAX31855_CS1);
  int ret = lpc43_gpio_config(GPIO_MAX31855_CS1);
  if (ret < 0)
    {
      snerr("ERROR: Error configuring chip select GPIO pin\n");
    }

  lpc43_pin_config(PINCONFIG_MAX31855_CS2);
  ret = lpc43_gpio_config(GPIO_MAX31855_CS2);
  if (ret < 0)
    {
      snerr("ERROR: Error configuring chip select GPIO pin\n");
    }
#endif
}

/****************************************************************************
 * Name:  lpc43_ssp0select
 *
 * Description:
 *   Perform chip selection using GPIO pins, controlling data flow in SSP0
 *   channel.
 *
 * Input parameters:
 *   devpath  - The full path to the driver.  E.g., "/dev/temp0"
 *   devid    - Minor device number.  E.g., 0, 1, 2, etc.
 *   selected - Logical state of the pin
 *
 ****************************************************************************/

void lpc43_ssp0select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

#if defined(CONFIG_SENSORS_MAX31855)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      lpc43_gpio_write(GPIO_MAX31855_CS1, !selected);
    }

  if (devid == SPIDEV_TEMPERATURE(1))
    {
      lpc43_gpio_write(GPIO_MAX31855_CS2, !selected);
    }
#endif
}

/****************************************************************************
 * Name:  lpc43_ssp0status
 *
 * Description:
 *   Perform status operations in SSP0 channel, using GPIO pins.
 *
 * Input parameters:
 *   devpath  - The full path to the driver.  E.g., "/dev/temp0"
 *   devid    - Minor device number.  E.g., 0, 1, 2, etc.
 *
 * Returned Value:
 *   Zero
 ****************************************************************************/

uint8_t lpc43_ssp0status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/****************************************************************************
 * Name:  lpc43_ssp1select
 *
 * Description:
 *   Perform chip selection using GPIO pins, controlling data flow in SSP1
 *   channel.
 *
 * Input parameters:
 *   devpath  - The full path to the driver.  E.g., "/dev/temp0"
 *   devid    - Minor device number.  E.g., 0, 1, 2, etc.
 *   selected - Logical state of the pin
 *
 ****************************************************************************/

void lpc43_ssp1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  spiinfo("devid: %d CS: %s\n", (int)devid,
          selected ? "assert" : "de-assert");

#if defined(CONFIG_SENSORS_MAX31855)
  if (devid == SPIDEV_TEMPERATURE(0))
    {
      lpc43_gpio_write(GPIO_MAX31855_CS1, !selected);
    }

  if (devid == SPIDEV_TEMPERATURE(1))
    {
      lpc43_gpio_write(GPIO_MAX31855_CS2, !selected);
    }
#endif
}

/****************************************************************************
 * Name:  lpc43_ssp1status
 *
 * Description:
 *   Perform status operations in SSP1 channel, using GPIO pins.
 *
 * Input parameters:
 *   devpath  - The full path to the driver.  E.g., "/dev/temp0"
 *   devid    - Minor device number.  E.g., 0, 1, 2, etc.
 *
 * Returned Value:
 *   Zero
 ****************************************************************************/

uint8_t lpc43_ssp1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
