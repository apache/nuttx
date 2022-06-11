/****************************************************************************
 * boards/arm/lpc43xx/bambino-200e/src/lpc43_max31855.c
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

#include <nuttx/sensors/max31855.h>

#include "bambino-200e.h"

#include <nuttx/spi/spi.h>

#include "lpc43_spi.h"
#include "lpc43_ssp.h"

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/random.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MAX31855)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc43_max31855initialize
 *
 * Description:
 *   Initialize and register the MAX31855 Temperature Sensor driver.
 *
 * Input parameters:
 *   devpath - The full path to the driver to register.  E.g., "/dev/temp0"
 *   spi     - An instance of the SPI interface to use to communicate with
 *             MAX31855
 *   devid   - Minor device number. E.g., 0, 1, 2, etc.
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lpc43_max31855initialize(const char *devpath, int bus,
                             uint16_t devid)
{
  struct spi_dev_s *spi;
  spi = lpc43_sspbus_initialize(bus);
  if (!spi)
    {
      snerr("ERROR: Failed to initialize SSP%d\n", bus);
      return -ENODEV;
    }

  /* Then register the temperature sensor */

  int ret = max31855_register(devpath, spi, devid);
  if (ret < 0)
    {
      snerr("ERROR: Error registering MAX31855\n");
    }

  return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_MAX31855 */
