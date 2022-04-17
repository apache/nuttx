/****************************************************************************
 * boards/arm/xmc4/xmc4500-relax/src/xmc4_max6675.c
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

#include <nuttx/spi/spi.h>
#include <nuttx/sensors/max6675.h>

#include "xmc4_spi.h"
#include "xmc4500-relax.h"

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_MAX6675) && defined(CONFIG_XMC4_SPI2)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAX6675_SPI_PORTNO 2   /* On SPI2 */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xmc4_max6675initialize
 *
 * Description:
 *   Initialize and register the MAX6675 Temperature Sensor driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int xmc4_max6675initialize(const char *devpath)
{
  struct spi_dev_s *spi;
  int ret;

  spi = xmc4_spibus_initialize(MAX6675_SPI_PORTNO);

  if (!spi)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  ret = max6675_register(devpath, spi);
  if (ret < 0)
    {
      snerr("ERROR: Error registering MAX6675\n");
    }

  return ret;
}

#endif /* CONFIG_SPI && CONFIG_SENSORS_MAX6675 */
