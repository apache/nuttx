/****************************************************************************
 * boards/arm/stm32/common/src/stm32_max6675.c
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
#include <nuttx/arch.h>
#include <nuttx/sensors/max6675.h>
#include <nuttx/spi/spi.h>
#include <stdio.h>
#include <debug.h>
#include "stm32_spi.h"

#ifdef CONFIG_SENSORS_MAX6675

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_max6675_initialize
 *
 * Description:
 *   Initialize and register the MAX6675 Temperature Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/tempN
 *   busno - The SPI bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_max6675_initialize(int devno, int busno)
{
  struct spi_dev_s *spi;
  char devpath[12];
  int ret;

  spi = stm32_spibus_initialize(busno);

  if (!spi)
    {
      return -ENODEV;
    }

  /* Then register the temperature sensor */

  snprintf(devpath, 12, "/dev/temp%d", devno);
  ret = max6675_register(devpath, spi);
  if (ret < 0)
    {
      snerr("ERROR: Error registering MAX6675\n");
    }

  return ret;
}

#endif
