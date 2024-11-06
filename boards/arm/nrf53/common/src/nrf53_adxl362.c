/****************************************************************************
 * boards/arm/nrf53/common/src/nrf53_adxl362.c
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

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/sensors/adxl362.h>

#include <nuttx/spi/spi.h>
#include "nrf53_spi.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_adxl362_init
 *
 * Description:
 *   Initialize and register the ADXL362 device
 *
 * Input Parameters:
 *   devno - The user specifies device number, from 0.
 *   busno - SPI or I2C bus number
 *   addr  - (I2C only) The I2C address
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf53_adxl362_init(int devno, int busno)
{
  struct spi_dev_s *spi;
  int ret;

  sninfo("Initializing ADXL362..\n");

  /* Initialize spi device */

  spi = nrf53_spibus_initialize(busno);
  if (!spi)
    {
      snerr("ERROR: Failed to initialize spi%d.\n", busno);
      return -ENODEV;
    }

  ret = adxl362_register(devno, spi);
  if (ret < 0)
    {
      snerr("Error registering ADXL362\n");
    }

  return ret;
}
