/****************************************************************************
 * boards/arm/nrf53/common/src/nrf53_bmi270.c
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

#include <nuttx/sensors/bmi270.h>

#ifdef CONFIG_SENSORS_BMI270_SPI
#  include <nuttx/spi/spi.h>
#  include "nrf53_spi.h"
#else
#  include "nrf53_i2c.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_bmi270spi_initialize
 *       nrf53_bmi270i2c_initialize
 *
 * Description:
 *   Initialzie and register the BMI270 character device as 'devpath'
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

#ifdef CONFIG_SENSORS_BMI270_SPI
int nrf53_bmi270spi_initialize(int devno, int busno)
{
  struct spi_dev_s *spi;
  int ret;

  sninfo("Initializing BMI270..\n");

  /* Initialize spi device */

  spi = nrf53_spibus_initialize(busno);
  if (!spi)
    {
      snerr("ERROR: Failed to initialize spi%d.\n", busno);
      return -ENODEV;
    }

  ret = bmi270_register_uorb(devno, spi);
  if (ret < 0)
    {
      snerr("Error registering BMI160\n");
    }

  return ret;
}
#else  /* CONFIG_SENSORS_BMI270_I2C */
int nrf53_bmi270i2c_initialize(int devno, int busno, uint8_t addr)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing BMI270!\n");

  /* Initialize I2C */

  i2c = nrf53_i2cbus_initialize(busno);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  ret = bmi270_register_uorb(devno, i2c, addr);
  if (ret < 0)
    {
      snerr("ERROR: Error registering BM180\n");
    }

  return ret;
}
#endif
