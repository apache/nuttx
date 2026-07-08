/****************************************************************************
 * boards/arm/nrf91/common/src/nrf91_bme680.c
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
#include <errno.h>
#include <stdio.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bme680.h>

#include "nrf91_i2c.h"

/****************************************************************************
 * Private data
 ****************************************************************************/

/* Default measurement configuration.  Without a configuration the driver
 * never leaves the "not calibrated" state and no measurements are taken.
 * Oversampling follows the Bosch "indoor navigation" recommendation; the
 * gas heater profile targets 320 C for 150 ms.
 */

struct bme680_config_s g_bme680_config =
{
  .temp_os         = BME680_OS_8X,
  .press_os        = BME680_OS_4X,
  .hum_os          = BME680_OS_2X,
  .target_temp     = 320,
  .heater_duration = 150,
  .nb_conv         = 0,
  .amb_temp        = 25,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf91_bme680_init
 ****************************************************************************/

int nrf91_bme680_init(int devno, int busno)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing BME680!\n");

  /* Initialize I2C */

  i2c = nrf91_i2cbus_initialize(busno);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the barometer sensor */

  ret = bme680_register(devno, i2c, &g_bme680_config);
  if (ret < 0)
    {
      snerr("ERROR: Error registering BME680\n");
    }

  return ret;
}
