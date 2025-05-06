/****************************************************************************
 * boards/arm/nrf53/common/src/nrf53_bme688.c
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
#include <errno.h>
#include <stdio.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bme688.h>

#include "nrf53_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf53_bme688_init
 *
 * Description:
 *   Initialize and register the BME688 as uorb sensor
 *
 * Input Parameters:
 *   devno - The user specifies device number, from 0.
 *   busno - I2C bus number
 *   addr  - The I2C address
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int nrf53_bme688_init(int devno, int busno, uint8_t addr)
{
  struct bme688_config_s  config;
  struct i2c_master_s    *i2c;
  int                     ret;

  /* Address not configurable yet */

  UNUSED(addr);

  sninfo("Initializing BME688!\n");

  /* Initialize I2C */

  i2c = nrf53_i2cbus_initialize(busno);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Default configuration */

  config.temp_os         = BME688_OS_2X;
#ifndef CONFIG_BME688_DISABLE_PRESS_MEAS
  config.press_os        = BME688_OS_16X;
#endif
#ifndef CONFIG_BME688_DISABLE_HUM_MEAS
  config.hum_os          = BME688_OS_1X;
#endif
#ifdef CONFIG_BME688_ENABLE_IIR_FILTER
  config.filter_coef     = BME688_FILTER_COEF3;
#endif
#ifndef CONFIG_BME688_DISABLE_GAS_MEAS
  config.target_temp     = 300;
  config.heater_duration = 100;
  config.nb_conv         = 0;
  config.amb_temp        = 30;
#endif

  /* Then register the sensor */

  ret = bme688_register(devno, i2c, &config);
  if (ret < 0)
    {
      snerr("ERROR: Error registering BME688\n");
    }

  return ret;
}
