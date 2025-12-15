/****************************************************************************
 * boards/risc-v/esp32c3/common/src/esp_board_bmp280.c
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

#include <nuttx/arch.h>
#include <nuttx/sensors/bmp280.h>
#include <nuttx/i2c/i2c_master.h>

#ifndef CONFIG_ESPRESSIF_I2C_BITBANG
#include "espressif/esp_i2c.h"
#else
#include "espressif/esp_i2c_bitbang.h"
#endif

#include "esp_board_bmp280.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_bmp280_initialize
 *
 * Description:
 *   Initialize and register the BMP280 Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/pressN
 *
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_bmp280_initialize(int devno)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing BMP280!\n");

  /* Initialize I2C */

#ifndef CONFIG_ESPRESSIF_I2C_BITBANG
  i2c = esp_i2cbus_initialize(ESPRESSIF_I2C0);
#else
  i2c = esp_i2cbus_bitbang_initialize();
#endif

  if (i2c)
    {
      /* Then try to register the barometer sensor in I2C0 */

      ret = bmp280_register(devno, i2c);
      if (ret < 0)
        {
          snerr("ERROR: Error registering BMP280 in I2C0\n");
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
