/****************************************************************************
 * boards/risc-v/esp32c6/common/src/esp_board_bmp180.c
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
#include <nuttx/sensors/bmp180.h>
#include <nuttx/i2c/i2c_master.h>

#include "espressif/esp_i2c.h"

#include "esp_board_bmp180.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_bmp180_initialize
 *
 * Description:
 *   Initialize and register the BMP180 Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/pressN
 *
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_bmp180_initialize(int devno)
{
  struct i2c_master_s *i2c;
  char devpath[12];
  int ret;

  sninfo("Initializing BMP180!\n");

  /* Initialize BMP180 */

  i2c = esp_i2cbus_initialize(ESPRESSIF_I2C0);

  if (i2c)
    {
      /* Then try to register the barometer sensor in I2C0 */

      snprintf(devpath, sizeof(devpath), "/dev/press%d", devno);
      ret = bmp180_register(devpath, i2c);
      if (ret < 0)
        {
          snerr("ERROR: Error registering BMP180 in I2C0\n");
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
