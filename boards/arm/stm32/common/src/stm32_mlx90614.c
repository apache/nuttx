/****************************************************************************
 * boards/arm/stm32/common/src/stm32_mlx90614.c
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
#include <stdio.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/mlx90614.h>

#include "stm32.h"
#include "stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MLX90614_ADDRESS    0x5a /* Default I2C Address, can be chanced in EEPROM */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mlx90614_initialize
 *
 * Description:
 *   Initialize and register the MLX90614 Infrared Thermometer.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/thermN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_mlx90614_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  char devpath[12];
  int ret;

  sninfo("Initializing MLX90614!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);

  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the light sensor */

  snprintf(devpath, 12, "/dev/therm%d", devno);
  ret = mlx90614_register(devpath, i2c, MLX90614_ADDRESS);
  if (ret < 0)
    {
      snerr("ERROR: Error registering BM180\n");
    }

  return ret;
}
