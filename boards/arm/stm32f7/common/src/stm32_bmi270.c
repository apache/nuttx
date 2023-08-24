/****************************************************************************
 * boards/arm/stm32f7/common/src/stm32_bmi270.c
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

#include <nuttx/spi/spi.h>
#include <arch/board/board.h>
#include <nuttx/sensors/bmi270.h>

#include "stm32_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI270_I2C_ADDR    0x68

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_bmi270_initialize
 *
 * Description:
 *   Initialize and register the BMI270 IMU device.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/imuN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_bmi270_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  char devpath[16];
  int ret;

  sninfo("Initializing BMI270!\n");

  /* Initialize I2C */

  i2c = stm32_i2cbus_initialize(busno);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the ambient light sensor */

  snprintf(devpath, sizeof(devpath), "/dev/imu%d", devno);
  ret = bmi270_register(devpath, i2c, BMI270_I2C_ADDR);
  if (ret < 0)
    {
      snerr("ERROR: Error registering BMI270\n");
    }

  return ret;
}

