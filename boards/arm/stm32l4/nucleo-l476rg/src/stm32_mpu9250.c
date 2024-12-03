/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_mpu9250.c
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
#include <nuttx/sensors/mpu9250.h>
#include <nuttx/i2c/i2c_master.h>

#include "stm32l4.h"
#include "stm32l4_i2c.h"
#include "stm32_mpu9250.h"

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
 * Name: board_mpu9250_initialize
 *
 * Description:
 *   Initialize and register
 *     MPU9250 (Gyro + Accelerometer + Compass) Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as
 *            /dev/uorb/sensor_accelx
 *            /dev/uorb/sensor_gyrox
 *            /dev/uorb/sensor_magx
 *
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_mpu9250_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing MPU9250!\n");

  /* Initialize MPU9250 */

  i2c = stm32l4_i2cbus_initialize(busno);
  if (i2c)
    {
      /* Then try to register the IMU sensor in I2Cx */

      struct mpu9250_config_s mpuc;
      memset(&mpuc, 0, sizeof(mpuc));
      mpuc.i2c = i2c;
      mpuc.addr = 0x68;

      ret = mpu9250_register(devno, &mpuc);
      if (ret < 0)
        {
          snerr("ERROR: Error registering MPU9250 in I2C%d\n", busno);
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
