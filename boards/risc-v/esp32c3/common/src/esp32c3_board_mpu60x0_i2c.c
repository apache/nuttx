/****************************************************************************
 * boards/risc-v/esp32c3/common/src/esp32c3_board_mpu60x0_i2c.c
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
#include <nuttx/kmalloc.h>
#include <nuttx/sensors/mpu60x0.h>
#include <nuttx/i2c/i2c_master.h>

#include "esp32c3_i2c.h"
#include "esp32c3_board_mpu60x0_i2c.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mpu60x0_initialize
 *
 * Description:
 *   Initialize and register the MPU60x0 Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/imuN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_mpu60x0_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  char devpath[10];
  FAR struct mpu_config_s *mpu_config;
  int ret;

  sninfo("Initializing MPU60X0!\n");

  /* Initialize MPU60X0 */

  i2c = esp32c3_i2cbus_initialize(busno);

  if (i2c)
    {
      /* Then try to register the IMU sensor in I2C<busno> */

      mpu_config = kmm_zalloc(sizeof(struct mpu_config_s));
      if (mpu_config == NULL)
        {
          snerr("ERROR: Failed to allocate mpu60x0 driver\n");
          ret = -ENODEV;
        }
      else
        {
          mpu_config->i2c = i2c;
          mpu_config->addr = 0x68;
          snprintf(devpath, sizeof(devpath), "/dev/imu%d", devno);
          ret = mpu60x0_register(devpath, mpu_config);
          if (ret < 0)
            {
              snerr("ERROR: Error registering MPU60X0 in I2C%d\n", busno);
            }
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}

