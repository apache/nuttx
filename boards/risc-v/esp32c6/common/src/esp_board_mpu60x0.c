/****************************************************************************
 * boards/risc-v/esp32c6/common/src/esp_board_mpu60x0.c
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
#include <nuttx/sensors/mpu60x0.h>
#include <nuttx/i2c/i2c_master.h>

#ifndef CONFIG_ESPRESSIF_I2C_BITBANG
#include "espressif/esp_i2c.h"
#else
#include "espressif/esp_i2c_bitbang.h"
#endif

#include "esp_board_mpu60x0.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_mpu60x0_initialize
 *
 * Description:
 *   Initialize and register the MPU60x0 IMU Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/imuN
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_mpu60x0_initialize(int devno)
{
  struct i2c_master_s *i2c;
  struct mpu_config_s *config;
  char devpath[12];
  int ret;

  sninfo("Initializing MPU60x0!\n");

  /* Initialize MPU60x0 */

#ifndef CONFIG_ESPRESSIF_I2C_BITBANG
  i2c = esp_i2cbus_initialize(ESPRESSIF_I2C0);
#else
  i2c = esp_i2cbus_bitbang_initialize();
#endif

  config = kmm_zalloc(sizeof(struct mpu_config_s));
  if (config == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to allocate mpu60x0 driver\n");
      return -ENOMEM;
    }
  else
    {
      config->i2c = i2c;
      config->addr = 0x68;
    }

  if (i2c != NULL)
    {
      /* Then try to register the barometer sensor in I2C0 */

      snprintf(devpath, sizeof(devpath), "/dev/imu%d", devno);
      ret = mpu60x0_register(devpath, config);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Error registering MPU60x0 in I2C0\n");
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
