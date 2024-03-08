/****************************************************************************
 * boards/arm/nrf52/common/src/nrf52_lsm9ds1.c
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
#include <nuttx/arch.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include "nrf52_i2c.h"
#include <nuttx/sensors/lsm9ds1.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSM9DS1MAG_DEVPATH "/dev/lsm9ds1mag0"
#define LSM9DS1ACC_DEVPATH "/dev/lsm9ds1acc0"
#define LSM9DS1GYR_DEVPATH "/dev/lsm9ds1gyr0"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_lsm9ds1_initialize
 *
 * Description:
 *   Initialize I2C-based LSM9DS1.
 *
 ****************************************************************************/

int nrf52_lsm9ds1_initialize(int bus)
{
#ifdef CONFIG_SENSORS_LSM9DS1_UORB
  struct lsm9ds1_config_s  config;
  struct i2c_master_s     *i2c;
  int                      ret = OK;

  sninfo("Initializing LMS6DSL!\n");

  i2c = nrf52_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sninfo("INFO: Initializing LMS9DS1 9DoF sensor over I2C%d\n", bus);

  config.i2c       = i2c;
  config.addr_acc  = LSM9DS1ACCEL_ADDR1;
  config.addr_gyro = LSM9DS1GYRO_ADDR1;
  config.addr_mag  = LSM9DS1MAG_ADDR1;

  /* Register sensor as uorb devices */

  ret = lsm9ds1_register_uorb(0, &config);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LMS9DS1 mag driver\n");
      return -ENODEV;
    }

  sninfo("INFO: LMS9DS1 sensor has been initialized successfully\n");

  return ret;
#else
  struct i2c_master_s *i2c;
  int ret = OK;

  sninfo("Initializing LMS6DSL!\n");

  i2c = nrf52_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sninfo("INFO: Initializing LMS9DS1 9DoF sensor over I2C%d\n", bus);

  /* Register sensor as character devices */

  ret = lsm9ds1mag_register(LSM9DS1MAG_DEVPATH, i2c, LSM9DS1MAG_ADDR1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LMS9DS1 mag driver %s\n",
            LSM9DS1MAG_DEVPATH);
      return -ENODEV;
    }

  ret = lsm9ds1gyro_register(LSM9DS1GYR_DEVPATH, i2c, LSM9DS1GYRO_ADDR1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LMS9DS1 gyro driver %s\n",
            LSM9DS1MAG_DEVPATH);
      return -ENODEV;
    }

  ret = lsm9ds1accel_register(LSM9DS1ACC_DEVPATH, i2c, LSM9DS1ACCEL_ADDR1);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LMS9DS1 accel driver %s\n",
            LSM9DS1MAG_DEVPATH);
      return -ENODEV;
    }

  return ret;
#endif
}
