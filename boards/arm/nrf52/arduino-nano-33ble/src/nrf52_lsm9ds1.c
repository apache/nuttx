/****************************************************************************
 * boards/arm/nrf52/arduino-nano-33ble/src/nrf52_lsm9ds1.c
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
#include "arduino-nano-33ble.h"
#include <nuttx/sensors/lsm9ds1.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF52_I2C0_MASTER
#  error "LSM9DS1 driver requires CONFIG_NRF52_I2C0_MASTER to be enabled"
#endif

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

int nrf52_lsm9ds1_initialize(void)
{
  struct i2c_master_s *i2c;
  int ret = OK;

  sninfo("Initializing LMS9DS1!\n");

#if defined(CONFIG_NRF52_I2C0_MASTER)
  i2c = nrf52_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sninfo("INFO: Initializing LMS9DS1 9DoF sensor over I2C0\n");

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

  sninfo("INFO: LMS9DS1 sensor has been initialized successfully\n");
#endif

  return ret;
}
