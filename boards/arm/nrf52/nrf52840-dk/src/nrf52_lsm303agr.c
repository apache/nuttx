/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_lsm303agr.c
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
#include "nrf52840-dk.h"
#include <nuttx/sensors/lsm303agr.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_NRF52_I2C0_MASTER
#  error "LSM303AGR driver requires CONFIG_NRF52_I2C0_MASTER to be enabled"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nrf52_lsm303agr_initialize
 *
 * Description:
 *   Initialize I2C-based LSM303AGR.
 *
 ****************************************************************************/

int nrf52_lsm303agr_initialize(char *devpath)
{
  FAR struct i2c_master_s *i2c;
  int ret = OK;

  sninfo("Initializing LSM303AGR!\n");

#ifdef CONFIG_NRF52_I2C0_MASTER
  i2c = nrf52_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      return -ENODEV;
    }

  sninfo("INFO: Initializing LSM303AGR accelero-magnet sensor over I2C%d\n",
         ret);

  ret = lsm303agr_sensor_register(devpath, i2c, LSM303AGRMAGNETO_ADDR);
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LSM303AGR acc-gyro driver %s\n",
            devpath);
      return -ENODEV;
    }

  sninfo("INFO: LSM303AGR sensor has been initialized successfully\n");
#endif

  return ret;
}
