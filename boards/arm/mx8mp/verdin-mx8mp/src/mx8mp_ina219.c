/****************************************************************************
 * boards/arm/mx8mp/verdin-mx8mp/src/mx8mp_ina219.c
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

#include <errno.h>
#include <syslog.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/sensors/ina219.h>

#include "mx8mp_i2c.h"
#include "mx8mp_ina219.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ina219_initialize
 *
 * Description:
 *   Initialize and register the INA219 voltage/current sensor.
 *
 * Input parameters:
 *   devno - The device number, used to build the device path as /dev/inaN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ina219_initialize(int busno)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing INA219!\n");

  /* Initialize I2C */

  i2c = mx8mp_i2cbus_initialize(busno);
  if (!i2c)
    {
      return -ENODEV;
    }

  /* Then register the sensor */

  ret = ina219_register("/dev/ina219", i2c, 0x40, 10000, 0x00);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Error registering INA219\n");
    }

  return ret;
}
