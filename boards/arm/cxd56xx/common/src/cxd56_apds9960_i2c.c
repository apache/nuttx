/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_apds9960_i2c.c
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
#include <errno.h>

#include <nuttx/board.h>

#include <nuttx/sensors/apds9960.h>

#include "cxd56_i2c.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#if defined(CONFIG_CXD56_I2C) && defined(CONFIG_APDS9960)

int board_apds9960_initialize(const char *devpath, int bus)
{
  int ret;
  struct i2c_master_s *i2c;

  sninfo("Initializing APDS9960...\n");

  /* Initialize i2c device */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  ret = apds9960_init(i2c, bus);
  if (ret < 0)
    {
      snerr("Error initialize APDS9960.\n");
      return ret;
    }

  /* Register devices for each FIFOs at I2C bus */

  ret = apds9960_register(devpath, i2c, bus);
  if (ret < 0)
    {
      snerr("Error registering APDS9960.\n");
      return ret;
    }

  return ret;
}

#endif /* CONFIG_CXD56_I2C && CONFIG_APDS9960 */
