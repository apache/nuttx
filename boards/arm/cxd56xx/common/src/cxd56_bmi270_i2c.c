/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_bmi270_i2c.c
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
#include <nuttx/sensors/bmi270.h>

#include "cxd56_i2c.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BMI270_I2C_ADDRESS  (0x69)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_bmi270_initialize(int bus)
{
  int ret;
  struct i2c_master_s *i2c;

  sninfo("Initializing BMI270..\n");

  /* Initialize i2c device */

  i2c = cxd56_i2cbus_initialize(bus);
  if (!i2c)
    {
      snerr("ERROR: Failed to initialize i2c%d.\n", bus);
      return -ENODEV;
    }

  ret = bmi270_register("/dev/accel0", i2c, BMI270_I2C_ADDRESS);
  if (ret < 0)
    {
      snerr("Error registering BMI270\n");
    }

  return ret;
}
