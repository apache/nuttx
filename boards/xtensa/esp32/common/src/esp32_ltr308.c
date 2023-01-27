/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_ltr308.c
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
#include <nuttx/sensors/ltr308.h>
#include <nuttx/i2c/i2c_master.h>

#include "esp32_board_i2c.h"
#include "esp32_i2c.h"
#include "esp32_ltr308.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ltr308_initialize
 *
 * Description:
 *   Initialize and register the LTR308 Lite-On ambient light sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as
 *           /dev/uorb/sensor_lightN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ltr308_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing LTR308!\n");

  /* Initialize LTR308 */

  i2c = esp32_i2cbus_initialize(busno);
  if (i2c != NULL)
    {
      /* Then try to register the light sensor in one of the two I2C
       * available controllers.
       */

      ret = ltr308_register(devno, i2c);
      if (ret < 0)
        {
          snerr("ERROR: Error registering LTR308 in I2C%d\n", busno);
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}
