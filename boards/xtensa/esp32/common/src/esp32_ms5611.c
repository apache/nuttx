/****************************************************************************
 * boards/xtensa/esp32/common/src/esp32_ms5611.c
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
#include <nuttx/sensors/ms5611.h>
#include <nuttx/i2c/i2c_master.h>

#include "esp32_board_i2c.h"
#include "esp32_i2c.h"
#include "esp32_ms5611.h"

#ifdef CONFIG_I2CMULTIPLEXER_TCA9548A
#  include "esp32_tca9548a.h"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_ms5611_initialize
 *
 * Description:
 *   Initialize and register the MS5611 Pressure Sensor driver.
 *
 * Input Parameters:
 *   devno - The device number, used to build the device path as /dev/pressN
 *   busno - The I2C bus number
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int board_ms5611_initialize(int devno, int busno)
{
  struct i2c_master_s *i2c;
  int ret;

  sninfo("Initializing MS5611!\n");

  /* Initialize MS5611 */

#ifdef CONFIG_BOARD_HAVE_I2CMUX
  /* We will use the devno as channel to avoid modifying the
   * board_ms5611_initialize() function. If you connected your
   * MS5611 to another channel, pass it to the devno of the
   * board_ms5611_initialize() function.
   */

  i2c = esp32_i2cmux_getmaster(busno, devno);
#else
  i2c = esp32_i2cbus_initialize(busno);
#endif

  if (i2c != NULL)
    {
      /* Then try to register the barometer sensor in I2C0 */

      ret = ms5611_register(i2c, devno, MS5611_ADDR0);
      if (ret < 0)
        {
          snerr("ERROR: Error registering MS5611 in I2C%d\n", busno);
        }
    }
  else
    {
      ret = -ENODEV;
    }

  return ret;
}

