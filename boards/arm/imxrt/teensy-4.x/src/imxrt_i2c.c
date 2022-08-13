/****************************************************************************
 * boards/arm/imxrt/teensy-4.x/src/imxrt_i2c.c
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

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmp280.h>
#include <imxrt_lpi2c.h>

#include "teensy-4.h"

#ifdef CONFIG_IMXRT_LPI2C

#define BMP280_LPI2C 3

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_i2c_register
 *
 * Description:
 *  Register I2C driver
 *
 ****************************************************************************/

#ifdef CONFIG_I2C_DRIVER
static void imxrt_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = imxrt_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      serr("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          serr("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          imxrt_i2cbus_uninitialize(i2c);
        }
    }

  #ifdef SENSORS_BMP280
  if (BMP280_LPI2C == bus)
    {
      /* Register the BMP280 driver */

      ret = bmp280_register(0, i2c);
      if (ret < 0)
        {
          serr("ERROR: Failed to register BMP280\n");
        }
    }
  #endif
}
#endif

/****************************************************************************
 * Name: imxrt_i2c_setup
 *
 * Description:
 *  Choose which I2C driver should be initialize
 *
 ****************************************************************************/

void imxrt_i2c_setup()
{
#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C1)
  imxrt_i2c_register(1);
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C2)
  serr("ERROR: LPI2C2 is not on Teensy-4.x board\n");
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C3)
  imxrt_i2c_register(3);
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C4)
  imxrt_i2c_register(4);
#endif
}

#endif /* CONFIG_IMXRT_LPI2C */
