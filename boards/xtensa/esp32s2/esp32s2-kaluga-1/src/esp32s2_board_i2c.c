/****************************************************************************
 * boards/xtensa/esp32s2/esp32s2-kaluga-1/src/esp32s2_board_i2c.c
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

#include <debug.h>
#include <errno.h>
#include <sys/types.h>

#include <nuttx/i2c/i2c_master.h>

#include "esp32s2_i2c.h"
#include "esp32s2-kaluga-1.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static int i2c_driver_init(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = esp32s2_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      i2cerr("Failed to get I2C%d interface\n", bus);
      return -ENODEV;
    }

  ret = i2c_register(i2c, bus);
  if (ret < 0)
    {
      i2cerr("Failed to register I2C%d driver: %d\n", bus, ret);
      esp32s2_i2cbus_uninitialize(i2c);
    }

  return ret;
}

/****************************************************************************
 * Name: board_i2c_init
 *
 * Description:
 *   Configure the I2C driver.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; A negated errno value is returned
 *   to indicate the nature of any failure.
 *
 ****************************************************************************/

int board_i2c_init(void)
{
  int ret = OK;

#ifdef CONFIG_ESP32S2_I2C0
  ret = i2c_driver_init(ESP32S2_I2C0);
  if (ret != OK)
    {
      goto done;
    }
#endif

#ifdef CONFIG_ESP32S2_I2C1
  ret = i2c_driver_init(ESP32S2_I2C1);
#endif

done:
  return ret;
}

