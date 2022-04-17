/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit/src/esp32c3_i2c.c
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

#include "esp32c3_i2c.h"
#include "esp32c3-devkit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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

#ifdef CONFIG_ESP32C3_I2C0
  struct i2c_master_s *i2c;

  i2c = esp32c3_i2cbus_initialize(0);

  if (i2c == NULL)
    {
      i2cerr("ERROR: Failed to get I2C0 interface\n");
      return -ENODEV;
    }

  ret = i2c_register(i2c, 0);
  if (ret < 0)
    {
      i2cerr("ERROR: Failed to register I2C0 driver: %d\n", ret);
      esp32c3_i2cbus_uninitialize(i2c);
    }
#endif

  return ret;
}

