/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_i2cdev_bitbang.c
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
#include <nuttx/i2c/i2c_master.h>

#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include "cxd56_i2c_bitbang.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_i2cdev_bitbang_initialize
 *
 * Description:
 *   Initialize i2c bitbang driver and register as the /dev/i2c device.
 *
 * Input Parameters:
 *   sda_pin - The pin number used as I2C SDA signal
 *   scl_pin - The pin number used as I2C SCL signal
 *
 * Returned Value:
 *   OK on success; Negated errno on failure.
 *
 ****************************************************************************/

int board_i2cdev_bitbang_initialize(uint32_t sda_pin, uint32_t scl_pin)
{
  int ret = 0;
  struct i2c_master_s *i2c;
  int port;

  /* Use a sda pin number as port number */

  port = sda_pin;

  _info("Initializing /dev/i2c%d..\n", port);

  /* Initialize i2c bitbang device */

  i2c = cxd56_i2c_bitbang_initialize(sda_pin, scl_pin);
  if (!i2c)
    {
      _err("ERROR: Failed to initialize i2c%d.\n", port);
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_register(i2c, port);
  if (ret < 0)
    {
      _err("ERROR: Failed to register i2c%d: %d\n", port, ret);
    }
#endif

  return ret;
}
