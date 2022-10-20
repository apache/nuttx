/****************************************************************************
 * boards/arm/cxd56xx/common/src/cxd56_i2cdev.c
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

#include "cxd56_i2c.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_i2cdev_initialize
 *
 * Description:
 *   Initialize and register i2c driver for the specified i2c port
 *
 ****************************************************************************/

int board_i2cdev_initialize(int port)
{
  int ret;
  struct i2c_master_s *i2c;

  _info("Initializing /dev/i2c%d..\n", port);

  /* Initialize i2c device */

  i2c = cxd56_i2cbus_initialize(port);
  if (!i2c)
    {
      _err("ERROR: Failed to initialize i2c%d.\n", port);
      return -ENODEV;
    }

  ret = i2c_register(i2c, port);
  if (ret < 0)
    {
      _err("ERROR: Failed to register i2c%d: %d\n", port, ret);
      cxd56_i2cbus_uninitialize(i2c);
    }

  return ret;
}
