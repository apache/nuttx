/****************************************************************************
 * boards/arm/mx8mp/verdin-mx8mp/src/mx8mp_i2cdev.c
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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>

#include "arm_internal.h"
#include "chip.h"
#include "mx8mp_i2c.h"

#include <arch/board/board.h>

/****************************************************************************
 * Private Functions
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

  i2cinfo("Initializing /dev/i2c%d..\n", port);

  /* Initialize i2c device */

  i2c = mx8mp_i2cbus_initialize(port);
  if (!i2c)
    {
      i2cerr("ERROR: Failed to initialize i2c%d.\n", port);
      return -ENODEV;
    }

  ret = i2c_register(i2c, port);
  if (ret < 0)
    {
      i2cerr("ERROR: Failed to register i2c%d: %d\n", port, ret);
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mx8mp_i2cdev_initialize
 *
 * Description:
 *   Called to configure all i2c drivers
 *
 ****************************************************************************/

int mx8mp_i2cdev_initialize(void)
{
  int ret = 0;

#ifdef CONFIG_MX8MP_I2C1
  ret = board_i2cdev_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C1.\n");
      return ret;
    }
#endif

#ifdef CONFIG_MX8MP_I2C2
  ret = board_i2cdev_initialize(2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C2.\n");
      return ret;
    }
#endif

#ifdef CONFIG_MX8MP_I2C3
  ret = board_i2cdev_initialize(3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C3.\n");
      return ret;
    }
#endif

#ifdef CONFIG_MX8MP_I2C4
  ret = board_i2cdev_initialize(4);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C4.\n");
      return ret;
    }
#endif

#ifdef CONFIG_MX8MP_I2C5
  ret = board_i2cdev_initialize(5);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C5.\n");
      return ret;
    }
#endif

#ifdef CONFIG_MX8MP_I2C6
  ret = board_i2cdev_initialize(6);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C6.\n");
      return ret;
    }
#endif

  return ret;
}
