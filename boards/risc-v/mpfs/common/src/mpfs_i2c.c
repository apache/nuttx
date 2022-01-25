/****************************************************************************
 * boards/risc-v/mpfs/common/src/mpfs_i2c.c
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

#include "mpfs_i2c.h"

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

int mpfs_board_i2c_init(void)
{
  int ret = OK;

#if defined(CONFIG_MPFS_I2C0) || defined(CONFIG_MPFS_I2C1)
#ifdef CONFIG_I2C_DRIVER
  int bus = 0;
#endif
  FAR struct i2c_master_s *i2c;
#endif

#ifdef CONFIG_MPFS_I2C0
  i2c = mpfs_i2cbus_initialize(0);

  if (i2c == NULL)
    {
      i2cerr("ERROR: Failed to init I2C0 interface\n");
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_register(i2c, bus++);
  if (ret < 0)
    {
      i2cerr("ERROR: Failed to register I2C0 driver: %d\n", ret);
      mpfs_i2cbus_uninitialize(i2c);
      return ret;
    }
#endif
#endif

#ifdef CONFIG_MPFS_I2C1
  i2c = mpfs_i2cbus_initialize(1);

  if (i2c == NULL)
    {
      i2cerr("ERROR: Failed to init I2C1 interface\n");
      return -ENODEV;
    }

#ifdef CONFIG_I2C_DRIVER
  ret = i2c_register(i2c, bus);
  if (ret < 0)
    {
      i2cerr("ERROR: Failed to register I2C1 driver: %d\n", ret);
      mpfs_i2cbus_uninitialize(i2c);
      return ret;
    }
#endif
#endif

  return ret;
}
