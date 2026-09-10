/****************************************************************************
 * boards/arm/am67/t3-gem-o1/src/am67_i2c.c
 *
 * SPDX-License-Identifier: Apache-2.0
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

#include <nuttx/i2c/i2c_master.h>
#include "am67_i2c.h"

#if defined(CONFIG_AM67_I2C0) || defined(CONFIG_AM67_WKUP_I2C0)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am67_i2cdev_initialize
 ****************************************************************************/

void am67_i2cdev_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  int ret;

#ifdef CONFIG_AM67_I2C0
  i2c = am67_i2cbus_initialize(0);
  if (i2c == NULL)
    {
      i2cerr("ERROR: Failed to initialize I2C0\n");
    }
  else
    {
      ret = i2c_register(i2c, 0);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register /dev/i2c0: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_AM67_WKUP_I2C0
  i2c = am67_i2cbus_initialize(2);
  if (i2c == NULL)
    {
      i2cerr("ERROR: Failed to initialize WKUP_I2C0\n");
    }
  else
    {
      ret = i2c_register(i2c, 2);
      if (ret < 0)
        {
          i2cerr("ERROR: Failed to register /dev/i2c2: %d\n", ret);
        }
    }
#endif
}

#endif /* CONFIG_AM67_I2C0 || CONFIG_AM67_WKUP_I2C0 */
