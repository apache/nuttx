/****************************************************************************
 * boards/arm/kinetis/freedom-k64f/src/k64_i2c.c
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

#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>

#include "kinetis_i2c.h"

#if defined(CONFIG_KINETIS_I2C0)

#  if defined(CONFIG_SENSORS_FXOS8700CQ)
#    include "nuttx/sensors/fxos8700cq.h"
#  endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_KINETIS_I2C0
struct i2c_master_s * g_i2c0_dev;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k64_i2cdev_initialize
 *
 * Description:
 *   Called to configure I2C
 *
 ****************************************************************************/

int k64_i2cdev_initialize(void)
{
  int ret = OK;

#ifdef CONFIG_KINETIS_I2C0
  g_i2c0_dev = kinetis_i2cbus_initialize(0);
  if (g_i2c0_dev == NULL)
    {
      syslog(LOG_ERR, "ERROR: kinetis_i2cbus_initialize(0) failed: %d\n",
             ret);
      ret = -ENODEV;
    }
  else
    {
#ifdef CONFIG_I2C_DRIVER
      ret = i2c_register(g_i2c0_dev, 0);
#if defined(CONFIG_SENSORS_FXOS8700CQ)
      fxos8700cq_register("/dev/accel0", g_i2c0_dev);
#endif
#endif
    }
#endif

  return ret;
}

#endif /* CONFIG_KINETIS_I2C0 */
