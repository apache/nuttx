/****************************************************************************
 * boards/arm64/am62x/beagleplay/src/beagleplay_bringup.c
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

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/i2c/i2c_master.h>

#include "am62x_gpio.h"
#include "am62x_i2c.h"
#include "am62x_tisci.h"
#include "beagleplay.h"

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_I2C_DRIVER) && \
    (defined(CONFIG_AM62X_I2C0) || defined(CONFIG_AM62X_I2C1))
static int am62x_i2cdev_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = am62x_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C%d\n", bus);
      return -ENODEV;
    }

  ret = i2c_register(i2c, bus);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register /dev/i2c%d: %d\n",
             bus, ret);
      am62x_i2cbus_uninitialize(i2c);
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: beagleplay_bringup
 *
 * Description:
 *   Bring up board features after the scheduler and device drivers are
 *   initialised.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure.  Non-fatal errors
 *   are logged but do not abort the boot.
 *
 ****************************************************************************/

int beagleplay_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_AM62X_TISCI
  /* Bring up the TISCI link first: peripheral power, clocks, resets, and
   * interrupt routing are requested from the DM/TIFS firmware through it.
   */

  ret = am62x_tisci_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize TISCI: %d\n", ret);
    }
#endif

#ifdef CONFIG_AM62X_GPIO
  ret = am62x_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize GPIO: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_AM62X_I2C0)
  ret = am62x_i2cdev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to bring up I2C0: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_AM62X_I2C1)
  ret = am62x_i2cdev_register(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to bring up I2C1: %d\n", ret);
    }
#endif

  return ret;
}
