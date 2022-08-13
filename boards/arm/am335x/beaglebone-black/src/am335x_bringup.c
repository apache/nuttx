/****************************************************************************
 * boards/arm/am335x/beaglebone-black/src/am335x_bringup.c
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

#include <nuttx/fs/fs.h>

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#include "beaglebone-black.h"

#include "am335x_i2c.h"
#include "am335x_can.h"

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_I2C_DRIVER) && (defined(CONFIG_AM335X_I2C0) || \
    defined(CONFIG_AM335X_I2C1) || defined(CONFIG_AM335X_I2C2))
static void am335x_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = am335x_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "Failed to register I2C%d driver: %d\n", bus, ret);
          am335x_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

#if defined(CONFIG_CAN) && (defined(CONFIG_AM335X_CAN0) || defined(CONFIG_AM335X_CAN1))
static void am335x_can_register(void)
{
  struct can_dev_s *can;
  int ret;

#ifdef CONFIG_AM335X_CAN0
  can = am335x_can_initialize(0);
  if (can == NULL)
    {
      syslog(LOG_ERR, "Failed to get DCAN0 interface\n");
    }
  else
    {
      ret = can_register("/dev/can0", can);
      if (ret < 0)
        {
          syslog(LOG_ERR, "can_register failed: %d\n", ret);
          am335x_can_uninitialize(can);
        }
    }
#endif

#ifdef CONFIG_AM335X_CAN1
  can = am335x_can_initialize(1);
  if (can == NULL)
    {
      syslog(LOG_ERR, "Failed to get DCAN1 interface\n");
    }
  else
    {
#ifdef CONFIG_AM335X_CAN0
      ret = can_register("/dev/can1", can);
#else
      ret = can_register("/dev/can0", can);
#endif
      if (ret < 0)
        {
          syslog(LOG_ERR, "can_register failed: %d\n", ret);
          am335x_can_uninitialize(can);
        }
    }
#endif
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: am335x_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int am335x_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_AM335X_I2C0)
  am335x_i2c_register(0);
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_AM335X_I2C1)
  am335x_i2c_register(1);
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_AM335X_I2C2)
  am335x_i2c_register(2);
#endif

#if defined(CONFIG_CAN) && (defined(CONFIG_AM335X_CAN0) || defined(CONFIG_AM335X_CAN1))
  am335x_can_register();
#endif

  UNUSED(ret);
  return ret;
}
