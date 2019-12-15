/****************************************************************************
 * boards/arm/am335x/beaglebone-black/src/am335x_bringup.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>
#include <debug.h>

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
  FAR struct i2c_master_s *i2c;
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
  FAR struct can_dev_s *can;
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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
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

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
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
