/****************************************************************************
 * boards/arm/mx8mp/verdin-mx8mp/src/mx8mp_bringup.c
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
#include <nuttx/leds/userled.h>
#include <nuttx/input/buttons.h>
#include <debug.h>

#include "verdin-mx8mp.h"
#include "mx8mp_gpio.h"

#ifdef CONFIG_SENSORS_INA219
#  include "mx8mp_ina219.h"
#endif

#ifdef CONFIG_MX8MP_RPMSG
#  include <mx8mp_rptun.h>
#endif

#ifdef CONFIG_RPMSG_UART
#  include <nuttx/serial/uart_rpmsg.h>
#endif

#ifdef CONFIG_FS_PROCFS
#  include <nuttx/fs/fs.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init("netcore", "proxy", 4096, true);
}
#endif

/****************************************************************************
 * Name: mx8mp_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int mx8mp_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_MX8MP_RPMSG
  mx8mp_rptun_init("imx8mp-shmem", "netcore");
#endif /* CONFIG_MX8MP_RPMSG */

#if defined(CONFIG_USERLED) && !defined(CONFIG_ARCH_LEDS)
#ifdef CONFIG_USERLED_LOWER
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#else
  /* Enable USER LED support for some other purpose */

  board_userled_initialize();
#endif /* CONFIG_USERLED_LOWER */
#endif /* CONFIG_USERLED && !CONFIG_ARCH_LEDS */

#ifdef CONFIG_INPUT_BUTTONS
#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#else
  /* Enable BUTTON support for some other purpose */

  board_button_initialize();
#endif /* CONFIG_INPUT_BUTTONS_LOWER */
#endif /* CONFIG_INPUT_BUTTONS */

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_MX8MP_I2C_DRIVER
  /* Initialize I2C buses */

  ret = mx8mp_i2cdev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mx8mp_i2cdev_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_INA219
  /* Configure and initialize the INA219 sensor in I2C4 */

  ret = board_ina219_initialize(4);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mx8mp_ina219_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_MX8MP_SPI_DRIVER
  /* Initialize SPI buses */

  ret = mx8mp_spidev_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: spidev_initialize() failed: %d\n", ret);
    }
#endif

  return ret;
}
