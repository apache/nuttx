/****************************************************************************
 * boards/arm/imxrt/arcx-socket-grid/src/imxrt_bringup.c
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
#include <debug.h>

#include <syslog.h>
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/video/fb.h>
#include <imxrt_lpi2c.h>
#include <imxrt_lpspi.h>

#ifdef CONFIG_IMXRT_USDHC
#  include "imxrt_usdhc.h"
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include "arcx-socket-grid.h"

#include <arch/board/board.h>  /* Must always be included last */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C)
static void imxrt_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = imxrt_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      serr("ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          serr("ERROR: Failed to register I2C%d driver: %d\n", bus, ret);
          imxrt_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: imxrt_bringup
 *
 * Description:
 *   Bring up board features
 *
 ****************************************************************************/

int imxrt_bringup(void)
{
  int ret;

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C_DRIVER) && defined(CONFIG_IMXRT_LPI2C1)
  imxrt_i2c_register(1);
#endif

#ifdef CONFIG_MMCSD_SPI
  /* Initialize SPI-based MMC/SD card support */

  imxrt_spidev_initialize();

  ret = imxrt_mmcsd_spi_initialize(MMCSD_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD slot %d: %d\n", ret);
    }
#endif

#if defined(CONFIG_IMXRT_USBOTG1) && defined(CONFIG_USBHOST)
  ret = imxrt_usbhost_initialize(0);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB1 Host: %d\n", ret);
      return ret;
    }
#endif

#if defined(CONFIG_IMXRT_USBOTG2) && defined(CONFIG_USBHOST)
  ret = imxrt_usbhost_initialize(1);
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB2 Host: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = imxrt_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

  UNUSED(ret);
  return OK;
}
