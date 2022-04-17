/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_bringup.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include "metro-m4.h"

#if defined(CONFIG_SAMD5E5_WDT) && defined(CONFIG_WATCHDOG)
  #include "sam_wdt.h"
#endif

#include <nuttx/fs/fs.h>

#ifdef CONFIG_SAMD5E5_SERCOM5_ISI2C
  #include <nuttx/i2c/i2c_master.h>
  #include "hardware/sam_i2c_master.h"
#endif

#ifdef CONFIG_USBHOST
#  include "sam_usbhost.h"
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_BQ27426
  #include <nuttx/power/battery_gauge.h>
  #include <nuttx/power/battery_ioctl.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PROCFS_MOUNTPOINT "/proc"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
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

int sam_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             PROCFS_MOUNTPOINT, ret);
    }
#endif

#if defined(CONFIG_SAMD5E5_WDT) && defined(CONFIG_WATCHDOG)
  sam_wdt_initialize(CONFIG_WATCHDOG_DEVPATH);
#endif

#ifdef CONFIG_SAMD5E5_SERCOM5_ISI2C
  /* Initialize I2C bus */

  ret = metro_m4_i2cdev_initialize();
#endif

#ifdef CONFIG_USBHOST
  /* Initialize USB host operation. samd_usbhost_initialize() starts a
   * thread will monitor for USB connection and disconnection events.
   */

  ret = samd_usbhost_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "samd_usbhost_initialize failed %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "usbmonitor_start failed %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_FS_SMARTFS
  /* Initialize Smart File System (SMARTFS) */

  ret = sam_smartfs_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount smartfs  %d\n", ret);
      return ret;
    }

  /* Mount the file system at /mnt/nvm */

  ret = nx_mount("/dev/smart0", "/mnt/nvm", "smartfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the SmartFS volume: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_FS_AUTOMOUNTER
  /* Initialize the auto-mounter */

  sam_automount_initialize();
#endif

#ifdef CONFIG_BQ27426
  /* Configure and initialize the BQ2426 distance sensor */

  ret = sam_bq27426_initialize("/dev/batt1");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_bq27426_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = sam_gpio_initialize();
#endif

  UNUSED(ret);
  return OK;
}
