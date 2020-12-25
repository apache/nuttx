/****************************************************************************
 * boards/arm/samd5e5/metro-m4/src/sam_bringup.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
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
#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include "metro-m4.h"

#if defined(CONFIG_SAMD5E5_WDT) && defined(CONFIG_WATCHDOG)
  #include "sam_wdt.h"
#endif

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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int sam_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             PROCFS_MOUNTPOINT, ret);
    }
#endif

#if defined(CONFIG_SAMD5E5_WDT) && defined(CONFIG_WATCHDOG)
  (void)sam_wdt_initialize(CONFIG_WATCHDOG_DEVPATH);
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

  ret = mount("/dev/smart0", "/mnt/nvm", "smartfs", 0, NULL);
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
