/****************************************************************************
 * boards/risc-v/esp32c3/esp32c3-devkit-rust-1/src/esp32c3_bringup.c
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

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <syslog.h>
#include <debug.h>
#include <stdio.h>

#include <nuttx/fs/fs.h>

#include "esp32c3_wlan.h"
#include "esp32c3_spiflash.h"
#include "esp32c3_partition.h"
#include "esp32c3-devkit-rust-1.h"

#include "esp32c3_rtc.h"
#ifdef CONFIG_ESP32C3_EFUSE
#  include "esp32c3_efuse.h"
#endif

#ifdef CONFIG_ESP32C3_SHA_ACCELERATOR
#  include "esp32c3_sha.h"
#endif

#ifdef CONFIG_RTC_DRIVER
#  include "esp32c3_rtc_lowerhalf.h"
#endif

#ifdef CONFIG_ESP32C3_BLE
#  include "esp32c3_ble.h"
#endif

#include "esp32c3_board_apds9960.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32c3_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y
 *     Called from the NSH library
 *
 ****************************************************************************/

int esp32c3_bringup(void)
{
  int ret;

#if defined(CONFIG_ESP32C3_EFUSE)
  ret = esp32c3_efuse_initialize("/dev/efuse");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init EFUSE: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32C3_SHA_ACCELERATOR) && \
    !defined(CONFIG_CRYPTO_CRYPTODEV_HARDWARE)
  ret = esp32c3_sha_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize SHA: %d\n", ret);
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

#ifdef CONFIG_FS_TMPFS
  /* Mount the tmpfs file system */

  ret = nx_mount(NULL, CONFIG_LIBC_TMPDIR, "tmpfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount tmpfs at %s: %d\n",
             CONFIG_LIBC_TMPDIR, ret);
    }
#endif

#ifdef CONFIG_RTC_DRIVER
  /* Instantiate the ESP32-C3 RTC driver */

  ret = esp32c3_rtc_driverinit();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to Instantiate the RTC driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_APDS9960
  /* Register the APDS-9960 gesture sensor */

  ret = board_apds9960_initialize(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_apds9960_initialize() failed: %d\n",
             ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
