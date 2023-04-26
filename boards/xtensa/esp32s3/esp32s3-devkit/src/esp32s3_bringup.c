/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_bringup.c
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

#include <errno.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_ESP32S3_TIMER
#  include "esp32s3_board_tim.h"
#endif

#ifdef CONFIG_ESP32S3_WIFI
#  include "esp32s3_board_wlan.h"
#endif

#ifdef CONFIG_ESP32S3_RT_TIMER
#  include "esp32s3_rt_timer.h"
#endif

#ifdef CONFIG_ESP32S3_I2C
#  include "esp32s3_i2c.h"
#endif

#ifdef CONFIG_WATCHDOG
#  include "esp32s3_board_wdt.h"
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_RTC_DRIVER
#  include "esp32s3_rtc_lowerhalf.h"
#endif

#ifdef CONFIG_VIDEO_FB
#include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_ESP32S3_EFUSE
#  include "esp32s3_efuse.h"
#endif

#ifdef CONFIG_ESP32S3_LEDC
#  include "esp32s3_ledc.h"
#endif

#include "esp32s3-devkit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_bringup
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

int esp32s3_bringup(void)
{
  int ret;

#if defined(CONFIG_ESP32S3_EFUSE)
  ret = esp32s3_efuse_initialize("/dev/efuse");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init EFUSE: %d\n", ret);
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

#ifdef CONFIG_ESP32S3_LEDC
  ret = esp32s3_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp32s3_pwm_setup() failed: %d\n", ret);
    }
#endif /* CONFIG_ESP32S3_LEDC */

#ifdef CONFIG_ESP32S3_TIMER
  /* Configure general purpose timers */

  ret = board_tim_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize timers: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32S3_RT_TIMER
  ret = esp32s3_rt_timer_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize RT timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_RTC_DRIVER
  /* Instantiate the ESP32-S3 RTC driver */

  ret = esp32s3_rtc_driverinit();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to Instantiate the RTC driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_WATCHDOG
  /* Configure watchdog timer */

  ret = board_wdt_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize watchdog timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_I2C_DRIVER
  /* Configure I2C peripheral interfaces */

  ret = board_i2c_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_BMP180
  /* Try to register BMP180 device in I2C0 */

  ret = board_bmp180_initialize(0, ESP32S3_I2C0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "Failed to initialize BMP180 driver for I2C0: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize button driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_DJOYSTICK
  ret = esp32s3_djoy_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to register djoystick driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32S3_SPIFLASH
  ret = board_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
    }
#endif

#ifdef CONFIG_ESP32S3_WIFI
  ret = board_wlan_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize wireless subsystem=%d\n",
             ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Frame Buffer Driver.\n");
    }
#elif defined(CONFIG_LCD)
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize LCD.\n");
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
