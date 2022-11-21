/****************************************************************************
 * boards/xtensa/esp32/esp32-sparrow-kit/src/esp32_bringup.c
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

#include <syslog.h>
#include <errno.h>
#include <nuttx/fs/fs.h>
#include <nuttx/himem/himem.h>

#include "esp32_partition.h"

#include <arch/board/board.h>

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_TIMER
#include <esp32_tim_lowerhalf.h>
#endif

#ifdef CONFIG_ONESHOT
#  include "esp32_board_oneshot.h"
#endif

#ifdef CONFIG_WATCHDOG
#  include "esp32_board_wdt.h"
#endif

#ifdef CONFIG_ESP32_SPIFLASH
#  include "esp32_board_spiflash.h"
#endif

#ifdef CONFIG_ESP32_BLE
#  include "esp32_ble.h"
#endif

#ifdef CONFIG_ESP32_WIFI
#  include "esp32_board_wlan.h"
#endif

#ifdef CONFIG_ESP32_I2C
#  include "esp32_board_i2c.h"
#endif

#ifdef CONFIG_SENSORS_BMP180
#  include "esp32_bmp180.h"
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_ESP32_RT_TIMER
#  include "esp32_rt_timer.h"
#endif

#ifdef CONFIG_LCD_DEV
#  include <nuttx/board.h>
#  include <nuttx/lcd/lcd_dev.h>
#endif

#ifdef CONFIG_RTC_DRIVER
#  include "esp32_rtc_lowerhalf.h"
#endif

#ifdef CONFIG_LCD_BACKPACK
#  include "esp32_lcd_backpack.h"
#endif

#include "esp32-sparrow-kit.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32_bringup
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

int esp32_bringup(void)
{
  int ret;

#if defined(CONFIG_ESP32_SPIRAM) && \
    defined(CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE)
  ret = esp_himem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init HIMEM: %d\n", ret);
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

#ifdef CONFIG_LCD_BACKPACK
  /* slcd:0, i2c:0, rows=2, cols=16 */

  ret = board_lcd_backpack_init(0, 0, 2, 16);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PCF8574 LCD, error %d\n", ret);
    }
#endif

#ifdef CONFIG_MMCSD
  ret = esp32_mmcsd_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_SPIFLASH
  ret = esp32_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
    }
#endif

#ifdef CONFIG_ESP32_PARTITION_TABLE
  ret = esp32_partition_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize partition error=%d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_RT_TIMER
  ret = esp32_rt_timer_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize RT timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_BLE
  ret = esp32_ble_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize BLE: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_WIFI
  ret = board_wlan_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize wireless subsystem=%d\n",
             ret);
    }
#endif

/* First, register the timer drivers and let timer 1 for oneshot
 * if it is enabled.
 */

#ifdef CONFIG_TIMER

#if defined(CONFIG_ESP32_TIMER0) && !defined(CONFIG_ESP32_RT_TIMER)
  ret = esp32_timer_initialize("/dev/timer0", TIMER0);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#if defined(CONFIG_ESP32_TIMER1) && !defined(CONFIG_ONESHOT)
  ret = esp32_timer_initialize("/dev/timer1", TIMER1);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_TIMER2
  ret = esp32_timer_initialize("/dev/timer2", TIMER2);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32_TIMER3
  ret = esp32_timer_initialize("/dev/timer3", TIMER3);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize timer driver: %d\n",
             ret);
    }
#endif

#endif /* CONFIG_TIMER */

  /* Now register one oneshot driver */

#if defined(CONFIG_ONESHOT) && defined(CONFIG_ESP32_TIMER1)

  ret = esp32_oneshot_init(ONESHOT_TIMER, ONESHOT_RESOLUTION_US);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp32_oneshot_init() failed: %d\n", ret);
    }

#endif /* CONFIG_ONESHOT */

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_WATCHDOG
  /* Configure watchdog timer */

  ret = board_wdt_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize watchdog drivers: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = esp32_gpio_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_I2C_DRIVER

#ifdef CONFIG_ESP32_I2C0
  ret = esp32_i2c_register(0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C Driver for I2C0: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_I2C1
  ret = esp32_i2c_register(1);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C Driver for I2C1: %d\n", ret);
    }
#endif

#endif

#ifdef CONFIG_SENSORS_BMP180
  /* Try to register BMP180 device in I2C0 */

  ret = board_bmp180_initialize(0, 0);

  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP180 driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Frame Buffer Driver.\n");
    }
#endif

#ifdef CONFIG_LCD_DEV
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_lcd_initialize() failed: %d\n", ret);
    }

  ret = lcddev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lcddev_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RTC_DRIVER
  /* Instantiate the ESP32 RTC driver */

  ret = esp32_rtc_driverinit();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to Instantiate the RTC driver: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
