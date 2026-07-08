/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-m5-cardputer/src/esp32s3_bringup.c
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

#include <syslog.h>
#include <sys/types.h>
#include <debug.h>

#include <errno.h>
#include <nuttx/fs/fs.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#ifdef CONFIG_ESPRESSIF_WIFI
#  include "esp32s3_board_wlan.h"
#endif

#ifdef CONFIG_ESPRESSIF_BLE
#  include "esp32s3_ble.h"
#endif

#ifdef CONFIG_ESPRESSIF_WIFI_BT_COEXIST
#  include "esp32s3_wifi_adapter.h"
#endif

#ifdef CONFIG_ESPRESSIF_HR_TIMER
#  include "espressif/esp_hr_timer.h"
#endif

#ifdef CONFIG_ESP32S3_I2C
#  include "esp32s3_i2c.h"
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_LCD_DEV
#  include <nuttx/lcd/lcd_dev.h>
#endif

#ifdef CONFIG_ESP32S3_SPI
#  include "esp32s3_spi.h"
#  include "esp32s3_board_spidev.h"
#endif

#if defined(CONFIG_ESP32S3_SDMMC) || defined(CONFIG_MMCSD_SPI)
#  include "esp32s3_board_sdmmc.h"
#endif

#include "esp32s3-m5-cardputer.h"

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
 ****************************************************************************/

int esp32s3_bringup(void)
{
  int ret;

  syslog(LOG_INFO, "esp32s3_bringup(): start.\n");

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_HR_TIMER
  /* Initialize the high-resolution timer subsystem.  The Wi-Fi stack relies
   * on it (ets_timer / esp_timer) for scan and connection timeouts.
   */

  ret = esp_hr_timer_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_hr_timer_init() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32S3_SPI) && defined(CONFIG_SPI_DRIVER)
  /* Register the microSD SPI bus as a generic SPI character device */

#  ifdef CONFIG_ESP32S3_SPI2
  ret = board_spidev_initialize(ESP32S3_SPI2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev SPI2: %d\n", ret);
    }
#  endif

#  ifdef CONFIG_ESP32S3_SPI3
  ret = board_spidev_initialize(ESP32S3_SPI3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev SPI3: %d\n", ret);
    }
#  endif
#endif /* CONFIG_ESP32S3_SPI && CONFIG_SPI_DRIVER */

#ifdef CONFIG_I2C_DRIVER
  /* Configure the Grove I2C bus */

  ret = board_i2c_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2C driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_WIRELESS

#  ifdef CONFIG_ESPRESSIF_WIFI_BT_COEXIST
  ret = esp_wifi_bt_coexist_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Wi-Fi and BT coexist\n");
    }
#  endif

#  ifdef CONFIG_ESPRESSIF_BLE
  ret = esp32s3_ble_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize BLE\n");
    }
#  endif

#  ifdef CONFIG_ESPRESSIF_WIFI
  ret = board_wlan_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize wlan subsystem=%d\n",
             ret);
    }
#  endif

#endif /* CONFIG_ESPRESSIF_WIRELESS */

#if defined(CONFIG_VIDEO_FB)
  /* Initialize and register the framebuffer (backed by the ST7789 LCD) */

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

#ifdef CONFIG_LCD_DEV
  ret = lcddev_register(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lcddev_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_MMCSD_SPI
  /* Mount the microSD card connected to the SPI3 bus */

  ret = board_sdmmc_spi_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize microSD: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32S3_M5_CARDPUTER_KEYBOARD
  /* Register the Cardputer matrix keyboard */

  ret = esp32s3_kbd_initialize("/dev/kbd0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize keyboard: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  syslog(LOG_INFO, "esp32s3_bringup(): end.\n");

  UNUSED(ret);
  return OK;
}
