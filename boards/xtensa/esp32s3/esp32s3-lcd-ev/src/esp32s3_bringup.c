/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-lcd-ev/src/esp32s3_bringup.c
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

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <debug.h>

#include <errno.h>
#include <nuttx/fs/fs.h>
#include <arch/board/board.h>

#include "esp32s3_gpio.h"

#ifdef CONFIG_ESP32S3_TIMER
#  include "esp32s3_board_tim.h"
#endif

#ifdef CONFIG_ESPRESSIF_WLAN
#  include "esp32s3_board_wlan.h"
#endif

#ifdef CONFIG_ESPRESSIF_BLE
#  include "esp32s3_ble.h"
#endif

#ifdef CONFIG_ESPRESSIF_WIFI_BT_COEXIST
#  include "esp32s3_wifi_adapter.h"
#endif

#ifdef CONFIG_ESP32S3_I2C
#  include "esp32s3_i2c.h"
#endif

#ifdef CONFIG_ESPRESSIF_I2S
#  include "espressif/esp_i2s.h"
#endif

#ifdef CONFIG_ESP32S3_RT_TIMER
#  include "esp32s3_rt_timer.h"
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

#ifdef CONFIG_ESP32S3_EFUSE
#  include "esp32s3_efuse.h"
#endif

#ifdef CONFIG_ESP32S3_SPI
#  include "esp32s3_spi.h"
#endif

#include "esp32s3-lcd-ev.h"

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
#if (defined(CONFIG_ESPRESSIF_I2S0) && !defined(CONFIG_AUDIO_CS4344) && \
     !defined(CONFIG_AUDIO_ES8311)) || defined(CONFIG_ESPRESSIF_I2S1)
  bool i2s_enable_tx;
  bool i2s_enable_rx;
#endif

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

#ifdef CONFIG_ESPRESSIF_I2S
#  ifdef CONFIG_ESPRESSIF_I2S0
#    ifdef CONFIG_AUDIO_ES8311

  /* Configure ES8311 audio on I2C0 and I2S0 */

  esp32s3_configgpio(SPEAKER_ENABLE_GPIO, OUTPUT);
  esp32s3_gpiowrite(SPEAKER_ENABLE_GPIO, true);

  ret = esp32s3_es8311_initialize(ESP32S3_I2C0, ES8311_I2C_ADDR,
                                  ES8311_I2C_FREQ, ESP32S3_I2S0);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize ES8311 audio: %d\n", ret);
    }

#    else
#      ifdef CONFIG_ESPRESSIF_I2S0_TX
  i2s_enable_tx = true;
#      else
  i2s_enable_tx = false;
#      endif /* CONFIG_ESPRESSIF_I2S0_TX */

#      ifdef CONFIG_ESPRESSIF_I2S0_RX
  i2s_enable_rx = true;
#      else
  i2s_enable_rx = false;
#      endif /* CONFIG_ESPRESSIF_I2S0_RX */

  /* Configure I2S generic audio on I2S0 */

  ret = board_i2sdev_initialize(ESP32S3_I2S0, i2s_enable_tx, i2s_enable_rx);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S0 driver: %d\n", ret);
    }

#    endif /* CONFIG_AUDIO_ES8311 */
#  endif /* CONFIG_ESPRESSIF_I2S0 */

#  ifdef CONFIG_ESPRESSIF_I2S1
#    ifdef CONFIG_ESPRESSIF_I2S1_TX
  i2s_enable_tx = true;
#    else
  i2s_enable_tx = false;
#    endif /* CONFIG_ESPRESSIF_I2S1_TX */

#    ifdef CONFIG_ESPRESSIF_I2S1_RX
  i2s_enable_rx = true;
#    else
  i2s_enable_rx = false;
#    endif /* CONFIG_ESPRESSIF_I2S1_RX */

  /* Configure I2S generic audio on I2S1 */

  ret = board_i2sdev_initialize(ESP32S3_I2S1, i2s_enable_tx, i2s_enable_rx);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S%d driver: %d\n",
             CONFIG_ESPRESSIF_I2S1, ret);
    }

#  endif /* CONFIG_ESPRESSIF_I2S1 */
#endif /* CONFIG_ESPRESSIF_I2S */

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize button driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32S3_SPIFLASH
  ret = board_spiflash_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
    }
#endif

#ifdef CONFIG_ESPRESSIF_WIRELESS

#ifdef CONFIG_ESPRESSIF_WIFI_BT_COEXIST
  ret = esp32s3_wifi_bt_coexist_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Wi-Fi and BT coexist\n");
    }
#endif

#ifdef CONFIG_ESPRESSIF_BLE
  ret = esp32s3_ble_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize BLE\n");
    }
#endif

#ifdef CONFIG_ESPRESSIF_WLAN
  ret = board_wlan_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize wlan subsystem=%d\n",
             ret);
    }
#endif

#endif

#if defined(CONFIG_WS2812) && !defined(CONFIG_WS2812_NON_SPI_DRIVER)
  ret = board_ws2812_initialize(0, ESP32S3_SPI2, CONFIG_WS2812_LED_COUNT);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize ws2812 driver\n");
    }
#endif

#ifdef CONFIG_ESP32S3_BOARD_LCD
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize LCD driver\n");
    }
#endif

#ifdef CONFIG_ESP32S3_BOARD_TOUCHPAD
  ret = board_touchscreen_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize touchscreen driver\n");
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
