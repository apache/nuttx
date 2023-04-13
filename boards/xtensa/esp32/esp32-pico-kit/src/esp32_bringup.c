/****************************************************************************
 * boards/xtensa/esp32/esp32-pico-kit/src/esp32_bringup.c
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
#if defined(CONFIG_ESP32_EFUSE)
#include <nuttx/efuse/efuse.h>
#endif
#include <nuttx/fs/fs.h>
#include <nuttx/himem/himem.h>

#if defined(CONFIG_ESP32_EFUSE)
#include "esp32_efuse.h"
#endif
#include "esp32_partition.h"

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

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
#  include "esp32_wifi_adapter.h"
#endif

#ifdef CONFIG_ESP32_I2C
#  include "esp32_board_i2c.h"
#endif

#ifdef CONFIG_ESP32_I2S
#  include "esp32_i2s.h"
#endif

#ifdef CONFIG_ESP32_AES_ACCELERATOR
#  include "esp32_aes.h"
#endif

#ifdef CONFIG_ESP32_RT_TIMER
#  include "esp32_rt_timer.h"
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_RTC_DRIVER
#  include "esp32_rtc_lowerhalf.h"
#endif

#ifdef CONFIG_SPI_DRIVER
#  include "esp32_spi.h"
#endif

#include "esp32-pico-kit.h"

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

#ifdef CONFIG_ESP32_AES_ACCELERATOR
  ret = esp32_aes_init();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize AES: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32_SPIRAM) && \
    defined(CONFIG_ESP32_SPIRAM_BANKSWITCH_ENABLE)
  ret = esp_himem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init HIMEM: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32_EFUSE)
  ret = esp32_efuse_initialize("/dev/efuse");
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

#ifdef CONFIG_LCD_BACKPACK
  /* slcd:0, i2c:0, rows=2, cols=16 */

  ret = board_lcd_backpack_init(0, 0, 2, 16);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PCF8574 LCD, error %d\n", ret);
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

#ifdef CONFIG_ESP32_LEDC
  ret = esp32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp32_pwm_setup() failed: %d\n", ret);
    }
#endif /* CONFIG_ESP32_LEDC */

#ifdef CONFIG_ESP32_TWAI

  /* Initialize TWAI and register the TWAI driver. */

  ret = esp32_twai_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp32_twai_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_RT_TIMER
  ret = esp32_rt_timer_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize RT timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32_WIFI_BT_COEXIST
  ret = esp32_wifi_bt_coexist_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Wi-Fi and BT "
             "coexistence support\n");
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

#ifdef CONFIG_ESP32_I2S

#if defined(CONFIG_ESP32_I2S0) && || defined(CONFIG_ESP32_I2S1)
  bool i2s_enable_tx;
  bool i2s_enable_rx;
#endif

#ifdef CONFIG_ESP32_I2S0

  /* Configure I2S0 */

#ifdef CONFIG_ESP32_I2S0_TX
  i2s_enable_tx = true;
#else
  i2s_enable_tx = false;
#endif /* CONFIG_ESP32_I2S0_TX */

#ifdef CONFIG_ESP32_I2S0_RX
    i2s_enable_rx = true;
#else
    i2s_enable_rx = false;
#endif /* CONFIG_ESP32_I2S0_RX */

  /* Configure I2S generic audio on I2S0 */

  ret = board_i2sdev_initialize(ESP32_I2S0, i2s_enable_tx, i2s_enable_rx);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S%d driver: %d\n",
             CONFIG_ESP32_I2S0, ret);
    }

#endif  /* CONFIG_ESP32_I2S0 */

#ifdef CONFIG_ESP32_I2S1

#ifdef CONFIG_ESP32_I2S1_TX
  i2s_enable_tx = true;
#else
  i2s_enable_tx = false;
#endif /* CONFIG_ESP32_I2S1_TX */

#ifdef CONFIG_ESP32_I2S1_RX
    i2s_enable_rx = true;
#else
    i2s_enable_rx = false;
#endif /* CONFIG_ESP32_I2S1_RX */

  /* Configure I2S generic audio on I2S1 */

  ret = board_i2sdev_initialize(ESP32_I2S1, i2s_enable_tx, i2s_enable_rx);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S%d driver: %d\n",
             CONFIG_ESP32_I2S1, ret);
    }

#endif  /* CONFIG_ESP32_I2S1 */

#endif /* CONFIG_ESP32_I2S */

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
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

#ifdef CONFIG_SPI_DRIVER
#  ifdef CONFIG_ESP32_SPI2
  ret = board_spidev_initialize(ESP32_SPI2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SPI%d driver: %d\n",
             ESP32_SPI2, ret);
    }
#  endif
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
