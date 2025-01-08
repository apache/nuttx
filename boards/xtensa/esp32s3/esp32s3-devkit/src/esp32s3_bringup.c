/****************************************************************************
 * boards/xtensa/esp32s3/esp32s3-devkit/src/esp32s3_bringup.c
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

#include <fcntl.h>
#include <unistd.h>
#include <syslog.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <debug.h>
#include <stdio.h>

#include <errno.h>
#include <nuttx/fs/fs.h>
#include <nuttx/himem/himem.h>
#include <arch/board/board.h>

#ifdef CONFIG_ESP32S3_TIMER
#  include "esp32s3_board_tim.h"
#endif

#ifdef CONFIG_ESP32S3_WIFI
#  include "esp32s3_board_wlan.h"
#endif

#ifdef CONFIG_ESP32S3_BLE
#  include "esp32s3_ble.h"
#endif

#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
#  include "esp32s3_wifi_adapter.h"
#endif

#ifdef CONFIG_ESP32S3_RT_TIMER
#  include "esp32s3_rt_timer.h"
#endif

#ifdef CONFIG_ESP32S3_I2C
#  include "esp32s3_i2c.h"
#endif

#ifdef CONFIG_ESP32S3_I2S
#  include "esp32s3_i2s.h"
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

#ifdef CONFIG_ESP32S3_PARTITION_TABLE
#  include "esp32s3_partition.h"
#endif

#ifdef CONFIG_ESP_RMT
#  include "esp32s3_board_rmt.h"
#endif

#ifdef CONFIG_ESP_MCPWM
#  include "esp32s3_board_mcpwm.h"
#endif

#ifdef CONFIG_ESP32S3_SPI
#include "esp32s3_spi.h"
#include "esp32s3_board_spidev.h"
#  ifdef CONFIG_ESPRESSIF_SPI_BITBANG
#    include "espressif/esp_spi_bitbang.h"
#  endif
#endif

#ifdef CONFIG_ESP32S3_SDMMC
#include "esp32s3_board_sdmmc.h"
#endif

#ifdef CONFIG_ESP32S3_AES_ACCELERATOR
#  include "esp32s3_aes.h"
#endif

#ifdef CONFIG_ESP32S3_ADC
#include "esp32s3_board_adc.h"
#endif

#ifdef CONFIG_ESPRESSIF_TEMP
#  include "espressif/esp_temperature_sensor.h"
#endif

#ifdef CONFIG_ESP_PCNT
#  include "espressif/esp_pcnt.h"
#  include "esp32s3_board_pcnt.h"
#endif

#ifdef CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL
#  include "espressif/esp_nxdiag.h"
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
#if (defined(CONFIG_ESP32S3_I2S0) && !defined(CONFIG_AUDIO_CS4344)) || \
    defined(CONFIG_ESP32S3_I2S1)
  bool i2s_enable_tx;
  bool i2s_enable_rx;
#endif

#if defined(CONFIG_ESP32S3_SPIRAM) && \
    defined(CONFIG_ESP32S3_SPIRAM_BANKSWITCH_ENABLE)
  ret = esp_himem_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init HIMEM: %d\n", ret);
    }
#endif

#if defined(CONFIG_ESP32S3_SPI) && defined(CONFIG_SPI_DRIVER)
  #ifdef CONFIG_ESP32S3_SPI2
  ret = board_spidev_initialize(ESP32S3_SPI2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev 2: %d\n", ret);
    }
  #endif

  #ifdef CONFIG_ESP32S3_SPI3
  ret = board_spidev_initialize(ESP32S3_SPI3);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev 3: %d\n", ret);
    }
  #endif

  #ifdef CONFIG_ESPRESSIF_SPI_BITBANG
  ret = board_spidev_initialize(ESPRESSIF_SPI_BITBANG);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to init spidev 3: %d\n", ret);
    }
  #endif /* CONFIG_ESPRESSIF_SPI_BITBANG */
#endif /* CONFIG_ESP32S3_SPI && CONFIG_SPI_DRIVER*/

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

#ifdef CONFIG_ESP32S3_SPIFLASH
  ret = board_spiflash_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI Flash\n");
    }
#endif

#ifdef CONFIG_ESP32S3_PARTITION_TABLE
  ret = esp32s3_partition_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize partition error=%d\n",
             ret);
    }
#endif

#ifdef CONFIG_ESP32S3_RT_TIMER
  ret = esp32s3_rt_timer_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize RT timer: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_RMT
  ret = board_rmt_txinitialize(RMT_TXCHANNEL, RMT_OUTPUT_PIN);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_rmt_txinitialize() failed: %d\n", ret);
    }

  ret = board_rmt_rxinitialize(RMT_RXCHANNEL, RMT_INPUT_PIN);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_rmt_txinitialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESPRESSIF_TEMP
  struct esp_temp_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG(10, 50);
  ret = esp_temperature_sensor_initialize(cfg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize temperature sensor driver: %d\n",
             ret);
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

#ifdef CONFIG_ESP32S3_TWAI

  /* Initialize TWAI and register the TWAI driver. */

  ret = esp32s3_twai_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp32s3_twai_setup failed: %d\n", ret);
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

#ifdef CONFIG_ESP32S3_I2S

#ifdef CONFIG_AUDIO_CS4344

  /* Configure CS4344 audio on I2S0 */

  ret = esp32s3_cs4344_initialize(ESP32S3_I2S0);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize CS4344 audio: %d\n", ret);
    }
#else

#ifdef CONFIG_ESP32S3_I2S0_TX
  i2s_enable_tx = true;
#else
  i2s_enable_tx = false;
#endif /* CONFIG_ESP32S3_I2S0_TX */

#ifdef CONFIG_ESP32S3_I2S0_RX
  i2s_enable_rx = true;
#else
  i2s_enable_rx = false;
#endif /* CONFIG_ESP32S3_I2S0_RX */

  /* Configure I2S generic audio on I2S0 */

  ret = board_i2sdev_initialize(ESP32S3_I2S0, i2s_enable_tx, i2s_enable_rx);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S0 driver: %d\n", ret);
    }
#endif /* CONFIG_AUDIO_CS4344 */

#ifdef CONFIG_ESP32S3_I2S1

#ifdef CONFIG_ESP32S3_I2S1_TX
  i2s_enable_tx = true;
#else
  i2s_enable_tx = false;
#endif /* CONFIG_ESP32S3_I2S1_TX */

#ifdef CONFIG_ESP32S3_I2S1_RX
  i2s_enable_rx = true;
#else
  i2s_enable_rx = false;
#endif /* CONFIG_ESP32S3_I2S1_RX */

  /* Configure I2S generic audio on I2S1 */

  ret = board_i2sdev_initialize(ESP32S3_I2S1, i2s_enable_tx, i2s_enable_rx);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize I2S%d driver: %d\n",
             CONFIG_ESP32S3_I2S1, ret);
    }

#endif /* CONFIG_ESP32S3_I2S1 */

#endif /* CONFIG_ESP32S3_I2S */

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

#ifdef CONFIG_ESP32S3_WIRELESS

#ifdef CONFIG_ESP32S3_WIFI_BT_COEXIST
  ret = esp32s3_wifi_bt_coexist_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Wi-Fi and BT coexist\n");
    }
#endif

#ifdef CONFIG_ESP32S3_BLE
  ret = esp32s3_ble_initialize();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize BLE\n");
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

#endif

#ifdef CONFIG_ESP32S3_OPENETH
  ret = esp_openeth_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize Open ETH ethernet.\n");
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = esp32s3_gpio_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
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

#ifdef CONFIG_NET_LAN9250
  ret = esp32s3_lan9250_initialize(LAN9250_SPI);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI ethernet LAN9250.\n");
    }
#endif

#ifdef CONFIG_ESP32S3_SDMMC
  ret = board_sdmmc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDMMC: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP32S3_AES_ACCELERATOR
  ret = esp32s3_aes_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize AES: %d\n", ret);
    }
#ifdef CONFIG_ESP32S3_AES_ACCELERATOR_TEST
  else
    {
      esp32s3_aes_test();
    }
#endif
#endif

#ifdef CONFIG_ESP32S3_ADC
  /* Configure ADC */

  ret = board_adc_init();
  if (ret)
    {
      syslog(LOG_ERR, "ERROR: board_adc_init() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_MCPWM_CAPTURE
  ret = board_capture_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_capture_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_MCPWM_MOTOR_BDC
  ret = board_motor_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_motor_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ESP_PCNT
  ret = board_pcnt_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_pcnt_initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SYSTEM_NXDIAG_ESPRESSIF_CHIP_WO_TOOL
  ret = esp_nxdiag_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: esp_nxdiag_initialize failed: %d\n", ret);
    }
#endif

  /* If we got here then perhaps not all initialization was successful, but
   * at least enough succeeded to bring-up NSH with perhaps reduced
   * capabilities.
   */

  UNUSED(ret);
  return OK;
}
