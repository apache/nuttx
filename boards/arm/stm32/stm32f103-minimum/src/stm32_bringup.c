/****************************************************************************
 * boards/arm/stm32/stm32f103-minimum/src/stm32_bringup.c
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
#include <syslog.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/timers/oneshot.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include "stm32.h"

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#include "stm32f103_minimum.h"

/* Conditional logic in stm32f103_minimum.h will determine if certain
 * features are supported.  Tests for these features need to be made after
 * including stm32f103_minimum.h.
 */

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

/* The following are includes from board-common logic */

#ifdef CONFIG_SENSORS_BMP180
#include "stm32_bmp180.h"
#endif

#ifdef CONFIG_LEDS_APA102
#include "stm32_apa102.h"
#endif

#ifdef CONFIG_WS2812
#include "stm32_ws2812.h"
#endif

#ifdef CONFIG_SENSORS_MAX6675
#include "stm32_max6675.h"
#endif

#ifdef CONFIG_SENSORS_VEML6070
#include "stm32_veml6070.h"
#endif

#ifdef CONFIG_INPUT_NUNCHUCK
#include "stm32_nunchuck.h"
#endif

#ifdef CONFIG_AUDIO_TONE
#include "stm32_tone.h"
#endif

#ifdef CONFIG_SENSORS_LM75
#include "stm32_lm75.h"
#endif

#ifdef CONFIG_WL_NRF24L01
#include "stm32_nrf24l01.h"
#endif

#ifdef CONFIG_SENSORS_HCSR04
#include "stm32_hcsr04.h"
#endif

#ifdef CONFIG_SENSORS_APDS9960
#include "stm32_apds9960.h"
#endif

#ifdef CONFIG_SENSORS_ZEROCROSS
#include "stm32_zerocross.h"
#endif

#ifdef CONFIG_SENSORS_QENCODER
#include "board_qencoder.h"
#endif

#ifdef CONFIG_SENSORS_HYT271
# define HAVE_SENSORS_DEVICE
#endif

#ifdef CONFIG_SENSORS_DS18B20
# define HAVE_SENSORS_DEVICE
#endif

#ifdef CONFIG_LCD_BACKPACK
#include "stm32_lcd_backpack.h"
#endif

#ifdef CONFIG_USBADB
#include <nuttx/usb/adb.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Checking needed by W25 Flash */

#define HAVE_W25      1

/* Can't support the W25 device if it SPI1 or W25 support is not enabled */

#if !defined(CONFIG_STM32_SPI1) || !defined(CONFIG_MTD_W25)
#  undef HAVE_W25
#endif

/* Can't support W25 features if mountpoints are disabled */

#ifdef CONFIG_DISABLE_MOUNTPOINT
#  undef HAVE_W25
#endif

/* Default W25 minor number */

#if defined(HAVE_W25) && !defined(CONFIG_NSH_W25MINOR)
#  define CONFIG_NSH_W25MINOR 0
#endif

/* Checking needed by MMC/SDCard */

#ifdef CONFIG_NSH_MMCSDMINOR
#  define MMCSD_MINOR CONFIG_NSH_MMCSDMINOR
#else
#  define MMCSD_MINOR 0
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_SENSORS_DEVICE
static int g_sensor_devno;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
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

int stm32_bringup(void)
{
#ifdef CONFIG_ONESHOT
  struct oneshot_lowerhalf_s *os = NULL;
#endif
  int ret = OK;

#ifdef CONFIG_DEV_GPIO
  ret = stm32_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
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

#ifdef CONFIG_LCD_BACKPACK
  /* slcd:0, i2c:1, rows=2, cols=16 */

  ret = board_lcd_backpack_init(0, 1, 2, 16);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize PCF8574 LCD, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_ZEROCROSS
  /* Configure the zero-crossing driver */

  ret = board_zerocross_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize Zero-Cross, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_MMCSD
  ret = stm32_mmcsd_initialize(MMCSD_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot %d: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_BMP180
  /* Initialize the BMP180 pressure sensor. */

  ret = board_bmp180_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP180, error %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_W25
  /* Initialize and register the W25 FLASH file system. */

  ret = stm32_w25initialize(CONFIG_NSH_W25MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize W25 minor %d: %d\n",
             CONFIG_NSH_W25MINOR, ret);
      return ret;
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef HAVE_AT24
  /* Initialize the AT24 driver */

  ret = stm32_at24_automount(AT24_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_at24_automount() failed: %d\n", ret);
      return ret;
    }
#endif /* HAVE_AT24 */

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_AUDIO_TONE
  /* Configure and initialize the tone generator. */

  ret = board_tone_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_tone_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LEDS_APA102
  /* Configure and initialize the APA102 LED Strip. */

  ret = board_apa102_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_apa102_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_WS2812
  /* Configure and initialize the WS2812 LEDs. */

  ret = board_ws2812_initialize(0, WS2812_SPI, WS2812_NLEDS);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_ws2812_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_HYT271
  /* Configure and initialize the HYT271 sensors */

  ret = stm32_hyt271initialize(g_sensor_devno);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_hyt271initialize() failed: %d\n", ret);
    }
  else
    {
      g_sensor_devno += ret;
    }
#endif

#ifdef CONFIG_SENSORS_DS18B20
  /* Configure and initialize the DS18B20 sensors */

  ret = stm32_ds18b20initialize(g_sensor_devno);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_ds18b20initialize() failed: %d\n", ret);
    }
  else
    {
      g_sensor_devno += ret;
    }
#endif

#ifdef CONFIG_LM75_I2C
  /* Configure and initialize the LM75 sensor */

  ret = board_lm75_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_lm75_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RGBLED
  /* Configure and initialize the RGB LED. */

  ret = stm32_rgbled_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_rgbled_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_HCSR04
  /* Configure and initialize the HC-SR04 distance sensor */

  ret = board_hcsr04_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_hcsr04_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MAX6675
  ret = board_max6675_initialize(0, 1);
  if (ret < 0)
    {
      serr("ERROR:  board_max6675_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAN_MCP2515
  /* Configure and initialize the MCP2515 CAN device */

  ret = stm32_mcp2515initialize("/dev/can0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mcp2515initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CL_MFRC522
  ret = stm32_mfrc522initialize("/dev/rfid0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mfrc522initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ONESHOT
  os = oneshot_initialize(1, 10);
  if (os)
    {
      ret = oneshot_register("/dev/oneshot", os);
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

#ifdef CONFIG_INPUT_NUNCHUCK
  /* Register the Nunchuck driver */

  ret = board_nunchuck_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_nunchuck_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER
  /* Initialize and register the qencoder driver */

  ret = board_qencoder_initialize(0,
                                  CONFIG_STM32F103MINIMUM_QETIMER);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_APDS9960
  /* Register the APDS-9960 gesture sensor */

  ret = board_apds9960_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_apds9960_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_SENSORS_VEML6070
  /* Register the UV-A light sensor */

  ret = board_veml6070_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_veml6070_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_WL_NRF24L01)
  /* Initialize the NRF24L01 wireless module */

  ret = board_nrf24l01_initialize(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_nrf24l01_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_USBADB
  usbdev_adb_initialize();
#endif

  return ret;
}
