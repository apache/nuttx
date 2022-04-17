/****************************************************************************
 * boards/arm/stm32/stm32f4discovery/src/stm32_bringup.c
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

#include <nuttx/fs/fs.h>

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include "stm32.h"
#include "stm32_romfs.h"

#ifdef CONFIG_STM32_OTGFS
#  include "stm32_usbhost.h"
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_RNDIS
#  include <nuttx/usb/rndis.h>
#endif

#include "stm32f4discovery.h"

/* Conditional logic in stm32f4discovery.h will determine if certain features
 * are supported.  Tests for these features need to be made after including
 * stm32f4discovery.h.
 */

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

/* The following are includes from board-common logic */

#ifdef CONFIG_SENSORS_BMP180
#include "stm32_bmp180.h"
#endif

#ifdef CONFIG_SENSORS_MS5611
#include "stm32_ms5611.h"
#endif

#ifdef CONFIG_SENSORS_MAX6675
#include "stm32_max6675.h"
#endif

#ifdef CONFIG_INPUT_NUNCHUCK
#include "stm32_nunchuck.h"
#endif

#ifdef CONFIG_SENSORS_ZEROCROSS
#include "stm32_zerocross.h"
#endif

#ifdef CONFIG_SENSORS_QENCODER
#include "board_qencoder.h"
#endif

#ifdef CONFIG_SENSORS_BH1750FVI
#include "stm32_bh1750.h"
#endif

#ifdef CONFIG_LIS3DSH
#include "stm32_lis3dsh.h"
#endif

#ifdef CONFIG_LCD_BACKPACK
#include "stm32_lcd_backpack.h"
#endif

#ifdef CONFIG_SENSORS_MAX31855
#include "stm32_max31855.h"
#endif

#ifdef CONFIG_SENSORS_MLX90614
#include "stm32_mlx90614.h"
#endif

#ifdef CONFIG_SENSORS_XEN1210
#include "stm32_xen1210.h"
#endif

#ifdef CONFIG_USBADB
#  include <nuttx/usb/adb.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void)
{
  stm32_i2c_register(1);
#if 0
  stm32_i2c_register(1);
  stm32_i2c_register(2);
#endif
}
#else
#  define stm32_i2ctool()
#endif

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
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
#endif
  int ret = OK;

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  stm32_i2ctool();
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

#ifdef CONFIG_SENSORS_MS5611
  /* Initialize the MS5611 pressure sensor. */

  ret = board_ms5611_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize MS5611, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_BH1750FVI
  ret = board_bh1750_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_bh1750initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_ZEROCROSS
  /* Configure the zero-crossing driver */

  board_zerocross_initialize(0);
#endif

#ifdef CONFIG_LEDS_MAX7219
  ret = stm32_max7219init("/dev/numdisp0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: max7219_leds_register failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LCD_ST7032
  ret = stm32_st7032init("/dev/slcd0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: st7032_register failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_RGBLED
  /* Configure the RGB LED driver */

  stm32_rgbled_setup();
#endif

#if defined(CONFIG_PCA9635PW)
  /* Initialize the PCA9635 chip */

  ret = stm32_pca9635_initialize();
  if (ret < 0)
    {
      serr("ERROR: stm32_pca9635_initialize failed: %d\n", ret);
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

#ifdef HAVE_SDIO
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret != OK)
    {
      ferr("ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_MMCSD_SPI
  /* Initialize the MMC/SD SPI driver (SPI2 is used) */

  ret = stm32_mmcsd_initialize(2, CONFIG_NSH_MMCSDMINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize SD slot %d: %d\n",
             CONFIG_NSH_MMCSDMINOR, ret);
    }
#endif

#ifdef HAVE_USBHOST
  /* Initialize USB host operation.  stm32_usbhost_initialize() starts a
   * thread will monitor for USB connection and disconnection events.
   */

  ret = stm32_usbhost_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      uerr("ERROR: Failed to start USB monitor: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_CAN_CHARDRIVER
  /* Initialize CAN and register the CAN driver. */

  ret = stm32_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_can_setup failed: %d\n", ret);
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
      syslog(LOG_ERR, "ERROR: nunchuck_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MLX90614
  ret = board_mlx90614_initialize(0, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize MLX90614, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER
  /* Initialize and register the qencoder driver */

  ret = board_qencoder_initialize(0, CONFIG_STM32F4DISCO_QETIMER);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
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

#ifdef CONFIG_RTC_DS1307
  ret = stm32_ds1307_init();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize DS1307 RTC driver: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32 lower-half RTC driver */

  lower = stm32_rtc_lowerhalf();
  if (!lower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, lower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef HAVE_CS43L22
  /* Configure CS43L22 audio */

  ret = stm32_cs43l22_initialize(1);
  if (ret != OK)
    {
      serr("Failed to initialize CS43L22 audio: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MAX31855
  /* Register device 0 on spi channel 2 */

  ret = board_max31855_initialize(0, 2);
  if (ret < 0)
    {
      serr("ERROR:  stm32_max31855initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MAX6675
  ret = board_max6675_initialize(0, 2);
  if (ret < 0)
    {
      serr("ERROR:  stm32_max6675initialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n",
           STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_STM32_ROMFS
  ret = stm32_romfs_initialize();
  if (ret < 0)
    {
      serr("ERROR: Failed to mount romfs at %s: %d\n",
           CONFIG_STM32_ROMFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_SENSORS_XEN1210
  ret = board_xen1210_initialize(0, 1);
  if (ret < 0)
    {
      serr("ERROR:  xen1210_archinitialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_LIS3DSH
  /* Create a lis3dsh driver instance fitting the chip built into
   * stm32f4discovery
   */

  ret = board_lis3dsh_initialize(0, 1);
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize LIS3DSH driver: %d\n", ret);
    }
#endif

#ifdef HAVE_HCIUART
  ret = hciuart_dev_initialize();
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize HCI UART driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_RNDIS)
  uint8_t mac[6];
  mac[0] = 0xa0; /* TODO */
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#ifdef CONFIG_WL_GS2200M
  ret = stm32_gs2200m_initialize("/dev/gs2200m", 3);
  if (ret < 0)
    {
      serr("ERROR: Failed to initialize GS2200M: %d\n", ret);
    }
#endif

#ifdef CONFIG_LPWAN_SX127X
  ret = stm32_lpwaninitialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize wireless driver:"
                      " %d\n", ret);
    }
#endif /* CONFIG_LPWAN_SX127X */

#ifdef CONFIG_USBADB
  usbdev_adb_initialize();
#endif

  return ret;
}
