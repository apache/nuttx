/****************************************************************************
 * boards/arm/stm32h7/linum-stm32h753bi/src/stm32_bringup.c
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

#include <sys/types.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>

#include "stm32_gpio.h"
#include "stm32_i2c.h"

#include "linum-stm32h753bi.h"

#ifdef CONFIG_USERLED
#include <nuttx/leds/userled.h>
#endif

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

#ifdef CONFIG_STM32H7_FDCAN
#include "stm32_fdcan_sock.h"
#endif

#ifdef CONFIG_RNDIS
#include <nuttx/usb/rndis.h>
#endif

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#include <arch/board/board.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void convert_lcd_rgb565(void)
{
  /* Put LCD_{R0,R1,R2,G0,G1,B0,B1,B2} in low level */

  stm32_configgpio(GPIO_LTDC_R0);
  stm32_gpiowrite(GPIO_LTDC_R0, 0);
  stm32_configgpio(GPIO_LTDC_R1);
  stm32_gpiowrite(GPIO_LTDC_R1, 0);
  stm32_configgpio(GPIO_LTDC_R2);
  stm32_gpiowrite(GPIO_LTDC_R2, 0);

  stm32_configgpio(GPIO_LTDC_G0);
  stm32_gpiowrite(GPIO_LTDC_G0, 0);
  stm32_configgpio(GPIO_LTDC_G1);
  stm32_gpiowrite(GPIO_LTDC_G1, 0);

  stm32_configgpio(GPIO_LTDC_B0);
  stm32_gpiowrite(GPIO_LTDC_B0, 0);
  stm32_configgpio(GPIO_LTDC_B1);
  stm32_gpiowrite(GPIO_LTDC_B1, 0);
  stm32_configgpio(GPIO_LTDC_B2);
  stm32_gpiowrite(GPIO_LTDC_B2, 0);
}

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
#ifdef CONFIG_STM32H7_I2C3
  stm32_i2c_register(3);
#endif
#ifdef CONFIG_STM32H7_I2C4
  stm32_i2c_register(4);
#endif
}
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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y &&
 *   CONFIG_NSH_ARCHINIT:
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

  UNUSED(ret);

#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
#endif

#ifdef CONFIG_STM32H7_RMII
  /* Reset Ethernet PHY */

  stm32_configgpio(GPIO_ETH_RESET);
  stm32_gpiowrite(GPIO_ETH_RESET, 0);
  usleep(50000);
  stm32_gpiowrite(GPIO_ETH_RESET, 1);
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n",  ret);
    }
#endif /* CONFIG_FS_PROCFS */

  /* Put pin not used in RG565 to level zero */

  convert_lcd_rgb565();

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
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

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32 lower-half RTC driver */

  lower = stm32_rtc_lowerhalf();
  if (!lower)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to instantiate the RTC lower-half driver\n");
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
          syslog(LOG_ERR,
                 "ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#if defined(CONFIG_FAT_DMAMEMORY)
  if (stm32_dma_alloc_init() < 0)
    {
      syslog(LOG_ERR, "DMA alloc FAILED");
    }
#endif

#ifdef HAVE_SDIO
  /* Initialize the SDIO block driver */

  ret = stm32_sdio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
    }
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  stm32_i2ctool();
#endif

#ifdef CONFIG_I2C_EE_24XX
  ret = stm32_at24_init("/dev/eeprom");
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize EEPROM HX24LCXXB: %d\n", ret);
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

#ifdef CONFIG_AUDIO_TONE
  /* Configure and initialize the tone generator. */

  ret = board_tone_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: board_tone_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_NETDEV_LATEINIT

#  ifdef CONFIG_STM32H7_FDCAN1

  /* Enable and configure CAN1 */

  stm32_configgpio(GPIO_CAN1_STD);
  stm32_gpiowrite(GPIO_CAN1_STD, false);
  stm32_fdcansockinitialize(0);
#  endif

#  ifdef CONFIG_STM32H7_FDCAN2

  /* Enable and configure CAN2 */

  stm32_configgpio(GPIO_CAN2_STD);
  stm32_gpiowrite(GPIO_CAN2_STD, false);
  stm32_fdcansockinitialize(1);
#  endif

#endif

#ifdef CONFIG_MTD_W25QXXXJV
  ret = stm32_w25qxxx_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_n25qxxx_setup failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_RNDIS) && !defined(CONFIG_RNDIS_COMPOSITE)
  uint8_t mac[6];
  mac[0] = 0xa0;
  mac[1] = (CONFIG_NETINIT_MACADDR_2 >> (8 * 0)) & 0xff;
  mac[2] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 3)) & 0xff;
  mac[3] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 2)) & 0xff;
  mac[4] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 1)) & 0xff;
  mac[5] = (CONFIG_NETINIT_MACADDR_1 >> (8 * 0)) & 0xff;
  usbdev_rndis_initialize(mac);
#endif

#if defined(CONFIG_SENSORS_QENCODER)
  /* Initialize and register the qencoder driver */

  ret = board_qencoder_initialize(0, LINUMSTM32H753BI_QETIMER);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_CL_MFRC522
  ret = stm32_mfrc522initialize("/dev/rfid0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mfrc522initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_FT5X06
  /* Initialize the touchscreen */

  ret = stm32_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_tsc_setup failed: %d\n", ret);
    }
#endif

  return OK;
}
