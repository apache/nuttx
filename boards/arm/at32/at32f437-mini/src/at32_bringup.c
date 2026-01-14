/****************************************************************************
 * boards/arm/at32/at32f437-mini/src/at32_bringup.c
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

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>

#include "at32.h"
#include "../include/board.h"
#include "at32f437-mini.h"

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "at32_rtc.h"
#endif

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

/****************************************************************************
 * Name: at32_bringup
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

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

int at32_bringup(void)
{
  int ret = OK;

#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *lower;
#endif

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, AT32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      serr("ERROR: Failed to mount procfs at %s: %d\n",
          AT32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  at32_gpio_initialize();
#endif

#ifdef CONFIG_USERLED
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, \
      "ERROR: userled_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the AT32 lower-half RTC driver */

  lower = at32_rtc_lowerhalf();
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

#ifdef CONFIG_AT32_IWDG
  /* Initialize the watchdog timer */

  at32_iwdginitialize("/dev/watchdog0", AT32_LSI_FREQUENCY);
#endif

#ifdef CONFIG_MTD_W25
  /* Initialize and register the W25 FLASH file system. */

  ret = at32_w25initialize(W25QXX_FLASH_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize W25 minor %d: %d\n",
            W25QXX_FLASH_MINOR, ret);
      return ret;
    }
#endif

#ifdef CONFIG_AT32_CAN_CHARDRIVER
  /* Initialize CAN and register the CAN driver. */
#if defined(CONFIG_AT32_CAN1)
  ret = at32_can_setup(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: at32_can_setup can1 failed: %d\n", ret);
    }
#endif

#endif

#ifdef CONFIG_AT32_CAN_SOCKET
  /* Initialize CAN socket interface */
#if defined(CONFIG_AT32_CAN1)
  ret = at32_cansock_setup(1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: at32_cansock_setup failed: %d\n", ret);
    }
#endif

#endif

#ifdef HAVE_USBHOST
  ret = at32_usbhost_initialize();
  if (ret != OK)
    {
      uerr("ERROR: Failed to initialize USB host: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_AT32_SDIO
  /* Initialize the SDIO block driver */

  ret = at32_sdinitialize(0);
  if (ret != OK)
    {
      syslog(LOG_ERR, \
      "ERROR: Failed to initialize MMC/SD driver: %d\n", ret);
    }
#endif

#ifdef CONFIG_AT32_PWM
  /* Initialize PWM and register the PWM device. */

  ret = at32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: at32_pwm_setup() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_AT32_TIM) && defined(CONFIG_TIMER)
  /* Initialize TIM and register the TIM device. */

  ret = at32_timer_driver_setup("/dev/timer0", AT32F437_MINI_TIMER);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: timer setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_AT32_ADC
  /* Initialize ADC and register the ADC device. */

  ret = at32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: at32_adc_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_MTD_AT24XX
  /* Initialize the AT24 driver */

  ret = at32_at24_automount(AT24_MINOR);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: at32_at24_automount() failed: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}
