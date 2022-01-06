/****************************************************************************
 * boards/arm/stm32/photon/src/stm32_bringup.c
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

#include <nuttx/input/buttons.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "photon.h"
#include "stm32_wdg.h"

#ifdef CONFIG_USBADB
#  include <nuttx/usb/adb.h>
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Called either by board_initialize() if CONFIG_BOARD_LATE_INITIALIZE or
 *   by board_app_initialize if CONFIG_BOARDCTL is selected.
 *   This function initializes and configures all on-board features
 *   appropriate for the selected configuration.
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_USERLED) && !defined(CONFIG_ARCH_LEDS)
#ifdef CONFIG_USERLED_LOWER
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#else
  /* Enable USER LED support for some other purpose */

  board_userled_initialize();
#endif /* CONFIG_USERLED_LOWER */
#endif /* CONFIG_USERLED && !CONFIG_ARCH_LEDS */

#ifdef CONFIG_INPUT_BUTTONS
#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#else
  /* Enable BUTTON support for some other purpose */

  board_button_initialize();
#endif /* CONFIG_INPUT_BUTTONS_LOWER */
#endif /* CONFIG_INPUT_BUTTONS */

#ifdef CONFIG_STM32_IWDG
  /* Initialize the watchdog timer */

  stm32_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
#endif

#ifdef CONFIG_PHOTON_WDG
  /* Start WDG kicker thread */

  ret = photon_watchdog_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to start watchdog thread: %d\n", ret);
      return ret;
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

#ifdef CONFIG_PHOTON_WLAN
  /* Initialize wlan driver and hardware */

  ret = photon_wlan_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize wlan: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_USBDEV_COMPOSITE

#ifndef CONFIG_BOARDCTL_USBDEVCTRL
  ret = board_composite_initialize(0);
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize composite: %d\n", ret);
      return ret;
    }

  if (board_composite_connect(0, 0) == NULL)
    {
      syslog(LOG_ERR, "Failed to connect composite: %d\n", ret);
      return ret;
    }
#endif /* !CONFIG_BOARDCTL_USBDEVCTRL */
#else
#ifdef CONFIG_USBADB
  usbdev_adb_initialize();
#endif
#endif /* CONFIG_USBDEV_COMPOSITE */
  return ret;
}
