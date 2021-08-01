/****************************************************************************
 * boards/arm/stm32/emw3162/src/stm32_bringup.c
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

#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#include <arch/board/board.h>

#include "emw3162.h"

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

#ifdef CONFIG_EMW3162_WLAN
  /* Initialize wlan driver and hardware */

  ret = emw3162_wlan_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to initialize wlan: %d\n", ret);
      return ret;
    }
#endif

  return ret;
}
