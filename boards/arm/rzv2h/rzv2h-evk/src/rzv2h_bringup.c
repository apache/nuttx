/****************************************************************************

 * boards/arm/rzv2h/rzv2h-evk/src/rzv2h_bringup.c
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
#include <sys/mount.h>
#include <syslog.h>

#include <arch/barriers.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/leds/userled.h>

#ifdef CONFIG_RZV2H_RTC
#  include <nuttx/timers/rtc.h>
#  include <nuttx/timers/arch_rtc.h>
#endif

#include "rzv2h-evk.h"

#ifdef CONFIG_RZV2H_RTC
#  include "rzv2h_rtc.h"
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rzv2h_bringup
 ****************************************************************************/

int rzv2h_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  UP_DSB();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#if defined(CONFIG_DEV_GPIO)
  ret = rzv2h_gpio_initialize();
  UP_DSB();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rzv2h_gpio_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
  /* Register the user LED driver.  The generic lower half calls
   * board_userled_initialize() before registering /dev/userleds.
   */

  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_RZV2H_RTC
  /* Initialize the RTC lower-half driver */

  ret = rzv2h_rtc_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rzv2h_rtc_initialize() failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_RZV2H_RTC) && defined(CONFIG_RTC_DRIVER)
  /* Register /dev/rtc0 and publish the lower half to the arch interface */

  ret = rtc_initialize(0, rzv2h_rtc_lowerhalf());
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rtc_initialize() failed: %d\n", ret);
    }
  else
    {
      /* Publish to CONFIG_RTC_ARCH bridge */

      FAR struct rtc_lowerhalf_s *lower = rzv2h_rtc_lowerhalf();

      up_rtc_set_lowerhalf(lower, false);

      /* Report where the boot system-clock seed came from. */

      if (lower->ops->havesettime != NULL &&
          lower->ops->havesettime(lower))
        {
          syslog(LOG_INFO,
                 "rzv2h_rtc: system clock seeded from RTC\n");
        }
      else
        {
          syslog(LOG_INFO,
                 "rzv2h_rtc: RTC not set; using build-time default date\n");
        }
    }
#endif

  return OK;
}
