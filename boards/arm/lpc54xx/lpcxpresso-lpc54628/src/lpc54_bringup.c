/****************************************************************************
 * boards/arm/lpc54xx/lpcxpresso-lpc54628/src/lpc54_bringup.c
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

#include <nuttx/fs/fs.h>

#ifdef CONFIG_VIDEO_FB
#  include <nuttx/video/fb.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS_LOWER
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_LPC54_SDMMC
#  include <nuttx/sdio.h>
#  include <nuttx/mmcsd.h>
#  include "lpc54_sdmmc.h"
#endif

#ifdef CONFIG_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "lpc54_rtc.h"
#endif

#include "lpcxpresso-lpc54628.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc54_bringup
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

int lpc54_bringup(void)
{
#ifdef HAVE_MMCSD
  struct sdio_dev_s *sdmmc;
#endif
#ifdef HAVE_RTC_DRIVER
  struct rtc_lowerhalf_s *rtc;
#endif
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at /proc: %d\n", ret);
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32 lower-half RTC driver */

  rtc = lpc54_rtc_lowerhalf();
  if (rtc == NULL)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to instantiate the RTC lower-half driver\n");
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, rtc);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind/register the RTC driver: %d\n",
                 ret);
        }
    }
#endif

#ifdef HAVE_I2CTOOL
  /* Register I2C drivers on behalf of the I2C tool */

  lpc54_i2ctool();
#endif

#ifdef CONFIG_VIDEO_FB
  /* Initialize and register the framebuffer driver */

  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_FT5x06
  /* Register the FT5x06 touch panel driver */

  ret = lpc54_ft5x06_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lpc54_ft5x06_register() failed: %d\n", ret);
    }
#endif

#ifdef HAVE_MMCSD
  /* Get an instance of the SDIO interface */

  sdmmc = lpc54_sdmmc_initialize(0);
  if (!sdmmc)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SD/MMC\n");
    }
  else
    {
      /* Dind the SDIO interface to the MMC/SD driver */

      ret = mmcsd_slotinitialize(MMCSD_MINOR, sdmmc);
      if (ret != OK)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
                 ret);
        }
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS_LOWER
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
