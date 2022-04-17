/****************************************************************************
 * boards/arm/stm32/viewtool-stm32f107/src/stm32_bringup.c
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
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

#include "viewtool_stm32f107.h"

#ifdef CONFIG_SENSORS_MPL115A
#include "stm32_mpl115a.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Default MMC/SD SLOT number */

#ifdef HAVE_MMCSD
#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != VIEWTOOL_MMCSD_SLOTNO
#    error "Only one MMC/SD slot:  VIEWTOOL_MMCSD_SLOTNO"
#    undef  CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO VIEWTOOL_MMCSD_SLOTNO
#  endif

#  ifndef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO VIEWTOOL_MMCSD_SLOTNO
#  endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: rtc_driver_initialize
 *
 * Description:
 *   Initialize and register the RTC driver.
 *
 ****************************************************************************/

#ifdef HAVE_RTC_DRIVER
static int rtc_driver_initialize(void)
{
  struct rtc_lowerhalf_s *lower;
  int ret;

  /* Instantiate the STM32 lower-half RTC driver */

  lower = stm32_rtc_lowerhalf();
  if (lower == NULL)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      ret = -ENOMEM;
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
        }
    }

  return ret;
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
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret;

#ifdef HAVE_RTC_DRIVER
  ret = rtc_driver_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: rtc_driver_initialize failed: %d\n", ret);
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

#ifdef HAVE_MMCSD
  ret = stm32_sdinitialize(CONFIG_NSH_MMCSDSLOTNO);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_sdinitialize failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_INPUT_ADS7843E
  /* Initialize the touchscreen */

  ret = stm32_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_tsc_setup failed: %d\n", ret);
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

#ifdef CONFIG_SENSORS_MPL115A
  ret = board_mpl115a_initialize(0, 5);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mpl115ainitialize failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_VIEWTOOL_FT80X_SPI1) || defined(CONFIG_VIEWTOOL_FT80X_SPI2)
  ret = stm32_ft80x_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_ft80x_setup failed: %d\n", ret);
    }
#endif

#if defined(CONFIG_VIEWTOOL_MAX3421E_SPI1) || defined(CONFIG_VIEWTOOL_MAX3421E_SPI2)
  ret = stm32_max3421e_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_ft80x_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
