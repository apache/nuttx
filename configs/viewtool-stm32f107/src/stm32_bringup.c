/****************************************************************************
 * config/viewtool-stm32f107/src/stm32_bringup.c
 *
 *   Copyright (C) 2013, 2016-2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>

#ifdef CONFIG_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32_rtc.h"
#endif

#include "viewtool_stm32f107.h"

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
  FAR struct rtc_lowerhalf_s *lower;
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
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
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

  ret = mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
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

#ifdef CONFIG_CAN
  /* Initialize CAN and register the CAN driver. */

  ret = stm32_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_can_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_MPL115A
  ret = stm32_mpl115ainitialize("/dev/press");
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
