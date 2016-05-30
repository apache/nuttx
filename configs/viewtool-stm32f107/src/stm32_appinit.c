/****************************************************************************
 * config/viewtool-stm32f107/src/stm32_appinit.c
 *
 *   Copyright (C) 2013, 2016 Gregory Nutt. All rights reserved.
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
      sdbg("ERROR: Failed to instantiate the RTC lower-half driver\n");
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
          sdbg("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
        }
    }

  return ret;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef HAVE_RTC_DRIVER
  (void)rtc_driver_initialize();
#endif

#ifdef CONFIG_MPL115A
  stm32_mpl115ainitialize("/dev/press");
#endif

#ifdef HAVE_MMCSD
  return stm32_sdinitialize(CONFIG_NSH_MMCSDSLOTNO);
#else
  return OK;
#endif
}
