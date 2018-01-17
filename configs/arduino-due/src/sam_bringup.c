/****************************************************************************
 * config/arduino-due/src/sam_bringup.c
 *
 *   Copyright (C) 2013, 2016, 2018 Gregory Nutt. All rights reserved.
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

#include <sys/mount.h>
#include <stdio.h>
#include <syslog.h>

#include <nuttx/board.h>

#include "arduino-due.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if defined(CONFIG_ARDUINO_ITHEAD_TFT) && defined(CONFIG_SPI_BITBANG) && \
    defined(CONFIG_MMCSD_SPI)
/* Support for the SD card slot on the ITEAD TFT shield */
/* Verify NSH PORT and SLOT settings */

#  define SAM34_MMCSDSLOTNO    0 /* There is only one slot */

#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != SAM34_MMCSDSLOTNO
#    error Only one MMC/SD slot:  Slot 0 (CONFIG_NSH_MMCSDSLOTNO)
#  endif

/* Default MMC/SD minor number */

#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_INITIALIZE=y :
 *     Called from board_initialize().
 *
 *   CONFIG_BOARD_INITIALIZE=y && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int sam_bringup(void)
{
  int ret;

#ifdef CONFIG_FS_PROCFS
  /* Mount the procfs file system */

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,"ERROR: Failed to mount procfs at /proc: %d\n",
            ret);
    }
#endif

#if defined(CONFIG_ARDUINO_ITHEAD_TFT) && defined(CONFIG_SPI_BITBANG) && \
    defined(CONFIG_MMCSD_SPI)
  /* Initialize the SPI-based MMC/SD slot */

  {
    int ret = sam_sdinitialize(CONFIG_NSH_MMCSDMINOR);
    if (ret < 0)
      {
        syslog(LOG_ERR,
               "board_app_initialize: Failed to initialize MMC/SD slot: %d\n",
               ret);
       return ret;
      }
  }
#endif

#if defined(CONFIG_ARDUINO_ITHEAD_TFT) && defined(CONFIG_SPI_BITBANG) && \
    defined(CONFIG_INPUT_ADS7843E)
  /* Initialize the touchscreen */

  ret = sam_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_tsc_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
