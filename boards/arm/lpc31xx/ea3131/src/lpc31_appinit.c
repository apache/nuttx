/****************************************************************************
 * boards/arm/lpc31xx/ea3131/src/lpc31_appinit.c
 *
 *   Copyright (C) 2009, 2012, 2016 Gregory Nutt. All rights reserved.
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

#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>

#ifdef CONFIG_LPC31_MCI
#  include <nuttx/sdio.h>
#  include <nuttx/mmcsd.h>
#endif

#include "lpc31.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_EA3131
#  define NSH_HAVEUSBDEV 1
#  define NSH_HAVEMMCSD  1
#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    error "Only one MMC/SD slot"
#    undef CONFIG_NSH_MMCSDSLOTNO
#  endif
#  ifndef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
#else
  /* Add configuration for new LPC31XX boards here */

#  error "Unrecognized LPC31XX board"
#  undef NSH_HAVEUSBDEV
#  undef NSH_HAVEMMCSD
#endif

/* Can't support USB features if USB is not enabled */

#ifndef CONFIG_USBDEV
#  undef NSH_HAVEUSBDEV
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_LPC31_MCI)
#  undef NSH_HAVEMMCSD
#endif

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
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
#ifdef NSH_HAVEMMCSD
  FAR struct sdio_dev_s *sdio;
  int ret;

  /* First, get an instance of the SDIO interface */

  syslog(LOG_INFO, "Initializing SDIO slot %d\n",
         CONFIG_NSH_MMCSDSLOTNO);

  sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!sdio)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDIO slot %d\n",
             CONFIG_NSH_MMCSDSLOTNO);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the MMC/SD driver */

  syslog(LOG_INFO, "Bind SDIO to the MMC/SD driver, minor=%d\n",
         CONFIG_NSH_MMCSDMINOR);

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "Successfully bound SDIO to the MMC/SD driver\n");

  /* Then let's guess and say that there is a card in the slot.
   * I need to check to see if the LPC313X10E-EVAL board supports a GPIO to
   * detect if there is a card in the slot.
   */

  sdio_mediachange(sdio, true);
#endif
  return OK;
}
