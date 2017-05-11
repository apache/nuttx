/****************************************************************************
 * config/mcu123-lpc214x/src/lpc2148_appinit.c
 *
 *   Copyright (C) 2008-2009, 2016 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include "lpc214x_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_USBDEV 1
#define HAVE_MMCSD  1

/* PORT and SLOT number depend on the board configuration */

#ifdef CONFIG_NSH_ARCHINIT
#  if !defined(CONFIG_NSH_MMCSDSPIPORTNO) || CONFIG_NSH_MMCSDSPIPORTNO != 1
#    error "The LPC214x MMC/SD is on SPI1"
#    undef CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO 1
#  endif

#  if !defined(CONFIG_NSH_MMCSDSLOTNO) || CONFIG_NSH_MMCSDSLOTNO != 0
#    error "The LPC214x MMC/SD is on SPI1"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif

#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif

#else
#  undef  CONFIG_NSH_MMCSDSPIPORTNO
#  define CONFIG_NSH_MMCSDSPIPORTNO 1
#  undef  CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#  undef  CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/* Can't support USB features if USB is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef HAVE_MMCSD
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
 *         between the board-specific initalization logic and the
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
#ifdef HAVE_MMCSD
  FAR struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

  spi = lpc214x_spibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  syslog(LOG_INFO, "Binding SPI port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO, ret);
      return ret;
    }

  syslog(LOG_INFO, "Successfuly bound SPI port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);
#endif

  return OK;
}
