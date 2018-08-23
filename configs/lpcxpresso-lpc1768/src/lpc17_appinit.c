/****************************************************************************
 * config/lpcxpresso-lpc1768/src/lpc17_appinit.c
 *
 *   Copyright (C) 2011, 2016 Gregory Nutt. All rights reserved.
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

#include "lpc17_ssp.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_LIB_BOARDCTL

/* PORT and SLOT number probably depend on the board configuration */

#define NSH_HAVEUSBDEV 1
#ifdef CONFIG_LPC17_SSP1
#  define NSH_HAVEMMCSD 1
#else
#  undef NSH_HAVEMMCSD
#endif

/* Do we have SPI support for MMC/SD? */

#ifdef NSH_HAVEMMCSD
#ifdef CONFIG_NSH_ARCHINIT
#  if !defined(CONFIG_NSH_MMCSDSPIPORTNO) || CONFIG_NSH_MMCSDSPIPORTNO != 1
#    error "The LPCXpresso MMC/SD is on SSP1"
#    undef CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO 1
#  endif
#  if !defined(CONFIG_NSH_MMCSDSLOTNO) || CONFIG_NSH_MMCSDSLOTNO != 0
#    error "The LPCXpresso MMC/SD has only one slot (0)"
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
#else
#  undef  CONFIG_NSH_MMCSDSPIPORTNO
#  define CONFIG_NSH_MMCSDSPIPORTNO 1
#  undef  CONFIG_NSH_MMCSDSLOTNO
#  define CONFIG_NSH_MMCSDSLOTNO 0
#endif
#endif

/* Can't support USB device features if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef NSH_HAVEUSBDEV
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef NSH_HAVEMMCSD
#endif

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/* Currnently MMC/SD support is available only for NSH configurations */

#else
#  undef NSH_HAVEMMCSD
#endif /* CONFIG_LIB_BOARDCTL */

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
#ifdef NSH_HAVEMMCSD
  FAR struct spi_dev_s *ssp;
#endif
  int ret;

#ifdef NSH_HAVEMMCSD
  /* Get the SSP port */

  ssp = lpc17_sspbus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!ssp)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SSP port %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  syslog(LOG_INFO, "Successfully initialized SSP port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

  /* Bind the SSP port to the slot */

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR, CONFIG_NSH_MMCSDSLOTNO, ssp);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to bind SSP port %d to MMC/SD slot %d: %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO, ret);
      return ret;
    }

  syslog(LOG_INFO, "Successfuly bound SSP port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = lpcexpresso_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lpcexpresso_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = lpcxpresso_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lpcxpresso_adc_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
  return OK;
}
