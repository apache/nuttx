/****************************************************************************
 * boards/arm/lpc17xx_40xx/u-blox-c027/src/lpc17_40_boot.c
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

#include <debug.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc17_40_gpio.h"
#include "lpc17_40_ssp.h"
#include "u-blox-c027.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifdef CONFIG_BOARDCTL

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_U_BLOX_C027
#  define NSH_HAVEUSBDEV 1
#  ifdef CONFIG_LPC17_40_SSP0
#    define NSH_HAVEMMCSD 1
#  else
#    undef NSH_HAVEMMCSD
#  endif
#else
#  error "Unrecognized board"
#  undef NSH_HAVEUSBDEV
#  undef NSH_HAVEMMCSD
#endif

/* Do we have SPI support for MMC/SD? */

#ifdef NSH_HAVEMMCSD
#    undef  CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO 0
#    undef  CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
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
#endif /* CONFIG_BOARDCTL */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lpc17_40_boardinitialize
 *
 * Description:
 *   All LPC17xx/LPC40xx architectures must provide the following entry
 *   point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void lpc17_40_boardinitialize(void)
{
  /* Configure SSP chip selects if 1) at least one SSP is enabled, and 2)
   * the weak function c027_sspdev_initialize() has been brought into the
   * link.
   */

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1)
  if (c027_sspdev_initialize)
    {
      c027_sspdev_initialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}

/****************************************************************************
 * Name: board_late_initialize
 *
 * Description:
 *   If CONFIG_BOARD_LATE_INITIALIZE is selected, then an additional
 *   initialization call will be performed in the boot-up sequence to a
 *   function called board_late_initialize(). board_late_initialize() will be
 *   called immediately after up_initialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

#ifdef CONFIG_MODEM_U_BLOX
  lpc17_40_ubxmdm_init(false);
#endif

#ifdef CONFIG_FS_PROCFS
  /* mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d\n", ret);
      return;
    }
#endif

#ifdef NSH_HAVEMMCSD
  struct spi_dev_s *ssp;

  /* Get the SSP port */

  ssp = lpc17_40_sspbus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!ssp)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SSP port %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO);
      return;
    }

  syslog(LOG_INFO, "Successfully initialized SSP port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

  /* Bind the SSP port to the slot */

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                                CONFIG_NSH_MMCSDSLOTNO, ssp);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SSP port %d to MMC/SD slot %d: %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO, ret);
      return;
    }

  syslog(LOG_INFO, "Successfully bound SSP port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = lpc17_40_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lpc17_40_pwm_setup() failed: %d\n", ret);
      return;
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = lpc17_40_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: lpc17_40_adc_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
