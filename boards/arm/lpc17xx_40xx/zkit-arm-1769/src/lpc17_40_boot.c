/****************************************************************************
 * boards/arm/lpc17xx_40xx/zkit-arm-1769/src/lpc17_40_boot.c
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

#include <syslog.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "lpc17_40_spi.h"
#include "zkit-arm-1769.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_ZKITARM
#  define CONFIG_NSH_HAVEUSBDEV 1
#  ifdef CONFIG_LPC17_40_SPI
#    define CONFIG_NSH_HAVEMMCSD 1
#  else
#    undef CONFIG_NSH_HAVEMMCSD
#  endif
#else
#  error "Unrecognized board"
#  undef CONFIG_NSH_HAVEUSBDEV
#  undef CONFIG_NSH_HAVEMMCSD
#endif

/* Do we have SPI support for MMC/SD? */

#ifdef CONFIG_NSH_HAVEMMCSD
#    undef CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO 0
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#endif

/* Can't support USB device features if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef CONFIG_NSH_HAVEUSBDEV
#endif

/* Can't support MMC/SD features if mountpoints are disabled */

#if defined(CONFIG_DISABLE_MOUNTPOINT)
#  undef CONFIG_NSH_HAVEMMCSD
#endif

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/* Debug ********************************************************************/

#ifdef CONFIG_DEBUG_INFO
#  define message _info
#else
#  define message _err
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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
   * the weak function zkit_sspdev_initialize() has been brought into the
   * link.
   */

#if defined(CONFIG_LPC17_40_SSP0) || defined(CONFIG_LPC17_40_SSP1)
  if (zkit_sspdev_initialize)
    {
      zkit_sspdev_initialize();
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
#ifdef CONFIG_NSH_HAVEMMCSD
  struct spi_dev_s *spi;
#endif
  int ret;

#ifdef CONFIG_NSH_HAVEMMCSD
  /* Get the SPI port */

  spi = lpc17_40_spibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!spi)
    {
      message("Failed to initialize SPI port %d\n",
              CONFIG_NSH_MMCSDSPIPORTNO);
      return;
    }

  message("Successfully initialized SPI port %d\n",
          CONFIG_NSH_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                                CONFIG_NSH_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      message("Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
              CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO, ret);
      return;
    }

  message("Successfully bound SPI port %d to MMC/SD slot %d\n",
          CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = zkit_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: zkit_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAN
  /* Initialize CAN and register the CAN driver. */

  ret = zkit_can_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: zkit_can_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
}
#endif
