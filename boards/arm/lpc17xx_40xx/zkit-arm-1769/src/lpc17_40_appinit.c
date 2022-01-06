/****************************************************************************
 * boards/arm/lpc17xx_40xx/zkit-arm-1769/src/lpc17_40_appinit.c
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
#  ifdef CONFIG_NSH_ARCHINIT
#    if !defined(CONFIG_NSH_MMCSDSPIPORTNO) || CONFIG_NSH_MMCSDSPIPORTNO != 0
#      error "The ZKit-arm MMC/SD is on SPI port 0"
#      undef CONFIG_NSH_MMCSDSPIPORTNO
#      define CONFIG_NSH_MMCSDSPIPORTNO 0
#    endif
#    if !defined(CONFIG_NSH_MMCSDSLOTNO) || CONFIG_NSH_MMCSDSLOTNO != 0
#      error "The ZKit-arm MMC/SD has only one slot (0)"
#      undef CONFIG_NSH_MMCSDSLOTNO
#      define CONFIG_NSH_MMCSDSLOTNO 0
#    endif
#  else
#    undef CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO 0
#    undef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
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
#ifdef CONFIG_NSH_HAVEMMCSD
  FAR struct spi_dev_s *spi;
#endif
  int ret;

#ifdef CONFIG_NSH_HAVEMMCSD
  /* Get the SPI port */

  spi = lpc17_40_spibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!spi)
    {
      message("board_app_initialize: Failed to initialize SPI port %d\n",
              CONFIG_NSH_MMCSDSPIPORTNO);
      return -ENODEV;
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
      return ret;
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
  return OK;
}
