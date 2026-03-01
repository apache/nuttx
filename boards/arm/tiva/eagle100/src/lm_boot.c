/****************************************************************************
 * boards/arm/tiva/eagle100/src/lm_boot.c
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
#include <arch/board/board.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include "tiva_ssi.h"
#include "arm_internal.h"
#include "eagle100.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#    undef  CONFIG_NSH_MMCSDSPIPORTNO
#    define CONFIG_NSH_MMCSDSPIPORTNO 0
#    undef  CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0

/* Can't support USB features if USB is not enabled */

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tiva_boardinitialize
 *
 * Description:
 *   All Stellaris architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void tiva_boardinitialize(void)
{
  /* Configure SPI chip selects if
   * 1) SSI is not disabled, and
   * 2) the weak function
   * lm_ssidev_initialize() has been brought into the link.
   */

  /* The Eagle100 microSD CS is on SSI0 */

#if defined(CONFIG_TIVA_SSI0) /* || defined(CONFIG_TIVA_SSI1) */
  if (lm_ssidev_initialize)
    {
      lm_ssidev_initialize();
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
 *   called immediately after up_intitialize() is called and just before the
 *   initial application is started.  This additional initialization phase
 *   may be used, for example, to initialize board-specific device drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  struct spi_dev_s *spi;
#if defined (CONFIG_MMCSD) && defined (CONFIG_MMCSD_SPI)
  int ret;
#endif

  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

  spi = tiva_ssibus_initialize(CONFIG_NSH_MMCSDSPIPORTNO);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO);
      return;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO);

#if defined (CONFIG_MMCSD) && defined (CONFIG_MMCSD_SPI)
  /* Bind the SPI port to the slot */

  syslog(LOG_INFO, "Binding SPI port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(CONFIG_NSH_MMCSDMINOR,
                                CONFIG_NSH_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
             CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO, ret);
      return;
    }

  syslog(LOG_INFO, "Successfully bound SPI port %d to MMC/SD slot %d\n",
         CONFIG_NSH_MMCSDSPIPORTNO, CONFIG_NSH_MMCSDSLOTNO);
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
