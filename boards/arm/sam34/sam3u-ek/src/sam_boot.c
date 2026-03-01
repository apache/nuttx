/****************************************************************************
 * boards/arm/sam34/sam3u-ek/src/sam_boot.c
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
#include <stdbool.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/mmcsd.h>
#include <nuttx/sdio.h>

#include "arm_internal.h"
#include "sam_hsmci.h"
#include "sam3u-ek.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#define NSH_HAVE_USBDEV 1
#define NSH_HAVE_MMCSD  1

/* Can't support MMC/SD if the card interface is not enable */

#ifndef CONFIG_SAM34_HSMCI
#  undef NSH_HAVE_MMCSD
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_SAM34_HSMCI)
#  undef NSH_HAVE_MMCSD
#endif

#ifdef NSH_HAVE_MMCSD
#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    error "Only one MMC/SD slot"
#    undef CONFIG_NSH_MMCSDSLOTNO
#  endif

#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif

#  ifndef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
#endif

/* Can't support USB features if USB is not enabled */

#ifndef CONFIG_USBDEV
#  undef NSH_HAVE_USBDEV
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sam_boardinitialize
 *
 * Description:
 *   All SAM3U architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void sam_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function sam_spidev_initialize() has been brought into the link.
   */

#ifdef CONFIG_SAM34_SPI0
  if (sam_spidev_initialize)
    {
      sam_spidev_initialize();
    }
#endif

  /* Configure on-board LEDs if LED support has been selected. */

#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif

  /* Setup SD card-related PIOs if 1) HSMCI is selected and 2) the weak
   * function sam_hsmciinit() has been brought into the build.
   */

#ifdef CONFIG_SAM34_HSMCI
  if (sam_hsmciinit)
    {
      sam_hsmciinit();
    }
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

#ifdef NSH_HAVE_MMCSD
  struct sdio_dev_s *sdio;

  /* Mount the SDIO-based MMC/SD block driver */

  /* First, get an instance of the SDIO interface */

  syslog(LOG_INFO, "Initializing SDIO slot %d\n",
         CONFIG_NSH_MMCSDSLOTNO);

  sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!sdio)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDIO slot %d\n",
             CONFIG_NSH_MMCSDSLOTNO);
      return;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  syslog(LOG_INFO, "Bind SDIO to the MMC/SD driver, minor=%d\n",
         CONFIG_NSH_MMCSDMINOR);

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, sdio);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n", ret);
      return;
    }

  syslog(LOG_INFO, "Successfully bound SDIO to the MMC/SD driver\n");

  /* Then inform the HSMCI driver if there is or is not a card in the slot. */

  sdio_mediachange(sdio, sam_cardinserted(0));
#endif

#ifdef CONFIG_INPUT
  /* Initialize the touchscreen */

  ret = sam_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_tsc_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
}
#endif
