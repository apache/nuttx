/****************************************************************************
 * boards/arm/kinetis/twr-k60n512/src/k60_boot.c
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

#ifdef CONFIG_KINETIS_SDHC
#  include <nuttx/sdio.h>
#  include <nuttx/mmcsd.h>
#endif

#include "arm_internal.h"
#include "kinetis.h"
#include "twr-k60n512.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_TWR_K60N512
#  define NSH_HAVEUSBDEV 1
#  define NSH_HAVEMMCSD  1
#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    error "Only one MMC/SD slot, slot 0"
#    undef CONFIG_NSH_MMCSDSLOTNO
#  endif
#  ifndef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
#else
  /* Add configuration for new Kinetis boards here */

#  error "Unrecognized Kinetis board"
#  undef NSH_HAVEUSBDEV
#  undef NSH_HAVEMMCSD
#endif

/* Can't support USB features if USB is not enabled */

#ifndef CONFIG_USBDEV
#  undef NSH_HAVEUSBDEV
#endif

/* Can't support MMC/SD features if mountpoints are disabled or if SDHC
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_KINETIS_SDHC)
#  undef NSH_HAVEMMCSD
#endif

#ifndef CONFIG_NSH_MMCSDMINOR
#  define CONFIG_NSH_MMCSDMINOR 0
#endif

/* We expect to receive GPIO interrupts for card insertion events */

#ifdef NSH_HAVEMMCSD
#  ifndef CONFIG_KINETIS_GPIOIRQ
#    error "CONFIG_KINETIS_GPIOIRQ required for card detect interrupt"
#  endif

#  ifndef CONFIG_KINETIS_PORTEINTS
#    error "CONFIG_KINETIS_PORTEINTS required for card detect interrupt"
#  endif
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure encapsulates the global variable used in this file and
 * reduces the probability of name collisions.
 */

#ifdef NSH_HAVEMMCSD
struct kinetis_nsh_s
{
  struct sdio_dev_s *sdhc; /* SDIO driver handle */
  bool inserted;           /* True: card is inserted */
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef NSH_HAVEMMCSD
static struct kinetis_nsh_s g_nsh;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_mediachange
 ****************************************************************************/

#ifdef NSH_HAVEMMCSD
static void kinetis_mediachange(void)
{
  bool inserted;

  /* Get the current value of the card detect pin.  This pin is pulled up on
   * board.  So low means that a card is present.
   */

  inserted = !kinetis_gpioread(GPIO_SD_CARDDETECT);

  /* Has the pin changed state? */

  if (inserted != g_nsh.inserted)
    {
      /* Yes..
       * perform the appropriate action (this might need some debounce).
       */

      g_nsh.inserted = inserted;
      sdhc_mediachange(g_nsh.sdhc, inserted);

      /* If the card has been inserted, then check if it is write protected
       * as well.  The pin is pulled up, but apparently logic high means
       * write protected.
       */

      if (inserted)
        {
          sdhc_wrprotect(g_nsh.sdhc, kinetis_gpioread(GPIO_SD_WRPROTECT));
        }
    }
}
#endif

/****************************************************************************
 * Name: kinetis_cdinterrupt
 ****************************************************************************/

#ifdef NSH_HAVEMMCSD
static int kinetis_cdinterrupt(int irq, void *context)
{
  /* All of the work is done by kinetis_mediachange() */

  kinetis_mediachange();
  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kinetis_boardinitialize
 *
 * Description:
 *   All Kinetis architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void kinetis_boardinitialize(void)
{
  /* Configure SPI chip selects if 1) SPI is not disabled, and 2) the weak
   * function kinetis_spidev_initialize() has been brought into the link.
   */

#if defined(CONFIG_KINETIS_SPI1) || defined(CONFIG_KINETIS_SPI2)
  if (kinetis_spidev_initialize)
    {
      kinetis_spidev_initialize();
    }
#endif

  /* Initialize USB is 1) USBDEV is selected, 2) the USB controller is not
   * disabled, and 3) the weak function kinetis_usbinitialize() has been
   * brought into the build.
   */

#if defined(CONFIG_USBDEV) && defined(CONFIG_KINETIS_USB)
  if (kinetis_usbinitialize)
    {
      kinetis_usbinitialize();
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
 *   function called board_late_initialize().  board_late_initialize() will
 *   be called immediately after up_intitialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
#ifdef NSH_HAVEMMCSD
  int ret;

  /* Configure GPIO pins */

  /* Attached the card detect interrupt (but don't enable it yet) */

  kinetis_pinconfig(GPIO_SD_CARDDETECT);
  kinetis_pinirqattach(GPIO_SD_CARDDETECT, kinetis_cdinterrupt, NULL);

  /* Configure the write protect GPIO */

  kinetis_pinconfig(GPIO_SD_WRPROTECT);

  /* Mount the SDHC-based MMC/SD block driver */

  /* First, get an instance of the SDHC interface */

  syslog(LOG_INFO, "Initializing SDHC slot %d\n",
         CONFIG_NSH_MMCSDSLOTNO);

  g_nsh.sdhc = sdhc_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!g_nsh.sdhc)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDHC slot %d\n",
             CONFIG_NSH_MMCSDSLOTNO);
      return;
    }

  /* Now bind the SDHC interface to the MMC/SD driver */

  syslog(LOG_INFO, "Bind SDHC to the MMC/SD driver, minor=%d\n",
         CONFIG_NSH_MMCSDMINOR);

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_nsh.sdhc);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDHC to the MMC/SD driver: %d\n",
             ret);
      return;
    }

  syslog(LOG_INFO, "Successfully bound SDHC to the MMC/SD driver\n");

  /* Handle the initial card state */

  kinetis_mediachange();

  /* Enable CD interrupts to handle subsequent media changes */

  kinetis_pinirqenable(GPIO_SD_CARDDETECT);
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
