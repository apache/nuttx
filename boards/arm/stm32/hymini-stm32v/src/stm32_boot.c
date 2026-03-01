/****************************************************************************
 * boards/arm/stm32/hymini-stm32v/src/stm32_boot.c
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
#include <nuttx/spi/spi.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>

#ifdef CONFIG_STM32_SPI1
#  include <nuttx/spi/spi.h>
#  include <nuttx/mtd/mtd.h>
#endif

#ifdef CONFIG_STM32_SDIO
#  include <nuttx/sdio.h>
#  include <nuttx/mmcsd.h>
#endif

#include "arm_internal.h"
#include "stm32.h"
#include "hymini-stm32v.h"

#include <arch/board/board.h>  /* Should always be included last due to dependencies */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* For now, don't build in any SPI1 support -- NSH is not using it */

#undef CONFIG_STM32_SPI1

/* Check if we can have USB device in NSH */

#define NSH_HAVEUSBDEV 1

/* Can't support USB features if USB is not enabled */

#ifndef CONFIG_USBDEV
#  undef NSH_HAVEUSBDEV
#endif

/* Check if we can have MMC/SD slot support in NSH */

#define NSH_HAVEMMCSD  1

/* Can't support MMC/SD features if mountpoints are disabled or if SDIO
 * support is not enabled.
 */

#if defined(CONFIG_DISABLE_MOUNTPOINT) || !defined(CONFIG_STM32_SDIO)
#  undef NSH_HAVEMMCSD
#endif

#ifdef NSH_HAVEMMCSD
#  ifndef CONFIG_NSH_MMCSDMINOR
#    define CONFIG_NSH_MMCSDMINOR 0
#  endif
#  if defined(CONFIG_NSH_MMCSDSLOTNO) && CONFIG_NSH_MMCSDSLOTNO != 0
#    error "Only one MMC/SD slot"
#    undef CONFIG_NSH_MMCSDSLOTNO
#  endif
#  ifndef CONFIG_NSH_MMCSDSLOTNO
#    define CONFIG_NSH_MMCSDSLOTNO 0
#  endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_MMCSD
static struct sdio_dev_s *g_sdiodev;
static bool g_sd_inserted;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_cdinterrupt
 *
 * Description:
 *   Card detect interrupt handler.
 *
 ****************************************************************************/

#ifdef NSH_HAVEMMCSD
static int nsh_cdinterrupt(int irq, void *context, void *arg)
{
  bool present;

  present = !stm32_gpioread(GPIO_SD_CD);
  if (present != g_sd_inserted)
    {
      sdio_mediachange(g_sdiodev, present);
      g_sd_inserted = present;
    }

  return OK;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This
 *   entry point is called early in the initialization -- after all memory
 *   has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void stm32_boardinitialize(void)
{
  /* Configure SPI chip selects if
   * 1) SPI is not disabled, and
   * 2) the weak function stm32_spidev_initialize() has been brought into
   * the link.
   */

#if defined(CONFIG_STM32_SPI1) || defined(CONFIG_STM32_SPI2)
  if (stm32_spidev_initialize)
    {
      stm32_spidev_initialize();
    }
#endif

  /* Initialize USB is 1) USBDEV is selected, 2) the USB controller is not
   * disabled, and 3) the weak function stm32_usbinitialize() has been
   * brought into the build.
   */

#if defined(CONFIG_USBDEV) && defined(CONFIG_STM32_USB)
  if (stm32_usbinitialize)
    {
      stm32_usbinitialize();
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
 *   be called immediately after up_initialize() is called and just before
 *   the initial application is started.  This additional initialization
 *   phase may be used, for example, to initialize board-specific device
 *   drivers.
 *
 ****************************************************************************/

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  int ret;

#ifdef NSH_HAVEMMCSD
  /* Configure the card detect GPIO */

  stm32_configgpio(GPIO_SD_CD);

  /* Register an interrupt handler for the card detect pin */

  stm32_gpiosetevent(GPIO_SD_CD, true, true, true, nsh_cdinterrupt, NULL);

  /* Mount the SDIO-based MMC/SD block driver */

  /* First, get an instance of the SDIO interface */

  syslog(LOG_INFO, "Initializing SDIO slot %d\n",
         CONFIG_NSH_MMCSDSLOTNO);

  g_sdiodev = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!g_sdiodev)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDIO slot %d\n",
             CONFIG_NSH_MMCSDSLOTNO);
      return;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  syslog(LOG_INFO, "Bind SDIO to the MMC/SD driver, minor=%d\n",
         CONFIG_NSH_MMCSDMINOR);

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_sdiodev);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
             ret);
      return;
    }

  syslog(LOG_INFO, "Successfully bound SDIO to the MMC/SD driver\n");

  /* Use SD card detect pin to check if a card is inserted */

  g_sd_inserted = !stm32_gpioread(GPIO_SD_CD);
  _info("Card detect : %hhu\n", g_sd_inserted);

  sdio_mediachange(g_sdiodev, g_sd_inserted);
#endif

#ifdef CONFIG_INPUT
  /* Initialize the touchscreen */

  ret = stm32_tsc_setup(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_tsc_setup failed: %d\n", ret);
    }
#endif

  UNUSED(ret);
}
#endif
