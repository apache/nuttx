/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/src/sam_boot.c
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

#include <assert.h>
#include <debug.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <syslog.h>

#include <nuttx/board.h>
#include <arch/board/board.h>
#include <nuttx/arch.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_PL2303
#  include <nuttx/usb/pl2303.h>
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#include "sam4s-xplained-pro.h"

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
#ifdef CONFIG_ARCH_LEDS
  /* Configure on-board LEDs if LED support has been selected. */

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
#if defined (HAVE_USBDEV) || defined(HAVE_HSMCI) || defined (HAVE_PROC) || \
    defined(HAVE_USBMONITOR)
  int ret;
#endif

#if (defined(CONFIG_SAM34_WDT) && !defined(CONFIG_WDT_DISABLE_ON_RESET))
  /* Configure watchdog timer and enable kicker kernel thread. */

  int check = sam_watchdog_initialize();
  DEBUGASSERT(check >= 0);
  UNUSED(check);
#endif

#ifndef CONFIG_ARCH_LEDS
  /* Initialize user led */

  sam_led_initialize();
#endif

#ifdef HAVE_USBDEV
  syslog(LOG_INFO, "Registering CDC/ACM serial driver\n");

  ret = cdcacm_initialize(CONFIG_SAM4S_XPLAINED_PRO_CDCACM_DEVMINOR, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to create the CDC/ACM serial device: %d\n", ret);
      return;
    }
#endif

#ifdef HAVE_NAND
  ret = sam_nand_automount(SAM_SMC_CS0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the NAND: %d\n", ret);
      return;
    }
#endif

#ifdef HAVE_HSMCI
  /* Initialize the HSMCI driver */

  syslog(LOG_INFO, "initializing HSMCI\n");

  ret = sam_hsmci_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_hsmci_initialize() failed: %d\n", ret);
      return;
    }
#endif

#ifdef HAVE_PROC
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

#if HAVE_HSMCI
  syslog(LOG_INFO, "Mounting /dev/mmcsd0 to /fat\n");

  ret = nx_mount("/dev/mmcsd0", "/fat", "vfat", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the FAT filesystem: %d\n", ret);
      return;
    }
#endif

  /* SPI */

#ifdef HAVE_MMCSD_SPI
  ret = sam_sdinitialize(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MMC/SD slot: %d\n", ret);
      return;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  syslog(LOG_INFO, "Starting USB Monitor\n");
  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
      return;
    }
#endif
}
#endif /* CONFIG_BOARD_LATE_INITIALIZE */
