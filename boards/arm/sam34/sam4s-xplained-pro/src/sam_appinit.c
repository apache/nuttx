/****************************************************************************
 * boards/arm/sam34/sam4s-xplained-pro/src/sam_appinit.c
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

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
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
#if defined (HAVE_USBDEV) || defined(HAVE_HSMCI) || defined (HAVE_PROC) || \
    defined(HAVE_USBMONITOR)
  int ret;
#endif

#ifdef HAVE_USBDEV
  syslog(LOG_INFO, "Registering CDC/ACM serial driver\n");

  ret = cdcacm_initialize(CONFIG_SAM4S_XPLAINED_PRO_CDCACM_DEVMINOR, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to create the CDC/ACM serial device: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_NAND
  ret = sam_nand_automount(SAM_SMC_CS0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize the NAND: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_HSMCI
  /* Initialize the HSMCI driver */

  syslog(LOG_INFO, "initializing HSMCI\n");

  ret = sam_hsmci_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_hsmci_initialize() failed: %d\n", ret);
      return ret;
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
      return ret;
    }
#endif

#if HAVE_HSMCI
  syslog(LOG_INFO, "Mounting /dev/mmcsd0 to /fat\n");

  ret = nx_mount("/dev/mmcsd0", "/fat", "vfat", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the FAT filesystem: %d\n", ret);
      return ret;
    }
#endif

  /* SPI */

#ifdef HAVE_MMCSD_SPI
  ret = sam_sdinitialize(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize MMC/SD slot: %d\n", ret);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  syslog(LOG_INFO, "Starting USB Monitor\n");
  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
      return ret;
    }
#endif

  return OK;
}
