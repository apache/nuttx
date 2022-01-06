/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_bringup.c
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

#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/binfmt/elf.h>
#include <nuttx/binfmt/nxflat.h>
#include <nuttx/binfmt/symtab.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_NET_CDCECM
#  include <nuttx/usb/cdcecm.h>
#  include <net/if.h>
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_CDCECM_COMPOSITE
#  include <nuttx/board.h>
#endif

#ifdef CONFIG_LPC17_40_ROMFS
#  include "lpc17_40_romfs.h"
#endif

#include "lpc17_40_spi.h"
#include "pnev5180b.h"
#include "lpc17_40_symtab.h"
#include "lpc17_40_progmem.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_PNEV5180B
#  define CONFIG_NSH_HAVEUSBDEV 1
#else
#  error "Unrecognized board"
#  undef CONFIG_NSH_HAVEUSBDEV
#endif

/* Can't support USB device features if USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef CONFIG_NSH_HAVEUSBDEV
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: pnev5180b_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int pnev5180b_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  ret = nx_mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the PROC filesystem: %d\n",
             ret);
      goto done;
    }
#endif

#ifdef CONFIG_FS_BINFS
  ret = nx_mount(NULL, "/bin", "binfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the BIN filesystem: %d\n",
             ret);
      goto done;
    }
#endif

#if !defined(CONFIG_BOARDCTL_USBDEVCTRL) && !defined(CONFIG_USBDEV_COMPOSITE)
#  ifdef CONFIG_CDCACM
  ret = cdcacm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcacm_initialize() failed: %d\n", ret);
      goto done;
    }
#  endif

#  ifdef CONFIG_NET_CDCECM
  ret = cdcecm_initialize(0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: cdcecm_initialize() failed: %d\n", ret);
      goto done;
    }
#  endif
#endif /* CONFIG_BOARDCTL_USBDEVCTRL */

#if defined(CONFIG_USBDEV_COMPOSITE)
  board_composite_connect(0, 0);
#endif

#ifdef CONFIG_LPC17_40_ROMFS
  ret = lpc17_40_romfs_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: lpc17_40_romfs_initialize() failed: %d\n",
             ret);
      goto done;
    }
#endif

#ifdef CONFIG_USBMONITOR
  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: usbmonitor_start() failed: %d\n", ret);
      goto done;
    }
#endif

#ifdef CONFIG_ELF
  ret = elf_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: elf_initialize() failed: %d\n", ret);
      goto done;
    }
#endif

#ifdef CONFIG_NXFLAT
  ret = nxflat_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: nxflat_initialize() failed: %d\n", ret);
      goto done;
    }
#endif

#if defined(CONFIG_ELF) || defined(CONFIG_NXFLAT)
  exec_setsymtab(lpc17_40_exports, lpc17_40_nexports);
#endif

  /* To avoid 'unused label' compiler warnings in specific configuration. */

  goto done;

done:
  return ret;
}
