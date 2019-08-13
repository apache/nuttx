/*****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_bringup.c
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Michael Jung <mijung@gmx.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/*****************************************************************************
 * Included Files
 *****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <sys/mount.h>

#include <nuttx/board.h>
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

/*****************************************************************************
 * Pre-processor Definitions
 *****************************************************************************/

/* Configuration *************************************************************/

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

/*****************************************************************************
 * Public Functions
 *****************************************************************************/

/****************************************************************************
 * Name: pnev5180b_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int pnev5180b_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_FS_PROCFS
  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
      goto done;
    }
#endif

#ifdef CONFIG_FS_BINFS
  ret = mount(NULL, "/bin", "binfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount the BIN filesystem: %d (%d)\n",
             ret, errno);
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
      syslog(LOG_ERR, "ERROR: lpc17_40_romfs_initialize() failed: %d\n", ret);
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
