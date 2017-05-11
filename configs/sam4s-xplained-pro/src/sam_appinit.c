/****************************************************************************
 * config/sam4s-xplained-pro/src/sam_appinit.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Bob Doiron
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/mount.h>

#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

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
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
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
             "ERROR: Failed to create the CDC/ACM serial device: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

#ifdef HAVE_HSMCI
  /* Initialize the HSMCI driver */

  syslog(LOG_INFO, "initializing HSMCI\n");

  ret = sam_hsmci_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: sam_hsmci_initialize() failed: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

#ifdef HAVE_PROC
  /* mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

#if HAVE_HSMCI
  syslog(LOG_INFO, "Mounting /dev/mmcsd0 to /fat\n");

  ret = mount("/dev/mmcsd0", "/fat", "vfat", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the FAT filesystem: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  syslog(LOG_INFO, "Starting USB Monitor\n");
  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d (%d)\n", ret, errno);
      return ret;
    }
#endif

  return OK;
}
