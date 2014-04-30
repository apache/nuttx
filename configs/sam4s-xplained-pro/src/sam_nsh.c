/****************************************************************************
 * config/sam4s-xplained-pro/src/sam_nsh.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
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
#include <debug.h>
#include <unistd.h>

#include <nuttx/arch.h>

#ifdef CONFIG_CDCACM
#  include <nuttx/usb/cdcacm.h>
#endif

#ifdef CONFIG_PL2303
#  include <nuttx/usb/pl2303.h>
#endif

#ifdef CONFIG_TIMER
#  include <nuttx/timer.h>
#endif

#ifdef CONFIG_SYSTEM_USBMONITOR
#  include <apps/usbmonitor.h>
#endif

#include "sam4s-xplained-pro.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

#ifdef CONFIG_CPP_HAVE_VARARGS
#  ifdef CONFIG_DEBUG
#    define message(...) syslog(__VA_ARGS__)
#  else
#    define message(...) printf(__VA_ARGS__)
#  endif
#else
#  ifdef CONFIG_DEBUG
#    define message syslog
#  else
#    define message printf
#  endif
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: nsh_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int nsh_archinitialize(void)
{
#if defined (HAVE_USBDEV) || defined(HAVE_HSMCI) || defined (HAVE_PROC) || \
    defined(HAVE_USBMONITOR)
  int ret;
#endif

#ifdef HAVE_USBDEV
  message("Registering CDC/ACM serial driver\n");
  ret = cdcacm_initialize(CONFIG_SAM4S_XPLAINED_PRO_CDCACM_DEVMINOR, NULL);
  if (ret < 0)
    {
      message("ERROR: Failed to create the CDC/ACM serial device: %d (%d)\n", ret, errno);
      return ret;
    }
#endif

#ifdef HAVE_HSMCI
  /* Initialize the HSMCI driver */

  message("initializing HSMCI\n");
  ret = sam_hsmci_initialize();
  if (ret < 0)
    {
      message("ERROR: sam_hsmci_initialize() failed: %d (%d)\n", ret, errno);
      return ret;
    }
#endif

#ifdef HAVE_PROC
  /* mount the proc filesystem */

  message("Mounting procfs to /proc\n");
  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      message("ERROR: Failed to mount the PROC filesystem: %d (%d)\n", ret, errno);
      return ret;
    }
#endif

#if HAVE_HSMCI
  message("Mounting /dev/mmcsd0 to /fat\n");
  ret = mount("/dev/mmcsd0", "/fat", "vfat", 0, NULL);
  if (ret < 0)
    {
      message("ERROR: Failed to mount the FAT filesystem: %d (%d)\n", ret, errno);
      return ret;
    }
#endif

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  message("Starting USB Monitor\n");
  ret = usbmonitor_start(0, NULL);
  if (ret != OK)
    {
      message("nsh_archinitialize: Start USB monitor: %d (%d)\n", ret, errno);
      return ret;
    }
#endif

  return OK;
}
