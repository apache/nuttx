/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_usbmsc.c
 *
 *   Copyright (C) 2014, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Configure and register the SAM3U MMC/SD SDIO block driver.
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

#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/board.h>

#include "sam4e-ek.h"

#ifdef CONFIG_SAM34_UDP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SYSTEM_USBMSC_DEVMINOR1
#  define CONFIG_SYSTEM_USBMSC_DEVMINOR1 0
#endif

/* SLOT number(s) depends on the board configuration */

#undef SAM_MMCSDSLOTNO
#define SAM_MMCSDSLOTNO 0

/* Can't use a block device if it is not available */

#ifndef HAVE_AT25
#  undef CONFIG_SAM4EEK_AT25_BLOCKDEVICE
#endif

#ifndef HAVE_HSMCI
#  undef CONFIG_SAM4EEK_HSMCI_BLOCKDEVICE
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_usbmsc_initialize
 *
 * Description:
 *   Perform architecture specific initialization as needed to establish
 *   the mass storage device that will be exported by the USB MSC device.
 *
 ****************************************************************************/

int board_usbmsc_initialize(int port)
{
  /* Initialize the AT25 MTD driver */

#if defined(CONFIG_SAM4EEK_AT25_BLOCKDEVICE)
  int ret = sam_at25_automount(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_at25_automount failed: %d\n", ret);
    }

  return ret;

#elif defined(CONFIG_SAM4EEK_HSMCI_BLOCKDEVICE)
  /* Initialize the HSMCI driver */

  int ret = sam_hsmci_initialize(0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: sam_hsmci_initialize(0) failed: %d\n", ret);
    }

  return ret;

#else
  return -ENODEV;
#endif
}

#endif /* CONFIG_SAM34_UDP */
