/****************************************************************************
 * boards/arm/sam34/sam4e-ek/src/sam_usbmsc.c
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
