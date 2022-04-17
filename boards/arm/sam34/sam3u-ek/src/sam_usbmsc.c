/****************************************************************************
 * boards/arm/sam34/sam3u-ek/src/sam_usbmsc.c
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
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include "sam_hsmci.h"

#ifdef CONFIG_SAM34_HSMCI

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
  struct sdio_dev_s *sdio;
  int ret;

  /* First, get an instance of the SDIO interface */

  syslg(LOG_INFO, "Initializing SDIO slot %d\n",
          SAM_MMCSDSLOTNO);

  sdio = sdio_initialize(SAM_MMCSDSLOTNO);
  if (!sdio)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDIO slot %d\n",
             SAM_MMCSDSLOTNO);
      return -ENODEV;
    }

  /* Now bind the SPI interface to the MMC/SD driver */

  syslog(LOG_INFO, ""
         "Bind SDIO to the MMC/SD driver, minor=%d\n",
         CONFIG_SYSTEM_USBMSC_DEVMINOR1);

  ret = mmcsd_slotinitialize(CONFIG_SYSTEM_USBMSC_DEVMINOR1, sdio);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
             ret);
      return ret;
    }

  syslog(LOG_INFO, "Successfully bound SDIO to the MMC/SD driver\n");

  /* Then let's guess and say that there is a card in the slot.
   * I need to check to see if the SAM3U10E-EVAL board supports a GPIO
   * to detect if there is a card in the slot.
   */

  sdio_mediachange(sdio, true);
  return OK;
}

#endif /* CONFIG_SAM34_HSMCI */
