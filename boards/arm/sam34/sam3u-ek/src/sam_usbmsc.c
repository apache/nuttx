/****************************************************************************
 * boards/arm/sam34/sam3u-ek/src/sam_usbmsc.c
 *
 *   Copyright (C) 2009, 2013, 2016 Gregory Nutt. All rights reserved.
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
  FAR struct sdio_dev_s *sdio;
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
