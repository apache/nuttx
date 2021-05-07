/****************************************************************************
 * boards/avr/at90usb/teensy-2.0/src/at90usb_usbmsc.c
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
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>

#include "avr.h"
#include "at90usb.h"
#include "teensy-20.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SYSTEM_USBMSC_DEVMINOR1
#  define CONFIG_SYSTEM_USBMSC_DEVMINOR1 0
#endif

/* MMC/SD PORT and SLOT number */

#define AVR_MMCSDSPIPORTNO 0
#define AVR_MMCSDSLOTNO 0

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
  FAR struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port\n");

  spi = avr_spibus_initialize(AVR_MMCSDSPIPORTNO);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: avr_spibus_initialize failed\n");
      return -ENODEV;
    }

  /* Bind the SPI port to the slot */

  syslog(LOG_INFO, "Binding SPI port to MMC/SD slot\n");

  ret = mmcsd_spislotinitialize(CONFIG_SYSTEM_USBMSC_DEVMINOR1,
                                AVR_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: mmcsd_spislotinitialize failed: %d\n", ret);
      return ret;
    }

  return OK;
}
