/****************************************************************************
 * boards/arm/lpc214x/mcu123-lpc214x/src/lpc2148_usbmsc.c
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

#include "lpc214x_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SYSTEM_USBMSC_DEVMINOR1
#  define CONFIG_SYSTEM_USBMSC_DEVMINOR1 0
#endif

/* PORT and SLOT number probably depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_MCU123_LPC214X
#  undef LPC214X_MMCSDSPIPORTNO
#  define LPC214X_MMCSDSPIPORTNO 1
#  undef LPC214X_MMCSDSLOTNO
#  define LPC214X_MMCSDSLOTNO 0

#else
  /* Add configuration for new LPC214x boards here */

#  error "Unrecognized LPC214x board"
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_usbmsc_initialize
 *
 * Description:
 *   Perform architecture specific initialization of the USB MSC device.
 *
 ****************************************************************************/

int board_usbmsc_initialize(int port)
{
  struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port %d\n",
         LPC214X_MMCSDSPIPORTNO);

  spi = lpc214x_spibus_initialize(LPC214X_MMCSDSPIPORTNO);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
             LPC214X_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port %d\n",
         LPC214X_MMCSDSPIPORTNO);

  /* Bind the SPI port to the slot */

  syslog(LOG_INFO, "Binding SPI port %d to MMC/SD slot %d\n",
         LPC214X_MMCSDSPIPORTNO, LPC214X_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(CONFIG_SYSTEM_USBMSC_DEVMINOR1,
                                LPC214X_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SPI port %d to MMC/SD slot %d: %d\n",
             LPC214X_MMCSDSPIPORTNO, LPC214X_MMCSDSLOTNO, ret);
      return ret;
    }

  syslog(LOG_INFO, "Successfully bound SPI port %d to MMC/SD slot %d\n",
         LPC214X_MMCSDSPIPORTNO, LPC214X_MMCSDSLOTNO);
  return OK;
}
