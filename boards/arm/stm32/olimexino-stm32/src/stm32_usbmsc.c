/****************************************************************************
 * boards/arm/stm32/olimexino-stm32/src/stm32_usbmsc.c
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

#include "stm32.h"
#include "olimexino-stm32.h"

/* There is nothing to do here if SPI support is not selected. */

#ifdef CONFIG_STM32_SPI

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SYSTEM_USBMSC_DEVMINOR1
#  define CONFIG_SYSTEM_USBMSC_DEVMINOR1 0
#endif

/* SLOT number(s) could depend on the board configuration */

#ifdef CONFIG_ARCH_BOARD_OLIMEXINO_STM32
#  undef OLIMEXINO_STM32_MMCSDSLOTNO
#  define OLIMEXINO_STM32_MMCSDSLOTNO 0
#  undef OLIMEXINO_STM32_MMCSDSPIPORTNO
#  define OLIMEXINO_STM32_MMCSDSPIPORTNO 2
#else
/* Add configuration for new STM32 boards here */

#  error "Unrecognized STM32 board"
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
  /* If system/usbmsc is built as an NSH command, then SD slot should
   * already have been initialized in board_app_initialize()
   * (see stm32_appinit.c).
   * In this case, there is nothing further to be done here.
   */

  struct spi_dev_s *spi;
  int ret;

  /* First, get an instance of the SPI interface */

  syslog(LOG_INFO, "Initializing SPI port %d\n",
      OLIMEXINO_STM32_MMCSDSPIPORTNO);

  spi = stm32_spibus_initialize(OLIMEXINO_STM32_MMCSDSPIPORTNO);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port %d\n",
          OLIMEXINO_STM32_MMCSDSPIPORTNO);
      return -ENODEV;
    }

  syslog(LOG_INFO, "Successfully initialized SPI port %d\n",
      OLIMEXINO_STM32_MMCSDSPIPORTNO);

  /* Now bind the SPI interface to the MMC/SD driver */

  syslog(LOG_INFO, "Bind SPI to the MMC/SD driver, minor=%d slot=%d\n",
      CONFIG_SYSTEM_USBMSC_DEVMINOR1, OLIMEXINO_STM32_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(CONFIG_SYSTEM_USBMSC_DEVMINOR1,
                                OLIMEXINO_STM32_MMCSDSLOTNO, spi);
  if (ret < 0)
    {
      syslog(LOG_ERR,
         "ERROR: Failed to bind SPI port %d to MMC/SD minor=%d slot=%d %d\n",
         OLIMEXINO_STM32_MMCSDSPIPORTNO, CONFIG_SYSTEM_USBMSC_DEVMINOR1,
         OLIMEXINO_STM32_MMCSDSLOTNO, ret);
      return ret;
    }

  syslog(LOG_INFO, "Successfully bound SPI to the MMC/SD driver\n");

  return OK;
}

#endif /* CONFIG_STM32_SPI */
