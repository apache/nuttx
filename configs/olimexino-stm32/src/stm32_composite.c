/************************************************************************************
 * configs/olimexino-stm32/src/stm32_composite.c
 *
 *   Copyright (C) 2015 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           David Sidrane <david_s5@nscdg.com>
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

#include <nuttx/spi/spi.h>
#include <nuttx/mmcsd.h>

#include "stm32.h"
#include "olimexino-stm32.h"

/* There is nothing to do here if SPI support is not selected. */

#ifdef CONFIG_STM32_SPI

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SYSTEM_COMPOSITE_DEVMINOR1
#  define CONFIG_SYSTEM_COMPOSITE_DEVMINOR1 0
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
 * Name: composite_archinitialize
 *
 * Description:
 *   Perform architecture specific initialization
 *
 ****************************************************************************/

int composite_archinitialize(void)
{
  /* If system/composite is built as an NSH command, then SD slot should
   * already have been initialized in nsh_archinitialize() (see stm32_nsh.c).
   * In this case, there is nothing further to be done here.
   */

  FAR struct spi_dev_s *spi;
  int ret;

  /* First, get an instance of the SPI interface */

  syslog(LOG_INFO, "Initializing SPI port %d\n",
         OLIMEXINO_STM32_MMCSDSPIPORTNO);

  spi = up_spiinitialize(OLIMEXINO_STM32_MMCSDSPIPORTNO);
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
         CONFIG_SYSTEM_COMPOSITE_DEVMINOR1, OLIMEXINO_STM32_MMCSDSLOTNO);

  ret = mmcsd_spislotinitialize(CONFIG_SYSTEM_COMPOSITE_DEVMINOR1,
                                OLIMEXINO_STM32_MMCSDSLOTNO, spi);
  if (ret != OK)
    {
      syslog(LOG_ERR,
            "ERROR: Failed to bind SPI port %d to MMC/SD minor=%d slot=%d %d\n",
             OLIMEXINO_STM32_MMCSDSPIPORTNO, CONFIG_SYSTEM_COMPOSITE_DEVMINOR1,
             OLIMEXINO_STM32_MMCSDSLOTNO, ret);
      return ret;
    }

  syslog(LOG_INFO, "Successfully bound SPI to the MMC/SD driver\n");

   return OK;
}
#endif /* CONFIG_STM32_SPI */
