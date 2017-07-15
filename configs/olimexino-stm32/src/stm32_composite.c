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

#include <nuttx/board.h>
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/composite.h>

#include "stm32.h"
#include "olimexino-stm32.h"

/* There is nothing to do here if SPI support is not selected. */

#ifdef CONFIG_STM32_SPI

/****************************************************************************
 * Pre-processor Definitions
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
 * Private Functions
 ****************************************************************************/

/************************************************************************************
 * Name: board_mscclassobject
 *
 * Description:
 *   If the mass storage class driver is part of composite device, then
 *   its instantiation and configuration is a multi-step, board-specific,
 *   process (See comments for usbmsc_configure below).  In this case,
 *   board-specific logic must provide board_mscclassobject().
 *
 *   board_mscclassobject() is called from the composite driver.  It must
 *   encapsulate the instantiation and configuration of the mass storage
 *   class and the return the mass storage device's class driver instance
 *   to the composite driver.
 *
 * Input Parameters:
 *   classdev - The location to return the mass storage class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ************************************************************************************/

static int board_mscclassobject(int minor, FAR struct usbdev_description_s *devdesc,
                                FAR struct usbdevclass_driver_s **classdev)
{
  FAR void *handle;
  int ret;

  ret = usbmsc_configure(1, &handle);
  if (ret >= 0)
    {
      ret = usbmsc_classobject(handle, devdesc, classdev);
    }

  return ret;
}

 /************************************************************************************
 * Name: board_mscuninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This is just an application-
 *   specific wrapper aboutn usbmsc_unitialize() that is called form the composite
 *   device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instrance previously give to the composite
 *     driver by board_mscclassobject().
 *
 * Returned Value:
 *   None
 *
 ************************************************************************************/

void board_mscuninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  usbmsc_uninitialize(classdev);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_composite_initialize
 *
 * Description:
 *   Perform architecture specific initialization of a composite USB device.
 *
 ****************************************************************************/

int board_composite_initialize(int port)
{
  /* If system/composite is built as an NSH command, then SD slot should
   * already have been initialized in board_app_initialize() (see stm32_appinit.c).
   * In this case, there is nothing further to be done here.
   */

  FAR struct spi_dev_s *spi;
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

/****************************************************************************
 * Name:  board_composite_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port using
 *   the specified configuration.  The interpretation of the configid is
 *   board specific.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *   configid - The USB composite configuration
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

FAR void *board_composite_connect(int port, int configid)
{
  /* Here we are composing the configuration of the usb composite device.
   *
   * The standard is to use one CDC/ACM and one USB mass storage device.
   */

  struct composite_devdesc_s dev[2];
  int ifnobase = 0;
  int strbase  = COMPOSITE_NSTRIDS;

  /* Configure the CDC/ACM device */

  /* Ask the cdcacm driver to fill in the constants we didn't
   * know here.
   */

  cdcacm_get_composite_devdesc(&dev[0]);

  /* Overwrite and correct some values... */
  /* The callback functions for the CDC/ACM class */

  dev[0].classobject  = cdcacm_classobject;
  dev[0].uninitialize = cdcacm_uninitialize;

  /* Interfaces */

  dev[0].devdesc.ifnobase = ifnobase;             /* Offset to Interface-IDs */
  dev[0].minor = CONFIG_SYSTEM_COMPOSITE_TTYUSB;  /* The minor interface number */

  /* Strings */

  dev[0].devdesc.strbase = strbase;               /* Offset to String Numbers */

  /* Endpoints */

  dev[0].devdesc.epno[CDCACM_EP_INTIN_IDX]   = 1;
  dev[0].devdesc.epno[CDCACM_EP_BULKIN_IDX]  = 2;
  dev[0].devdesc.epno[CDCACM_EP_BULKOUT_IDX] = 3;

  /* Count up the base numbers */

  ifnobase += dev[0].devdesc.ninterfaces;
  strbase  += dev[0].devdesc.nstrings;

  /* Configure the mass storage device device */
  /* Ask the usbmsc driver to fill in the constants we didn't
   * know here.
   */

  usbmsc_get_composite_devdesc(&dev[1]);

  /* Overwrite and correct some values... */
  /* The callback functions for the USBMSC class */

  dev[1].classobject  = board_mscclassobject;
  dev[1].uninitialize = board_mscuninitialize;

  /* Interfaces */

  dev[1].devdesc.ifnobase = ifnobase;               /* Offset to Interface-IDs */
  dev[1].minor = CONFIG_SYSTEM_COMPOSITE_DEVMINOR1; /* The minor interface number */

  /* Strings */

  dev[1].devdesc.strbase = strbase;   /* Offset to String Numbers */

  /* Endpoints */

  dev[1].devdesc.epno[USBMSC_EP_BULKIN_IDX]  = 5;
  dev[1].devdesc.epno[USBMSC_EP_BULKOUT_IDX] = 4;

  /* Count up the base numbers */

  ifnobase += dev[1].devdesc.ninterfaces;
  strbase  += dev[1].devdesc.nstrings;

  return composite_initialize(2, dev);
}

#endif /* CONFIG_STM32_SPI */
