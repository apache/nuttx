/****************************************************************************
 * configs/mcu123-lpc214x/src/lpc2148_composite.c
 *
 *   Copyright (C) 2012, 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Configure and register the LPC214x MMC/SD SPI block driver.
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

#include "lpc214x_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SYSTEM_COMPOSITE_DEVMINOR1
#  define CONFIG_SYSTEM_COMPOSITE_DEVMINOR1 0
#endif

/* PORT and SLOT number depend on the board configuration */

#define LPC214X_MMCSDSPIPORTNO 1
#define LPC214X_MMCSDSLOTNO 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_cdcclassobject
 *
 * Description:
 *   If the CDC serial class driver is part of composite device, then
 *   board-specific logic must provide board_cdcclassobject().  In the
 *   simplest case, board_cdcclassobject() is simply a wrapper around
 *   cdcacm_classobject() that provides the correct device minor number.
 *
 * Input Parameters:
 *   classdev - The location to return the CDC serial class' device
 *     instance.
 *
 * Returned Value:
 *   0 on success; a negated errno on failure
 *
 ****************************************************************************/

static int board_cdcclassobject(int minor,
                                FAR struct usbdev_description_s *devdesc,
                                FAR struct usbdevclass_driver_s **classdev)
{
  return cdcacm_classobject(minor, devdesc, classdev);
}

/****************************************************************************
 * Name: board_cdcuninitialize
 *
 * Description:
 *   Un-initialize the USB serial class driver.  This is just an application-
 *   specific wrapper around cdcadm_unitialize() that is called form the
 *   composite device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instance previously given to the composite
 *     driver by board_cdcclassobject().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void board_cdcuninitialize(FAR struct usbdevclass_driver_s *classdev)
{
  cdcacm_initialize(classdev);
}

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
 *   to the composite dirver.
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
      retr = usbmsc_classobject(handle, devdesc, classdev);
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
   * already have been initialized in board_app_initialize() (see lpc2148_appinit.c).
   * In this case, there is nothing further to be done here.
   *
   * NOTE: CONFIG_NSH_BUILTIN_APPS is not a fool-proof indication that NSH
   * was built.
   */

#ifndef CONFIG_NSH_BUILTIN_APPS
  FAR struct spi_dev_s *spi;
  int ret;

  /* Get the SPI port */

  syslog(LOG_INFO, "Initializing SPI port %d\n", LPC214X_MMCSDSPIPORTNO);

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

  ret = mmcsd_spislotinitialize(CONFIG_SYSTEM_COMPOSITE_DEVMINOR1,
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

#endif /* CONFIG_NSH_BUILTIN_APPS */

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

  dev[0].classobject  = board_cdcclassobject;
  dev[0].uninitialize = board_cdcuninitialize;

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
