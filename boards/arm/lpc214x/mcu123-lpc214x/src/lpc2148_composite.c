/****************************************************************************
 * boards/arm/lpc214x/mcu123-lpc214x/src/lpc2148_composite.c
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
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/mmcsd.h>
#include <nuttx/spi/spi.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/usbmsc.h>
#include <nuttx/usb/composite.h>

#include "lpc214x_spi.h"

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* PORT and SLOT number depend on the board configuration */

#define LPC214X_MMCSDSPIPORTNO 1
#define LPC214X_MMCSDSLOTNO 0

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static void *g_mschandle;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
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
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static int board_mscclassobject(int minor,
                                struct usbdev_devinfo_s *devinfo,
                                struct usbdevclass_driver_s **classdev)
{
  int ret;

  DEBUGASSERT(g_mschandle == NULL);

  /* Configure the mass storage device */

  uinfo("Configuring with NLUNS=1\n");
  ret = usbmsc_configure(1, &g_mschandle);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_configure failed: %d\n", -ret);
      return ret;
    }

  uinfo("MSC handle=%p\n", g_mschandle);

  /* Bind the LUN(s) */

  uinfo("Bind LUN=0 to /dev/mmcsd0\n");
  ret = usbmsc_bindlun(g_mschandle, "/dev/mmcsd0", 0, 0, 0, false);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_bindlun failed for LUN 1 at /dev/mmcsd0: %d\n",
           ret);
      usbmsc_uninitialize(g_mschandle);
      g_mschandle = NULL;
      return ret;
    }

  /* Get the mass storage device's class object */

  ret = usbmsc_classobject(g_mschandle, devinfo, classdev);
  if (ret < 0)
    {
      uerr("ERROR: usbmsc_classobject failed: %d\n", -ret);
      usbmsc_uninitialize(g_mschandle);
      g_mschandle = NULL;
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: board_mscuninitialize
 *
 * Description:
 *   Un-initialize the USB storage class driver.  This is just an
 *   application specific wrapper aboutn usbmsc_unitialize() that is called
 *   form the composite device logic.
 *
 * Input Parameters:
 *   classdev - The class driver instrance previously give to the composite
 *     driver by board_mscclassobject().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static void board_mscuninitialize(struct usbdevclass_driver_s *classdev)
{
  DEBUGASSERT(g_mschandle != NULL);
  usbmsc_uninitialize(g_mschandle);
  g_mschandle = NULL;
}
#endif

/****************************************************************************
 * Name:  board_composite0_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port for
 *   configuration 0.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

#ifdef CONFIG_USBMSC_COMPOSITE
static void *board_composite0_connect(int port)
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

  dev[0].devinfo.ifnobase = ifnobase;             /* Offset to Interface-IDs */
  dev[0].minor = 0;                               /* The minor interface number */

  /* Strings */

  dev[0].devinfo.strbase = strbase;               /* Offset to String Numbers */

  /* Endpoints */

  dev[0].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 1;
  dev[0].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 2;
  dev[0].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 3;

  /* Count up the base numbers */

  ifnobase += dev[0].devinfo.ninterfaces;
  strbase  += dev[0].devinfo.nstrings;

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

  dev[1].devinfo.ifnobase = ifnobase; /* Offset to Interface-IDs */
  dev[1].minor = 0;                   /* The minor interface number */

  /* Strings */

  dev[1].devinfo.strbase = strbase;   /* Offset to String Numbers */

  /* Endpoints */

  dev[1].devinfo.epno[USBMSC_EP_BULKIN_IDX]  = 5;
  dev[1].devinfo.epno[USBMSC_EP_BULKOUT_IDX] = 4;

  /* Count up the base numbers */

  ifnobase += dev[1].devinfo.ninterfaces;
  strbase  += dev[1].devinfo.nstrings;

  return composite_initialize(2, dev);
}
#endif

/****************************************************************************
 * Name:  board_composite1_connect
 *
 * Description:
 *   Connect the USB composite device on the specified USB device port for
 *   configuration 1.
 *
 * Input Parameters:
 *   port     - The USB device port.
 *
 * Returned Value:
 *   A non-NULL handle value is returned on success.  NULL is returned on
 *   any failure.
 *
 ****************************************************************************/

static void *board_composite1_connect(int port)
{
  struct composite_devdesc_s dev[2];
  int strbase = COMPOSITE_NSTRIDS;
  int ifnobase = 0;
  int epno;
  int i;

  for (i = 0, epno = 1; i < 2; i++)
    {
      /* Ask the cdcacm driver to fill in the constants we didn't know here */

      cdcacm_get_composite_devdesc(&dev[i]);

      /* Overwrite and correct some values... */

      /* The callback functions for the CDC/ACM class */

      dev[i].classobject = cdcacm_classobject;
      dev[i].uninitialize = cdcacm_uninitialize;

      dev[i].minor = i;                         /* The minor interface number */

      /* Interfaces */

      dev[i].devinfo.ifnobase = ifnobase;        /* Offset to Interface-IDs */

      /* Strings */

      dev[i].devinfo.strbase = strbase;          /* Offset to String Numbers */

      /* Endpoints */

      dev[i].devinfo.epno[CDCACM_EP_INTIN_IDX]   = epno++;
      dev[i].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = epno++;
      dev[i].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = epno++;

      ifnobase += dev[i].devinfo.ninterfaces;
      strbase  += dev[i].devinfo.nstrings;
    }

  return composite_initialize(2, dev);
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
   * already have been initialized in board_app_initialize()
   * (see lpc2148_appinit.c).
   * In this case, there is nothing further to be done here.
   *
   * NOTE: CONFIG_NSH_BUILTIN_APPS is not a fool-proof indication that NSH
   * was built.
   */

#ifndef CONFIG_NSH_BUILTIN_APPS
  struct spi_dev_s *spi;
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

  ret = mmcsd_spislotinitialize(0, LPC214X_MMCSDSLOTNO, spi);
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

void *board_composite_connect(int port, int configid)
{
  if (configid == 0)
    {
#ifdef CONFIG_USBMSC_COMPOSITE
      return board_composite0_connect(port);
#else
      return NULL;
#endif
    }
  else if (configid == 1)
    {
      return board_composite1_connect(port);
    }
  else
    {
      return NULL;
    }
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
