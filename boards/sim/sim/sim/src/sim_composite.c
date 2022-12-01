/****************************************************************************
 * boards/sim/sim/sim/src/sim_composite.c
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
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/adb.h>
#include <nuttx/usb/rndis.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/cdcecm.h>
#include <nuttx/usb/composite.h>

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

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

static void *board_composite0_connect(int port)
{
  struct composite_devdesc_s dev[2];
  int ifnobase = 0;
  int strbase = COMPOSITE_NSTRIDS - 1;
  int dev_idx = 0;

#ifdef CONFIG_RNDIS
  /* Configure the RNDIS USB device */

  usbdev_rndis_get_composite_devdesc(&dev[dev_idx]);

  /* Interfaces */

  dev[dev_idx].devinfo.ifnobase = ifnobase;
  dev[dev_idx].minor = 0;

  /* Strings */

  dev[dev_idx].devinfo.strbase = strbase;

  /* Endpoints */

  dev[dev_idx].devinfo.epno[RNDIS_EP_BULKIN_IDX] = 1;
  dev[dev_idx].devinfo.epno[RNDIS_EP_BULKOUT_IDX] = 2;
  dev[dev_idx].devinfo.epno[RNDIS_EP_INTIN_IDX] = 5;

  /* Count up the base numbers */

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase += dev[dev_idx].devinfo.nstrings;

  dev_idx += 1;
#endif

#ifdef CONFIG_USBADB
  /* Configure the ADB USB device */

  usbdev_adb_get_composite_devdesc(&dev[dev_idx]);

  /* Interfaces */

  dev[dev_idx].devinfo.ifnobase = ifnobase;
  dev[dev_idx].minor = 0;

  /* Strings */

  dev[dev_idx].devinfo.strbase = strbase;

  /* Endpoints */

  dev[dev_idx].devinfo.epno[USBADB_EP_BULKIN_IDX] = 6;
  dev[dev_idx].devinfo.epno[USBADB_EP_BULKOUT_IDX] = 7;

  /* Count up the base numbers */

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase += dev[dev_idx].devinfo.nstrings;

  dev_idx += 1;
#endif

  return composite_initialize(dev_idx, dev);
}

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
  int ifnobase = 0;
  int strbase = COMPOSITE_NSTRIDS - 1;
  int dev_idx = 0;

#ifdef CONFIG_CDCACM
  /* Configure the CDC/ACM device */

  cdcacm_get_composite_devdesc(&dev[dev_idx]);

  /* The callback functions for the CDC/ACM class */

  dev[dev_idx].classobject = cdcacm_classobject;
  dev[dev_idx].uninitialize = cdcacm_uninitialize;

  /* Interfaces */

  dev[dev_idx].devinfo.ifnobase = ifnobase;
  dev[dev_idx].minor = 0;

  /* Strings */

  dev[dev_idx].devinfo.strbase = strbase;

  /* Endpoints */

  dev[dev_idx].devinfo.epno[CDCACM_EP_INTIN_IDX] = 5;
  dev[dev_idx].devinfo.epno[CDCACM_EP_BULKIN_IDX] = 6;
  dev[dev_idx].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 7;

  /* Count up the base numbers */

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase += dev[dev_idx].devinfo.nstrings;

  dev_idx += 1;
#endif

#ifdef CONFIG_NET_CDCECM
  /* Configure the CDC/ECM device */

  cdcecm_get_composite_devdesc(&dev[dev_idx]);

  /* Interfaces */

  dev[dev_idx].devinfo.ifnobase = ifnobase;
  dev[dev_idx].minor = 0;

  /* Strings */

  dev[dev_idx].devinfo.strbase = strbase;

  /* Endpoints */

  dev[dev_idx].devinfo.epno[CDCECM_EP_INTIN_IDX] = 10;
  dev[dev_idx].devinfo.epno[CDCECM_EP_BULKIN_IDX] = 11;
  dev[dev_idx].devinfo.epno[CDCECM_EP_BULKOUT_IDX] = 12;

  /* Count up the base numbers */

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase += dev[dev_idx].devinfo.nstrings;

  dev_idx += 1;
#endif

  return composite_initialize(dev_idx, dev);
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
      return board_composite0_connect(port);
    }
  else
    {
      return board_composite1_connect(port);
    }
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
