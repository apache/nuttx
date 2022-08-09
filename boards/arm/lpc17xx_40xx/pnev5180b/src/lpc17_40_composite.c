/****************************************************************************
 * boards/arm/lpc17xx_40xx/pnev5180b/src/lpc17_40_composite.c
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

#include <syslog.h>
#include <stddef.h>

#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/cdcecm.h>
#include <nuttx/usb/composite.h>

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

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
  syslog(LOG_INFO, "board_composite_initialize(port: %d)\n", port);

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
  struct composite_devdesc_s dev[2];
  int ifnobase = 0;
  int strbase  = COMPOSITE_NSTRIDS;

  syslog(LOG_INFO, "board_composite_connect(port: %d, configid: %d)\n",
         port, configid);

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

  dev[0].devinfo.ifnobase = ifnobase;         /* Offset to Interface-IDs */
  dev[0].minor = 0;                           /* The minor interface number */

  /* Strings */

  dev[0].devinfo.strbase = strbase;           /* Offset to String Numbers */

  /* Endpoints */

  dev[0].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 1;
  dev[0].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 5;
  dev[0].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 2;

  /* Count up the base numbers */

  ifnobase += dev[0].devinfo.ninterfaces;
  strbase  += dev[0].devinfo.nstrings;

  /* Configure the CDC/ECM device */

  cdcecm_get_composite_devdesc(&dev[1]);

  dev[1].devinfo.ifnobase = ifnobase;         /* Offset to Interface-IDs */
  dev[1].devinfo.strbase = strbase;           /* Offset to String Numbers */
  dev[1].minor = 0;                           /* The minor interface number */
  dev[1].devinfo.epno[CDCECM_EP_INTIN_IDX]   = 4;
  dev[1].devinfo.epno[CDCECM_EP_BULKIN_IDX]  = 11;
  dev[1].devinfo.epno[CDCECM_EP_BULKOUT_IDX] = 8;

  return composite_initialize(2, dev);
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
