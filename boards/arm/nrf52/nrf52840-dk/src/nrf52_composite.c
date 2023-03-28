/****************************************************************************
 * boards/arm/nrf52/nrf52840-dk/src/nrf52_composite.c
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

#include <assert.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdcacm.h>
#include <nuttx/usb/rndis.h>
#include <nuttx/usb/composite.h>

#include "nrf52_usbd.h"

#include "nrf52840-dk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void *board_composite0_connect(void)
{
  struct composite_devdesc_s dev[2];
  int ifnobase = 0;
  int strbase  = COMPOSITE_NSTRIDS;
  int dev_idx = 0;
  int epin = 1;
  int epout = 1;

#ifdef CONFIG_RNDIS_COMPOSITE
  /* Configure the RNDIS USB device */

  /* Ask the rndis driver to fill in the constants we didn't
   * know here.
   */

  usbdev_rndis_get_composite_devdesc(&dev[dev_idx]);

  /* Interfaces */

  dev[dev_idx].devinfo.ifnobase = ifnobase;
  dev[dev_idx].minor = 0;

  /* Strings */

  dev[dev_idx].devinfo.strbase = strbase;

  /* Endpoints */

  dev[dev_idx].devinfo.epno[RNDIS_EP_INTIN_IDX] = epin++;
  dev[dev_idx].devinfo.epno[RNDIS_EP_BULKIN_IDX] = epin++;
  dev[dev_idx].devinfo.epno[RNDIS_EP_BULKOUT_IDX] = epout++;

  /* Count up the base numbers */

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase += dev[dev_idx].devinfo.nstrings;

  dev_idx += 1;
#endif

#ifdef CONFIG_CDCACM_COMPOSITE
  /* Configure the CDC/ACM device */

  /* Ask the cdcacm driver to fill in the constants we didn't
   * know here.
   */

  cdcacm_get_composite_devdesc(&dev[dev_idx]);

  /* Overwrite and correct some values... */

  /* The callback functions for the CDC/ACM class */

  dev[dev_idx].classobject  = cdcacm_classobject;
  dev[dev_idx].uninitialize = cdcacm_uninitialize;

  /* Interfaces */

  dev[dev_idx].devinfo.ifnobase = ifnobase;             /* Offset to Interface-IDs */
  dev[dev_idx].minor = 0;                               /* The minor interface number */

  /* Strings */

  dev[dev_idx].devinfo.strbase = strbase;               /* Offset to String Numbers */

  /* Endpoints */

  dev[dev_idx].devinfo.epno[CDCACM_EP_INTIN_IDX] = epin++;
  dev[dev_idx].devinfo.epno[CDCACM_EP_BULKIN_IDX] = epin++;
  dev[dev_idx].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = epout++;

  /* Count up the base numbers */

  ifnobase += dev[dev_idx].devinfo.ninterfaces;
  strbase  += dev[dev_idx].devinfo.nstrings;

  dev_idx += 1;
#endif

  DEBUGASSERT(epin <= NRF52_NENDPOINTS);
  DEBUGASSERT(epout <= NRF52_NENDPOINTS);

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
      return board_composite0_connect();
    }
  else
    {
      return NULL;
    }
}
