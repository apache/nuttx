/****************************************************************************
 * boards/arm/rp23xx/raspberrypi-pico-2-w/src/rp23xx_composite.c
 *
 * SPDX-License-Identifier: Apache-2.0
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
#include <sys/types.h>
#include <debug.h>
#include <assert.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/composite.h>

#ifdef CONFIG_CDCACM_COMPOSITE
#  include <nuttx/usb/cdcacm.h>
#endif
#ifdef CONFIG_CDCNCM_COMPOSITE
#  include <nuttx/usb/cdcncm.h>
#endif

#if defined(CONFIG_BOARDCTL_USBDEVCTRL) && defined(CONFIG_USBDEV_COMPOSITE)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_composite_initialize
 ****************************************************************************/

int board_composite_initialize(int port)
{
  return OK;
}

/****************************************************************************
 * Name:  board_composite_connect
 *
 * Description:
 *   Connect the CDC/ACM serial console + CDC/NCM network composite device.
 *   The two classes use disjoint endpoint numbers (ACM: 1/2/3, NCM: 4/5/6)
 *   so they can coexist in one configuration.
 *
 ****************************************************************************/

void *board_composite_connect(int port, int configid)
{
  if (configid == 0)
    {
      struct composite_devdesc_s dev[2];
      int ifnobase = 0;
      int strbase  = COMPOSITE_NSTRIDS;
      int n = 0;

#ifdef CONFIG_CDCACM_COMPOSITE
      /* CDC/ACM serial console */

      cdcacm_get_composite_devdesc(&dev[n]);

      dev[n].classobject  = cdcacm_classobject;
      dev[n].uninitialize = cdcacm_uninitialize;

      dev[n].devinfo.ifnobase = ifnobase;
      dev[n].minor = 0;
      dev[n].devinfo.strbase = strbase;

      dev[n].devinfo.epno[CDCACM_EP_INTIN_IDX]   = 1;
      dev[n].devinfo.epno[CDCACM_EP_BULKIN_IDX]  = 2;
      dev[n].devinfo.epno[CDCACM_EP_BULKOUT_IDX] = 3;

      ifnobase += dev[n].devinfo.ninterfaces;
      strbase  += dev[n].devinfo.nstrings;
      n++;
#endif

#ifdef CONFIG_CDCNCM_COMPOSITE
      /* CDC/NCM network device */

      cdcncm_get_composite_devdesc(&dev[n]);

      dev[n].devinfo.ifnobase = ifnobase;
      dev[n].minor = 0;
      dev[n].devinfo.strbase = strbase;

      dev[n].devinfo.epno[CDCNCM_EP_INTIN_IDX]   = 4;
      dev[n].devinfo.epno[CDCNCM_EP_BULKIN_IDX]  = 5;
      dev[n].devinfo.epno[CDCNCM_EP_BULKOUT_IDX] = 6;

      ifnobase += dev[n].devinfo.ninterfaces;
      strbase  += dev[n].devinfo.nstrings;
      n++;
#endif

      return composite_initialize(composite_getdevdescs(), dev, n);
    }
  else
    {
      return NULL;
    }
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
