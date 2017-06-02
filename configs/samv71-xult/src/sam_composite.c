/****************************************************************************
 * configs/samv71-xult/src/sam_composite.c
 *
 *   Copyright (C) 2016, 2017 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
#include <sys/types.h>

#include <nuttx/usb/composite.h>

#include "samv71-xult.h"

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
   *
   * You will also find an example below which generates three CDC/ACM
   * devices. This example can be used on samv71-xult.
   */

#if 1
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

  dev[0].devdesc.epno[CDCACM_EP_INTIN_IDX]   = 3;
  dev[0].devdesc.epno[CDCACM_EP_BULKIN_IDX]  = 4;
  dev[0].devdesc.epno[CDCACM_EP_BULKOUT_IDX] = 5;

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

  dev[1].devdesc.epno[USBMSC_EP_BULKIN_IDX]  = 1;
  dev[1].devdesc.epno[USBMSC_EP_BULKOUT_IDX] = 2;

  /* Count up the base numbers */

  ifnobase += dev[1].devdesc.ninterfaces;
  strbase  += dev[1].devdesc.nstrings;

  return composite_initialize(2, dev);
#else
  /* Example with three CDC/ACMs
   *
   * This Example can be used e.g. on a samv71-xult. The samv71 has
   * 10 Endpoints (EPs). The EPs 0 up to 7 are DMA aware. The EPs 8
   * and 9 are not.
   *
   * In a composite device we need the EP0 as an control Endpoint.
   * Each CDC/ACM needs one Interrupt driven and two bulk Endpoints.
   * This is why we configure the EPs 7, 8 and 9 to be the IRQ-EPs
   * and the EP-Pairs 1/2, 3/4, 5/6 to be the bulk EPs for each
   * device.
   *
   * This means, that
   *
   * - the Composite device uses EP0 as the control-Endpoint,
   * - the CDC/ACM 0 uses EP7, EP1 and EP2,
   * - the CDC/ACM 1 uses EP8, EP3 and EP4,
   * - the CDC/ACM 2 uses EP9, EP5 and EP6
   *
   * as its EP-Configuration.
   */

  struct composite_devdesc_struct dev[3];
  int strbase = COMPOSITE_NSTRIDS;
  int ifnobase = 0;
  int ia;

  for (ia = 0; ia < 3; ia++)
    {
      /* Ask the cdcacm driver to fill in the constants we didn't know here */

      cdcacm_get_composite_devdesc(&dev[ia]);

      /* Overwrite and correct some values... */
      /* The callback functions for the CDC/ACM class */

      dev[ia].classobject = cdcacm_classobject;
      dev[ia].uninitialize = cdcacm_uninitialize;

      dev[ia].minor = ia;                         /* The minor interface number */

      /* Interfaces */

      dev[ia].devdesc.ifnobase = ifnobase;        /* Offset to Interface-IDs */

      /* Strings */

      dev[ia].devdesc.strbase = strbase;          /* Offset to String Numbers */

      /* Endpoints */

      dev[ia].devdesc.epno[CDCACM_EP_INTIN_IDX]   = 7 + ia;
      dev[ia].devdesc.epno[CDCACM_EP_BULKIN_IDX]  = 1 + ia * 2;
      dev[ia].devdesc.epno[CDCACM_EP_BULKOUT_IDX] = 2 + ia * 2;

      ifnobase += dev[ia].devdesc.ninterfaces;
      strbase  += dev[ia].devdesc.nstrings;
    }

  return composite_initialize(3, dev);
#endif
}

#endif /* CONFIG_BOARDCTL_USBDEVCTRL && CONFIG_USBDEV_COMPOSITE */
