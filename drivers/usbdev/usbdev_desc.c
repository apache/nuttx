/****************************************************************************
 * drivers/usbdev/usbdev_desc.c
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

#include <string.h>

#include <nuttx/usb/usbdev.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbdev_copy_epdesc
 *
 * Description:
 *   Copies the requested Endpoint Description into the buffer given.
 *   Returns the number of Bytes filled in ( sizeof(struct usb_epdesc_s) ).
 *   This function is provided by various classes.
 *
 ****************************************************************************/

void usbdev_copy_epdesc(FAR struct usb_epdesc_s *epdesc,
                        uint8_t epno, bool hispeed,
                        FAR const struct usbdev_epinfo_s *epinfo)
{
#ifndef CONFIG_USBDEV_DUALSPEED
  UNUSED(hispeed);
#endif

  memcpy(epdesc, &epinfo->desc, sizeof(struct usb_epdesc_s));
  epdesc->addr |= epno;

#ifdef CONFIG_USBDEV_DUALSPEED
  if (hispeed)
    {
      /* Maximum packet size (high speed) */

      epdesc->mxpacketsize[0] = LSBYTE(epinfo->hssize);
      epdesc->mxpacketsize[1] = MSBYTE(epinfo->hssize);
    }
  else
#endif
    {
      /* Maximum packet size (full speed) */

      epdesc->mxpacketsize[0] = LSBYTE(epinfo->fssize);
      epdesc->mxpacketsize[1] = MSBYTE(epinfo->fssize);
    }
}
