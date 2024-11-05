/****************************************************************************
 * drivers/usbdev/usbmsc_desc.c
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
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev_trace.h>

#ifdef CONFIG_BOARD_USBDEV_SERIALSTR
#include <nuttx/board.h>
#endif

#include "usbmsc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Descriptors **************************************************************/

/* Device descriptor.  If the USB mass storage device is configured as part
 * of a composite device, then the device descriptor will be provided by the
 * composite device logic.
 */

#ifndef CONFIG_USBMSC_COMPOSITE
static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {LSBYTE(0x0200), MSBYTE(0x0200)},             /* usb */
  USB_CLASS_PER_INTERFACE,                      /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBMSC_EP0MAXPACKET,                   /* maxpacketsize */
  {                                             /* vendor */
    LSBYTE(CONFIG_USBMSC_VENDORID),
    MSBYTE(CONFIG_USBMSC_VENDORID)
  },
  {                                             /* product */
    LSBYTE(CONFIG_USBMSC_PRODUCTID),
    MSBYTE(CONFIG_USBMSC_PRODUCTID) },
  {                                             /* device */
    LSBYTE(CONFIG_USBMSC_VERSIONNO),
    MSBYTE(CONFIG_USBMSC_VERSIONNO)
  },
  USBMSC_MANUFACTURERSTRID,                     /* imfgr */
  USBMSC_PRODUCTSTRID,                          /* iproduct */
  USBMSC_SERIALSTRID,                           /* serno */
  USBMSC_NCONFIGS                               /* nconfigs */
};
#endif

#ifdef CONFIG_USBDEV_DUALSPEED
#ifndef CONFIG_USBMSC_COMPOSITE
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {                                             /* usb */
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  USB_CLASS_PER_INTERFACE,                      /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
  CONFIG_USBMSC_EP0MAXPACKET,                   /* mxpacketsize */
  USBMSC_NCONFIGS,                              /* nconfigs */
  0,                                            /* reserved */
};
#endif
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Strings ******************************************************************/

#ifndef CONFIG_USBMSC_COMPOSITE
const char g_mscvendorstr[]  = CONFIG_USBMSC_VENDORSTR;
const char g_mscproductstr[] = CONFIG_USBMSC_PRODUCTSTR;
#ifndef CONFIG_BOARD_USBDEV_SERIALSTR
const char g_mscserialstr[]  = CONFIG_USBMSC_SERIALSTR;
#endif
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbmsc_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int usbmsc_mkstrdesc(uint8_t id, struct FAR usb_strdesc_s *strdesc)
{
  FAR uint8_t *data = (FAR uint8_t *)(strdesc + 1);
  FAR const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
#ifndef CONFIG_USBMSC_COMPOSITE
    case 0:
      {
        /* Descriptor 0 is the language id */

        strdesc->len  = 4;
        strdesc->type = USB_DESC_TYPE_STRING;
        data[0] = LSBYTE(USBMSC_STR_LANGUAGE);
        data[1] = MSBYTE(USBMSC_STR_LANGUAGE);
        return 4;
      }

    case USBMSC_MANUFACTURERSTRID:
      str = g_mscvendorstr;
      break;

    case USBMSC_PRODUCTSTRID:
      str = g_mscproductstr;
      break;

    case USBMSC_SERIALSTRID:
#ifdef CONFIG_BOARD_USBDEV_SERIALSTR
      str = board_usbdev_serialstr();
#else
      str = g_mscserialstr;
#endif
      break;
#endif

  /* case USBMSC_CONFIGSTRID: */

    case USBMSC_INTERFACESTRID:
      str = CONFIG_USBMSC_CONFIGSTR;
      break;

    default:
      return -EINVAL;
    }

  /* The string is utf16-le.  The poor man's utf-8 to utf16-le
   * conversion below will only handle 7-bit en-us ascii
   */

  len = strlen(str);
  if (len > (USBMSC_MAXSTRLEN / 2))
    {
      len = (USBMSC_MAXSTRLEN / 2);
    }

  for (i = 0, ndata = 0; i < len; i++, ndata += 2)
    {
      data[ndata]     = str[i];
      data[ndata + 1] = 0;
    }

  strdesc->len  = ndata + 2;
  strdesc->type = USB_DESC_TYPE_STRING;
  return strdesc->len;
}

/****************************************************************************
 * Name: usbmsc_getdevdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_USBMSC_COMPOSITE
FAR const struct usb_devdesc_s *usbmsc_getdevdesc(void)
{
  return &g_devdesc;
}
#endif

/****************************************************************************
 * Name: usbmsc_copy_epcompdesc
 *
 * Description:
 *   Construct the endpoint companion descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_SUPERSPEED
static void
usbmsc_copy_epcompdesc(enum usbmsc_epdesc_e epid,
                       FAR struct usb_ss_epcompdesc_s *epcompdesc)
{
    switch (epid)
    {
    case USBMSC_EPBULKOUT:  /* Bulk OUT endpoint */
    case USBMSC_EPBULKIN:   /* Bulk IN endpoint */
      {
        epcompdesc->len  = USB_SIZEOF_SS_EPCOMPDESC;           /* Descriptor length */
        epcompdesc->type = USB_DESC_TYPE_ENDPOINT_COMPANION;   /* Descriptor type */

        if (USBMSC_SSBULKMAXBURST >= USB_SS_BULK_EP_MAXBURST)
          {
            epcompdesc->mxburst = USB_SS_BULK_EP_MAXBURST - 1;
          }
        else
          {
            epcompdesc->mxburst = USBMSC_SSBULKMAXBURST;
          }

        if (USBMSC_SSBULKMAXSTREAM > USB_SS_BULK_EP_MAXSTREAM)
          {
            epcompdesc->attr = USB_SS_BULK_EP_MAXSTREAM;
          }
        else
          {
            epcompdesc->attr = USBMSC_SSBULKMAXSTREAM;
          }

        epcompdesc->wbytes[0] = 0;
        epcompdesc->wbytes[1] = 0;
      }
      break;

    default:
      break;
    }
}
#endif

/****************************************************************************
 * Name: usbmsc_copy_epdesc
 *
 * Description:
 *   Copies the requested Endpoint Description into the buffer given.
 *   Returns the number of Bytes filled in ( sizeof(struct usb_epdesc_s) ).
 *
 ****************************************************************************/

int usbmsc_copy_epdesc(enum usbmsc_epdesc_e epid,
                       FAR struct usb_epdesc_s *epdesc,
                       FAR struct usbdev_devinfo_s *devinfo,
                       uint8_t speed)
{
  int len = sizeof(struct usb_epdesc_s);

#if !defined(CONFIG_USBDEV_DUALSPEED) && !defined(CONFIG_USBDEV_SUPERSPEED)
    UNUSED(speed);
#endif

#ifdef CONFIG_USBDEV_SUPERSPEED
  if (speed == USB_SPEED_SUPER || speed == USB_SPEED_SUPER_PLUS)
    {
      len += sizeof(struct usb_ss_epcompdesc_s);
    }
#endif

  if (epdesc == NULL)
    {
      return len;
    }

    switch (epid)
    {
    case USBMSC_EPBULKOUT:  /* Bulk OUT endpoint */
      {
        epdesc->len = USB_SIZEOF_EPDESC;            /* Descriptor length */
        epdesc->type = USB_DESC_TYPE_ENDPOINT;      /* Descriptor type */
        epdesc->addr = USBMSC_MKEPBULKOUT(devinfo); /* Endpoint address */
        epdesc->attr = USBMSC_EPOUTBULK_ATTR;       /* Endpoint attributes */

#ifdef CONFIG_USBDEV_SUPERSPEED
        if (speed == USB_SPEED_SUPER || speed == USB_SPEED_SUPER_PLUS)
          {
            /* Maximum packet size (super speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMSC_SSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMSC_SSBULKMAXPACKET);

            /* Copy endpoint companion description */

            epdesc++;
            usbmsc_copy_epcompdesc(epid,
                                   (FAR struct usb_ss_epcompdesc_s *)epdesc);
          }
        else
#endif
#ifdef CONFIG_USBDEV_DUALSPEED
        if (speed == USB_SPEED_HIGH)
          {
            /* Maximum packet size (high speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMSC_HSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMSC_HSBULKMAXPACKET);
          }
        else
#endif
          {
            /* Maximum packet size (full speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMSC_FSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMSC_FSBULKMAXPACKET);
          }

        epdesc->interval = 0;                       /* Interval */
      }
      break;

    case USBMSC_EPBULKIN:  /* Bulk IN endpoint */
      {
        epdesc->len = USB_SIZEOF_EPDESC;            /* Descriptor length */
        epdesc->type = USB_DESC_TYPE_ENDPOINT;      /* Descriptor type */
        epdesc->addr = USBMSC_MKEPBULKIN(devinfo);  /* Endpoint address */
        epdesc->attr = USBMSC_EPINBULK_ATTR;        /* Endpoint attributes */

#ifdef CONFIG_USBDEV_SUPERSPEED
        if (speed == USB_SPEED_SUPER || speed == USB_SPEED_SUPER_PLUS)
          {
            /* Maximum packet size (super speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMSC_SSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMSC_SSBULKMAXPACKET);

            /* Copy endpoint companion description */

            epdesc++;
            usbmsc_copy_epcompdesc(epid,
                                   (FAR struct usb_ss_epcompdesc_s *)epdesc);
          }
        else
#endif
#ifdef CONFIG_USBDEV_DUALSPEED
        if (speed == USB_SPEED_HIGH)
          {
            /* Maximum packet size (high speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMSC_HSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMSC_HSBULKMAXPACKET);
          }
        else
#endif
          {
            /* Maximum packet size (full speed) */

            epdesc->mxpacketsize[0] = LSBYTE(USBMSC_FSBULKMAXPACKET);
            epdesc->mxpacketsize[1] = MSBYTE(USBMSC_FSBULKMAXPACKET);
          }

        epdesc->interval = 0;                       /* Interval */
      }
      break;

    default:
        return 0;
    }

  return len;
}

/****************************************************************************
 * Name: usbmsc_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

int16_t usbmsc_mkcfgdesc(uint8_t *buf,
                         FAR struct usbdev_devinfo_s *devinfo,
                         uint8_t speed, uint8_t type)
{
#ifndef CONFIG_USBMSC_COMPOSITE
  FAR struct usb_cfgdesc_s *cfgdesc = NULL;
#endif
  int16_t totallen = 0;
  int ret;

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG && speed < USB_SPEED_SUPER)
    {
      speed = speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH;
    }

  /* Fill in all descriptors directly to the buf */

  /* Configuration descriptor.  If the USB mass storage device is
   * configured as part of a composite device, then the configuration
   * descriptor will be provided by the composite device logic.
   */

#ifndef CONFIG_USBMSC_COMPOSITE
  /* Configuration descriptor  If the USB mass storage device is
   * configured as part of a composite device, then the configuration
   * descriptor will be provided by the composite device logic.
   */

  cfgdesc = (FAR struct usb_cfgdesc_s *)buf;

  cfgdesc->len         = USB_SIZEOF_CFGDESC;               /* Descriptor length */
  cfgdesc->type        = type;                             /* Descriptor type */
  cfgdesc->ninterfaces = USBMSC_NINTERFACES;               /* Number of interfaces */
  cfgdesc->cfgvalue    = USBMSC_CONFIGID;                  /* Configuration value */
  cfgdesc->icfg        = USBMSC_CONFIGSTRID;               /* Configuration */
  cfgdesc->attr        = USB_CONFIG_ATTR_ONE |             /* Attributes */
                         USBMSC_SELFPOWERED |
                         USBMSC_REMOTEWAKEUP;
  cfgdesc->mxpower     = (CONFIG_USBDEV_MAXPOWER + 1) / 2; /* Max power (mA/2) */

  buf += sizeof(struct usb_cfgdesc_s);
  totallen += sizeof(struct usb_cfgdesc_s);
#endif

  /* Copy the canned interface descriptor */

  /* Single interface descriptor */

  FAR struct usb_ifdesc_s * dest = (struct usb_ifdesc_s *)buf;

  dest->len      = USB_SIZEOF_IFDESC;                        /* Descriptor length */
  dest->type     = USB_DESC_TYPE_INTERFACE;                  /* Descriptor type */
  dest->ifno     = devinfo->ifnobase;                        /* Interface number */
  dest->alt      = USBMSC_ALTINTERFACEID;                    /* Alternate setting */
  dest->neps     = USBMSC_NENDPOINTS;                        /* Number of endpoints */
  dest->classid  = USB_CLASS_MASS_STORAGE;                   /* Interface class */
  dest->subclass = USBMSC_SUBCLASS_SCSI;                     /* Interface sub-class */
  dest->protocol = USBMSC_PROTO_BULKONLY;                    /* Interface protocol */
  dest->iif      = devinfo->strbase + USBMSC_INTERFACESTRID; /* iInterface */

  buf += sizeof(struct usb_ifdesc_s);
  totallen += sizeof(struct usb_ifdesc_s);

  /* Make the two endpoint configurations */

  /* Bulk IN endpoint descriptor */

  ret = usbmsc_copy_epdesc(USBMSC_EPBULKIN,
                            (FAR struct usb_epdesc_s *)buf,
                            devinfo, speed);

  buf += ret;
  totallen += ret;

  /* Bulk OUT endpoint descriptor */

  ret = usbmsc_copy_epdesc(USBMSC_EPBULKOUT,
                            (FAR struct usb_epdesc_s *)buf,
                            devinfo, speed);

  buf += ret;
  totallen += ret;

#ifndef CONFIG_USBMSC_COMPOSITE
  if (cfgdesc)
    {
      cfgdesc->totallen[0] = LSBYTE(totallen);
      cfgdesc->totallen[1] = MSBYTE(totallen);
    }
#endif

  return totallen;
}

/****************************************************************************
 * Name: usbmsc_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_USBMSC_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *usbmsc_getqualdesc(void)
{
  return &g_qualdesc;
}
#endif
