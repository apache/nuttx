/****************************************************************************
 * drivers/usbdev/composite_desc.c
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

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>

#include "composite.h"

#ifdef CONFIG_USBDEV_COMPOSITE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Packet sizes */

#ifndef CONFIG_COMPOSITE_EP0MAXPACKET
#  define CONFIG_COMPOSITE_EP0MAXPACKET 64
#endif

/* Vendor and product IDs and strings */

#ifndef CONFIG_COMPOSITE_VENDORID
#  warning "CONFIG_COMPOSITE_VENDORID not defined"
#  define CONFIG_COMPOSITE_VENDORID     0x03eb
#endif

#ifndef CONFIG_COMPOSITE_PRODUCTID
#  warning "CONFIG_COMPOSITE_PRODUCTID not defined"
#  define CONFIG_COMPOSITE_PRODUCTID    0x2022
#endif

#ifndef CONFIG_COMPOSITE_VERSIONNO
#  define CONFIG_COMPOSITE_VERSIONNO    (0x0101)
#endif

#ifndef CONFIG_COMPOSITE_VENDORSTR
#  warning "No Vendor string specified"
#  define CONFIG_COMPOSITE_VENDORSTR    "NuttX"
#endif

#ifndef CONFIG_COMPOSITE_PRODUCTSTR
#  warning "No Product string specified"
#  define CONFIG_COMPOSITE_PRODUCTSTR   "Composite Device"
#endif

#undef CONFIG_COMPOSITE_SERIALSTR
#define CONFIG_COMPOSITE_SERIALSTR      "0101"

#undef CONFIG_COMPOSITE_CONFIGSTR
#define CONFIG_COMPOSITE_CONFIGSTR      "Composite"

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define COMPOSITE_SELFPOWERED         USB_CONFIG_ATTR_SELFPOWER
#else
#  define COMPOSITE_SELFPOWERED         (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define COMPOSITE_REMOTEWAKEUP        USB_CONFIG_ATTR_WAKEUP
#else
#  define COMPOSITE_REMOTEWAKEUP        (0)
#endif

/* Descriptors **************************************************************/

/* String language */

#define COMPOSITE_STR_LANGUAGE          (0x0409) /* en-us */

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Device Descriptor */

static const struct usb_devdesc_s g_devdesc =
{
  USB_SIZEOF_DEVDESC,                           /* len */
  USB_DESC_TYPE_DEVICE,                         /* type */
  {                                             /* usb */
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
#ifdef CONFIG_COMPOSITE_IAD
  USB_CLASS_MISC,                               /* classid */
  2,                                            /* subclass */
  1,                                            /* protocol */
#else
  USB_CLASS_PER_INTERFACE,                      /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
#endif
  CONFIG_COMPOSITE_EP0MAXPACKET,                /* maxpacketsize */
  {
    LSBYTE(CONFIG_COMPOSITE_VENDORID),          /* vendor */
    MSBYTE(CONFIG_COMPOSITE_VENDORID)
  },
  {
    LSBYTE(CONFIG_COMPOSITE_PRODUCTID),         /* product */
    MSBYTE(CONFIG_COMPOSITE_PRODUCTID)
  },
  {
    LSBYTE(CONFIG_COMPOSITE_VERSIONNO),         /* device */
    MSBYTE(CONFIG_COMPOSITE_VERSIONNO)
  },
  COMPOSITE_MANUFACTURERSTRID,                  /* imfgr */
  COMPOSITE_PRODUCTSTRID,                       /* iproduct */
  COMPOSITE_SERIALSTRID,                        /* serno */
  COMPOSITE_NCONFIGS                            /* nconfigs */
};

static const struct usbdev_strdesc_s g_strdesc[] =
{
  {COMPOSITE_MANUFACTURERSTRID, CONFIG_COMPOSITE_VENDORSTR},
  {COMPOSITE_PRODUCTSTRID,      CONFIG_COMPOSITE_PRODUCTSTR},
#ifdef CONFIG_COMPOSITE_SERIALSTR
  {COMPOSITE_SERIALSTRID,       CONFIG_COMPOSITE_SERIALSTR},
#else
  {COMPOSITE_SERIALSTRID,       ""},
#endif
  {COMPOSITE_CONFIGSTRID,       CONFIG_COMPOSITE_CONFIGSTR},
  {}
};

static const struct usbdev_strdescs_s g_strdescs =
{
  .language = COMPOSITE_STR_LANGUAGE,
  .strdesc  = g_strdesc,
};

static const struct usb_cfgdesc_s g_cfgdesc =
{
  .len      = USB_SIZEOF_CFGDESC,              /* Descriptor length    */
  .type     = USB_DESC_TYPE_CONFIG,            /* Descriptor type      */
  .cfgvalue = COMPOSITE_CONFIGID,              /* Configuration value  */
  .icfg     = COMPOSITE_CONFIGSTRID,           /* Configuration        */
  .attr     = USB_CONFIG_ATTR_ONE   |
              COMPOSITE_SELFPOWERED |
              COMPOSITE_REMOTEWAKEUP,          /* Attributes           */

  .mxpower  = (CONFIG_USBDEV_MAXPOWER + 1) / 2 /* Max power (mA/2) */
};

#ifdef CONFIG_USBDEV_DUALSPEED
static const struct usb_qualdesc_s g_qualdesc =
{
  USB_SIZEOF_QUALDESC,                          /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,                /* type */
  {                                             /* usb */
     LSBYTE(0x0200),
     MSBYTE(0x0200)
  },
#  ifdef CONFIG_COMPOSITE_IAD
  USB_CLASS_MISC,                               /* classid */
  2,                                            /* subclass */
  1,                                            /* protocol */
#  else
  USB_CLASS_VENDOR_SPEC,                        /* classid */
  0,                                            /* subclass */
  0,                                            /* protocol */
#  endif
  CONFIG_COMPOSITE_EP0MAXPACKET,                /* mxpacketsize */
  COMPOSITE_NCONFIGS,                           /* nconfigs */
  0,                                            /* reserved */
};
#endif

static const struct usbdev_devdescs_s g_composite_devdescs =
{
  &g_cfgdesc,
  &g_strdescs,
  &g_devdesc,
#ifdef CONFIG_USBDEV_DUALSPEED
  &g_qualdesc,
#endif
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

const char g_compvendorstr[]  = CONFIG_COMPOSITE_VENDORSTR;
const char g_compproductstr[] = CONFIG_COMPOSITE_PRODUCTSTR;
#ifndef CONFIG_COMPOSITE_BOARD_SERIALSTR
const char g_compserialstr[]  = CONFIG_COMPOSITE_SERIALSTR;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: composite_getdevdescs
 *
 * Description:
 *   Return a pointer to the device descriptor
 *
 ****************************************************************************/

FAR const struct usbdev_devdescs_s *composite_getdevdescs(void)
{
  return &g_composite_devdescs;
}

#endif /* CONFIG_USBDEV_COMPOSITE */
