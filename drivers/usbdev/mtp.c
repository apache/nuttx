/****************************************************************************
 * drivers/usbdev/mtp.c
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

#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/nuttx.h>
#include <nuttx/kmalloc.h>
#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>
#include <nuttx/usb/mtp.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_BOARD_USBDEV_SERIALSTR
#include <nuttx/board.h>
#endif

#include "usbdev_fs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define USBMTP_CHARDEV_PATH        "/dev/mtp"

/* USB Controller */

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define USBMTP_SELFPOWERED       USB_CONFIG_ATTR_SELFPOWER
#else
#  define USBMTP_SELFPOWERED       (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define USBMTP_REMOTEWAKEUP      USB_CONFIG_ATTR_WAKEUP
#else
#  define USBMTP_REMOTEWAKEUP      (0)
#endif

/* Buffer big enough for any of our descriptors (the config descriptor is the
 * biggest).
 */

#define USBMTP_MXDESCLEN           (64)
#define USBMTP_MAXSTRLEN           (USBMTP_MXDESCLEN - 2)

/* Device descriptor values */

#define USBMTP_VERSIONNO           (0x0101) /* Device version number 1.1 (BCD) */

/* String language */

#define USBMTP_STR_LANGUAGE        (0x0409) /* en-us */

/* Descriptor strings.  If this MTP device is part of a composite device
 * then the manufacturer, product, and serial number strings will be provided
 * by the composite logic.
 */

#ifndef CONFIG_USBMTP_COMPOSITE
#  define USBMTP_MANUFACTURERSTRID (1)
#  define USBMTP_PRODUCTSTRID      (2)
#  define USBMTP_SERIALSTRID       (3)
#  define USBMTP_CONFIGSTRID       (4)
#  define USBMTP_INTERFACESTRID    (5)
#  define USBMTP_NSTRIDS           (5)
#else
#  define USBMTP_INTERFACESTRID    (1)
#  define USBMTP_NSTRIDS           (1)
#endif

#define USBMTP_NCONFIGS            (1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB descriptor ***********************************************************/

#ifndef CONFIG_USBMTP_COMPOSITE
static const struct usb_devdesc_s g_mtp_devdesc =
{
  .len = USB_SIZEOF_DEVDESC,         /* Descriptor length */
  .type = USB_DESC_TYPE_DEVICE,      /* Descriptor type */
  .usb =                             /* USB version */
  {
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  .classid = 0,                               /* Device class */
  .subclass = 0,                              /* Device sub-class */
  .protocol = 0,                              /* Device protocol */
  .mxpacketsize = CONFIG_USBMTP_EP0MAXPACKET, /* Max packet size (ep0) */
  .vendor =                                   /* Vendor ID */
  {
    LSBYTE(CONFIG_USBMTP_VENDORID),
    MSBYTE(CONFIG_USBMTP_VENDORID)
  },
  .product =                         /* Product ID */
  {
    LSBYTE(CONFIG_USBMTP_PRODUCTID),
    MSBYTE(CONFIG_USBMTP_PRODUCTID)
  },
  .device =                          /* Device ID */
  {
    LSBYTE(USBMTP_VERSIONNO),
    MSBYTE(USBMTP_VERSIONNO)
  },
  .imfgr = USBMTP_MANUFACTURERSTRID, /* Manufacturer */
  .iproduct = USBMTP_PRODUCTSTRID,   /* Product */
  .serno = USBMTP_SERIALSTRID,       /* Serial number */
  .nconfigs = USBMTP_NCONFIGS,       /* Number of configurations */
};

static const struct usb_cfgdesc_s g_mtp_cfgdesc =
{
  .len      = USB_SIZEOF_CFGDESC,   /* Descriptor length    */
  .type     = USB_DESC_TYPE_CONFIG, /* Descriptor type      */
  .cfgvalue = 1,                    /* Configuration value  */
  .icfg     = USBMTP_CONFIGSTRID,   /* Configuration        */
  .attr     = USB_CONFIG_ATTR_ONE |
              USBMTP_SELFPOWERED  |
              USBMTP_REMOTEWAKEUP,  /* Attributes           */
  .mxpower  = (CONFIG_USBDEV_MAXPOWER + 1) / 2
};

#  ifdef CONFIG_USBDEV_DUALSPEED
static const struct usb_qualdesc_s g_mtp_qualdesc =
{
  .len  = USB_SIZEOF_QUALDESC,                /* Descriptor length */
  .type = USB_DESC_TYPE_DEVICEQUALIFIER,      /* Descriptor type */
  .usb  =                                     /* USB version */
  {
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  .classid  = 0,                              /* Device class */
  .subclass = 0,                              /* Device sub-class */
  .protocol = 0,                              /* Device protocol */
  .mxpacketsize = CONFIG_USBMTP_EP0MAXPACKET, /* Max packet size (ep0) */
  .nconfigs = USBMTP_NCONFIGS,                /* Number of configurations */
  .reserved = 0,
};
#  endif

static const struct usbdev_strdesc_s g_mtp_strdesc[] =
{
  {USBMTP_MANUFACTURERSTRID, CONFIG_USBMTP_VENDORSTR},
  {USBMTP_PRODUCTSTRID,      CONFIG_USBMTP_PRODUCTSTR},
#  ifdef CONFIG_USBMTP_SERIALSTR
  {USBMTP_SERIALSTRID,       CONFIG_USBMTP_SERIALSTR},
#  else
  {USBMTP_SERIALSTRID,       ""},
#  endif
  {USBMTP_CONFIGSTRID,       CONFIG_USBMTP_CONFIGSTR},
  {}
};

static const struct usbdev_strdescs_s g_mtp_strdescs =
{
  .language = USBMTP_STR_LANGUAGE,
  .strdesc  = g_mtp_strdesc,
};

static const struct usbdev_devdescs_s g_mtp_devdescs =
{
  .cfgdesc  = &g_mtp_cfgdesc,
  .strdescs = &g_mtp_strdescs,
  .devdesc  = &g_mtp_devdesc,
#  ifdef CONFIG_USBDEV_DUALSPEED
  .qualdesc = &g_mtp_qualdesc,
#  endif
};
#endif

static const struct usb_ifdesc_s g_mtp_ifdesc =
{
  .len      = USB_SIZEOF_IFDESC,
  .type     = USB_DESC_TYPE_INTERFACE,
  .ifno     = 0,
  .alt      = 0,
  .neps     = 3,
  .classid  = USB_CLASS_STILL_IMAGE,
  .subclass = 0x01,
  .protocol = 0x01,
  .iif      = 0x05
};

static const struct usbdev_epinfo_s g_mtp_epbulkin =
{
  .desc =
    {
      .len       = USB_SIZEOF_EPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT,
      .addr      = USB_DIR_IN,
      .attr      = USB_EP_ATTR_XFER_BULK |
                   USB_EP_ATTR_NO_SYNC   |
                   USB_EP_ATTR_USAGE_DATA,
      .interval  = 0,
    },
  .reqnum        = CONFIG_USBMTP_NWRREQS,
  .fssize        = CONFIG_USBMTP_EPBULKIN_FSSIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
  .hssize        = CONFIG_USBMTP_EPBULKIN_HSSIZE,
#endif
#ifdef CONFIG_USBDEV_SUPERSPEED
  .sssize        = CONFIG_USBMTP_EPBULKIN_SSSIZE,
  .compdesc      =
    {
      .len       = USB_SIZEOF_SS_EPCOMPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT_COMPANION,
      .mxburst   = CONFIG_USBMTP_EPBULKIN_MAXBURST,
      .attr      = CONFIG_USBMTP_EPBULKIN_MAXSTREAM,
      .wbytes[0] = 0,
      .wbytes[1] = 0,
    },
#endif
};

static const struct usbdev_epinfo_s g_mtp_epbulkout =
{
  .desc =
    {
      .len       = USB_SIZEOF_EPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT,
      .addr      = USB_DIR_OUT,
      .attr      = USB_EP_ATTR_XFER_BULK |
                   USB_EP_ATTR_NO_SYNC   |
                   USB_EP_ATTR_USAGE_DATA,
      .interval  = 0,
    },
  .reqnum        = CONFIG_USBMTP_NRDREQS,
  .fssize        = CONFIG_USBMTP_EPBULKOUT_FSSIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
  .hssize        = CONFIG_USBMTP_EPBULKOUT_HSSIZE,
#endif
#ifdef CONFIG_USBDEV_SUPERSPEED
  .sssize        = CONFIG_USBMTP_EPBULKOUT_SSSIZE,
  .compdesc      =
    {
      .len       = USB_SIZEOF_SS_EPCOMPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT_COMPANION,
      .mxburst   = CONFIG_USBMTP_EPBULKOUT_MAXBURST,
      .attr      = CONFIG_USBMTP_EPBULKOUT_MAXSTREAM,
      .wbytes[0] = 0,
      .wbytes[1] = 0,
    },
#endif
};

static const struct usbdev_epinfo_s g_mtp_epintin =
{
  .desc =
    {
      .len       = USB_SIZEOF_EPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT,
      .addr      = USB_DIR_IN,
      .attr      = USB_EP_ATTR_XFER_INT |
                   USB_EP_ATTR_NO_SYNC   |
                   USB_EP_ATTR_USAGE_DATA,
      .interval  = CONFIG_USBMTP_EPINTIN_INTERVAL,
    },
  .reqnum        = CONFIG_USBMTP_NWRREQS,
  .fssize        = CONFIG_USBMTP_EPINTIN_SIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
  .hssize        = CONFIG_USBMTP_EPINTIN_SIZE,
#endif
#ifdef CONFIG_USBDEV_SUPERSPEED
  .sssize        = CONFIG_USBMTP_EPINTIN_SIZE,
  .compdesc      =
    {
      .len       = USB_SIZEOF_SS_EPCOMPDESC,
      .type      = USB_DESC_TYPE_ENDPOINT_COMPANION,
      .mxburst   = CONFIG_USBMTP_EPINTIN_MAXBURST,
      .attr      = 0,
      .wbytes[0] = LSBYTE((CONFIG_USBMTP_EPINTIN_MAXBURST + 1) *
                           CONFIG_USBMTP_EPINTIN_SIZE),
      .wbytes[1] = MSBYTE((CONFIG_USBMTP_EPINTIN_MAXBURST + 1) *
                           CONFIG_USBMTP_EPINTIN_SIZE),
    },
#endif
};

static const FAR struct usbdev_epinfo_s *g_mtp_epinfos[USBMTP_NUM_EPS] =
{
  &g_mtp_epbulkin,
  &g_mtp_epbulkout,
  &g_mtp_epintin,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: usbclass_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo,
                                  uint8_t speed, uint8_t type)
{
  FAR uint8_t *epdesc;
  FAR struct usb_ifdesc_s *dest;
  uint32_t totallen = 0;
  int ret;

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG && speed < USB_SPEED_SUPER)
    {
      speed = speed == USB_SPEED_HIGH ? USB_SPEED_FULL : USB_SPEED_HIGH;
    }

  dest = (FAR struct usb_ifdesc_s *)buf;
  epdesc = (FAR uint8_t *)(buf + sizeof(g_mtp_ifdesc));

  memcpy(dest, &g_mtp_ifdesc, sizeof(g_mtp_ifdesc));
  totallen += sizeof(struct usb_ifdesc_s);

  ret = usbdev_copy_epdesc((FAR struct usb_epdesc_s *)epdesc,
                           devinfo->epno[USBMTP_EP_BULKIN_IDX],
                           speed, &g_mtp_epbulkin);
  totallen += ret;
  epdesc += ret;

  ret = usbdev_copy_epdesc((FAR struct usb_epdesc_s *)epdesc,
                           devinfo->epno[USBMTP_EP_BULKOUT_IDX],
                           speed, &g_mtp_epbulkout);
  totallen += ret;
  epdesc += ret;

  ret = usbdev_copy_epdesc((FAR struct usb_epdesc_s *)epdesc,
                           devinfo->epno[USBMTP_EP_INTIN_IDX],
                           speed, &g_mtp_epintin);
  totallen += ret;
  epdesc += ret;

#ifdef CONFIG_USBMTP_COMPOSITE
  /* For composite device, apply possible offset to the interface numbers */

  dest->ifno = devinfo->ifnobase;
  dest->iif  = devinfo->strbase + USBMTP_INTERFACESTRID;
#endif

  return totallen;
}

/****************************************************************************
 * Name: usbclass_mkstrdesc
 *
 * Description:
 *   Construct the string descriptor
 *
 ****************************************************************************/

static int usbclass_mkstrdesc(uint8_t id, FAR struct usb_strdesc_s *strdesc)
{
  FAR uint8_t *data = (FAR uint8_t *)(strdesc + 1);
  FAR const char *str;
  int len;
  int ndata;
  int i;

  switch (id)
    {
      /* Composite driver removes offset before calling mkstrdesc() */

      case USBMTP_INTERFACESTRID:
        str = CONFIG_USBMTP_INTERFACESTR;
        break;

      default:
        return -EINVAL;
    }

  /* The string is utf16-le.  The poor man's utf-8 to utf16-le
   * conversion below will only handle 7-bit en-us ascii
   */

  len = strlen(str);
  if (len > (USBMTP_MAXSTRLEN / 2))
    {
      len = (USBMTP_MAXSTRLEN / 2);
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
 * Public Functions
 ****************************************************************************/

#ifndef CONFIG_USBMTP_COMPOSITE
/****************************************************************************
 * Name: usbdev_mtp_initialize
 *
 * Description:
 *   Initialize the Media Transfer protocol USB device driver.
 *
 * Returned Value:
 *   A non-NULL "handle" is returned on success.
 *
 ****************************************************************************/

FAR void *usbdev_mtp_initialize(void)
{
  struct composite_devdesc_s devdesc;

  usbdev_mtp_get_composite_devdesc(&devdesc);

  devdesc.devinfo.epno[USBMTP_EP_BULKIN_IDX] =
    USB_EPNO(CONFIG_USBMTP_EPBULKIN);
  devdesc.devinfo.epno[USBMTP_EP_BULKOUT_IDX] =
    USB_EPNO(CONFIG_USBMTP_EPBULKOUT);
  devdesc.devinfo.epno[USBMTP_EP_INTIN_IDX] =
    USB_EPNO(CONFIG_USBMTP_EPINTIN);

  return usbdev_fs_initialize(&g_mtp_devdescs, &devdesc);
}

/****************************************************************************
 * Name: usbdev_mtp_uninitialize
 *
 * Description:
 *   Uninitialize the Media Transfer protocol USB device driver.
 *
 ****************************************************************************/

void usbdev_mtp_uninitialize(FAR void *handle)
{
  usbdev_fs_uninitialize(handle);
}
#endif

/****************************************************************************
 * Name: usbdev_mtp_get_composite_devdesc
 *
 * Description:
 *   Helper function to fill in some constants into the composite
 *   configuration struct.
 *
 * Input Parameters:
 *     dev - Pointer to the configuration struct we should fill
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbdev_mtp_get_composite_devdesc(FAR struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  dev->mkconfdesc          = usbclass_mkcfgdesc;
  dev->mkstrdesc           = usbclass_mkstrdesc;
  dev->classobject         = usbdev_fs_classobject;
  dev->uninitialize        = usbdev_fs_classuninitialize;
  dev->nconfigs            = USBMTP_NCONFIGS;
  dev->configid            = 1;
#ifdef CONFIG_USBDEV_SUPERSPEED
  dev->cfgdescsize         = sizeof(g_mtp_ifdesc) +
                             USB_SIZEOF_EPDESC * 3 +
                             USB_SIZEOF_SS_EPCOMPDESC * 3;
#else
  dev->cfgdescsize         = sizeof(g_mtp_ifdesc) + 3 * USB_SIZEOF_EPDESC;
#endif
  dev->devinfo.ninterfaces = 1;
  dev->devinfo.nstrings    = USBMTP_NSTRIDS;
  dev->devinfo.nendpoints  = USBMTP_NUM_EPS;
  dev->devinfo.epinfos     = g_mtp_epinfos;
  dev->devinfo.name        = USBMTP_CHARDEV_PATH;
}
