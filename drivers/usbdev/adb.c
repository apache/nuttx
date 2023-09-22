/****************************************************************************
 * drivers/usbdev/adb.c
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
#include <nuttx/usb/adb.h>
#include <nuttx/fs/fs.h>

#ifdef CONFIG_BOARD_USBDEV_SERIALSTR
#include <nuttx/board.h>
#endif

#include "usbdev_fs.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* FIXME use minor for char device npath */

#ifdef CONFIG_USBFASTBOOT
#  define USBADB_CHARDEV_PATH      "/dev/fastboot"
#else
#  define USBADB_CHARDEV_PATH      "/dev/adb0"
#endif

/* USB Controller */

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define USBADB_SELFPOWERED       USB_CONFIG_ATTR_SELFPOWER
#else
#  define USBADB_SELFPOWERED       (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define USBADB_REMOTEWAKEUP      USB_CONFIG_ATTR_WAKEUP
#else
#  define USBADB_REMOTEWAKEUP      (0)
#endif

/* Buffer big enough for any of our descriptors (the config descriptor is the
 * biggest).
 */

#define USBADB_MXDESCLEN           (64)
#define USBADB_MAXSTRLEN           (USBADB_MXDESCLEN-2)

/* Device descriptor values */

#define USBADB_VERSIONNO           (0x0101) /* Device version number 1.1 (BCD) */

/* String language */

#define USBADB_STR_LANGUAGE        (0x0409) /* en-us */

/* Descriptor strings.  If there serial device is part of a composite device
 * then the manufacturer, product, and serial number strings will be provided
 * by the composite logic.
 */

#ifndef CONFIG_USBADB_COMPOSITE
#  define USBADB_MANUFACTURERSTRID (1)
#  define USBADB_PRODUCTSTRID      (2)
#  define USBADB_SERIALSTRID       (3)
#  define USBADB_CONFIGSTRID       (4)
#  define USBADB_INTERFACESTRID    (5)
#  define USBADB_NSTRIDS           (5)
#else
#  define USBADB_INTERFACESTRID    (1)
#  define USBADB_NSTRIDS           (1)
#endif

#define USBADB_NCONFIGS            (1)

#ifdef CONFIG_USBFASTBOOT
#  define USBADB_INTERFACEPROTOCOL (3)
#else
#  define USBADB_INTERFACEPROTOCOL (1)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* USB descriptor ***********************************************************/

#ifndef CONFIG_USBADB_COMPOSITE
static const struct usb_devdesc_s g_adb_devdesc =
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
  .mxpacketsize = CONFIG_USBADB_EP0MAXPACKET, /* Max packet size (ep0) */
  .vendor =                                   /* Vendor ID */
  {
    LSBYTE(CONFIG_USBADB_VENDORID),
    MSBYTE(CONFIG_USBADB_VENDORID)
  },
  .product =                         /* Product ID */
  {
    LSBYTE(CONFIG_USBADB_PRODUCTID),
    MSBYTE(CONFIG_USBADB_PRODUCTID)
  },
  .device =                          /* Device ID */
  {
    LSBYTE(USBADB_VERSIONNO),
    MSBYTE(USBADB_VERSIONNO)
  },
  .imfgr = USBADB_MANUFACTURERSTRID, /* Manufacturer */
  .iproduct = USBADB_PRODUCTSTRID,   /* Product */
  .serno = USBADB_SERIALSTRID,       /* Serial number */
  .nconfigs = USBADB_NCONFIGS,       /* Number of configurations */
};

#  ifdef CONFIG_USBDEV_DUALSPEED
static const struct usb_qualdesc_s g_adb_qualdesc =
{
  USB_SIZEOF_QUALDESC,               /* len */
  USB_DESC_TYPE_DEVICEQUALIFIER,     /* type */
  {                                  /* usb */
    LSBYTE(0x0200),
    MSBYTE(0x0200)
  },
  0,                                 /* classid */
  0,                                 /* subclass */
  0,                                 /* protocol */
  CONFIG_USBADB_EP0MAXPACKET,        /* mxpacketsize */
  USBADB_NCONFIGS,                   /* nconfigs */
  0,                                 /* reserved */
};
#  endif

static const struct usbdev_strdesc_s g_adb_strdesc[] =
{
  {USBADB_MANUFACTURERSTRID, CONFIG_USBADB_VENDORSTR},
  {USBADB_PRODUCTSTRID,      CONFIG_USBADB_PRODUCTSTR},
#  ifdef CONFIG_USBADB_SERIALSTR
  {USBADB_SERIALSTRID,       CONFIG_USBADB_SERIALSTR},
#  else
  {USBADB_SERIALSTRID,       ""},
#  endif
  {USBADB_CONFIGSTRID,       CONFIG_USBADB_CONFIGSTR},
  {}
};

static const struct usbdev_strdescs_s g_adb_strdescs =
{
  .language = USBADB_STR_LANGUAGE,
  .strdesc  = g_adb_strdesc,
};

static const struct usb_cfgdesc_s g_adb_cfgdesc =
{
  .len      = USB_SIZEOF_CFGDESC,   /* Descriptor length    */
  .type     = USB_DESC_TYPE_CONFIG, /* Descriptor type      */
  .cfgvalue = 1,                    /* Configuration value  */
  .icfg     = USBADB_CONFIGSTRID,   /* Configuration        */
  .attr     = USB_CONFIG_ATTR_ONE |
              USBADB_SELFPOWERED  |
              USBADB_REMOTEWAKEUP,  /* Attributes           */

  .mxpower  = (CONFIG_USBDEV_MAXPOWER + 1) / 2 /* Max power (mA/2) */
};

static const struct usbdev_devdescs_s g_adb_devdescs =
{
  .cfgdesc  = &g_adb_cfgdesc,
  .strdescs = &g_adb_strdescs,
  .devdesc  = &g_adb_devdesc,
#ifdef CONFIG_USBDEV_DUALSPEED
  .qualdesc = &g_adb_qualdesc,
#endif
};
#endif

static const struct usb_ifdesc_s g_adb_ifdesc =
{
  .len      = USB_SIZEOF_IFDESC,
  .type     = USB_DESC_TYPE_INTERFACE,
  .ifno     = 0,
  .alt      = 0,
  .neps     = 2,
  .classid  = USB_CLASS_VENDOR_SPEC,
  .subclass = 0x42,
  .protocol = USBADB_INTERFACEPROTOCOL,
  .iif      = USBADB_INTERFACESTRID
};

static const struct usbdev_epinfo_s g_adb_epbulkin =
{
  .desc =
    {
      .len      = USB_SIZEOF_EPDESC,
      .type     = USB_DESC_TYPE_ENDPOINT,
      .addr     = USB_DIR_IN,
      .attr     = USB_EP_ATTR_XFER_BULK |
                  USB_EP_ATTR_NO_SYNC   |
                  USB_EP_ATTR_USAGE_DATA,
      .interval = 0,
    },
  .fssize = CONFIG_USBADB_EPBULKIN_FSSIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
  .hssize = CONFIG_USBADB_EPBULKIN_HSSIZE,
#endif
  .reqnum = CONFIG_USBADB_NWRREQS,
};

static const struct usbdev_epinfo_s g_adb_epbulkout =
{
  .desc =
    {
      .len      = USB_SIZEOF_EPDESC,
      .type     = USB_DESC_TYPE_ENDPOINT,
      .addr     = USB_DIR_OUT,
      .attr     = USB_EP_ATTR_XFER_BULK |
                  USB_EP_ATTR_NO_SYNC   |
                  USB_EP_ATTR_USAGE_DATA,
      .interval = 0,
    },
  .fssize = CONFIG_USBADB_EPBULKOUT_FSSIZE,
#ifdef CONFIG_USBDEV_DUALSPEED
  .hssize = CONFIG_USBADB_EPBULKOUT_HSSIZE,
#endif
  .reqnum = CONFIG_USBADB_NRDREQS,
};

static const FAR struct usbdev_epinfo_s *g_adb_epinfos[USBADB_NUM_EPS] =
{
  &g_adb_epbulkin,
  &g_adb_epbulkout,
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

#ifdef CONFIG_USBDEV_DUALSPEED
static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo,
                                  uint8_t speed, uint8_t type)
#else
static int16_t usbclass_mkcfgdesc(FAR uint8_t *buf,
                                  FAR struct usbdev_devinfo_s *devinfo)
#endif
{
  bool hispeed = false;
  FAR struct usb_epdesc_s *epdesc;
  FAR struct usb_ifdesc_s *dest;

#ifdef CONFIG_USBDEV_DUALSPEED
  hispeed = (speed == USB_SPEED_HIGH);

  /* Check for switches between high and full speed */

  if (type == USB_DESC_TYPE_OTHERSPEEDCONFIG)
    {
      hispeed = !hispeed;
    }
#endif

  dest = (FAR struct usb_ifdesc_s *)buf;
  epdesc = (FAR struct usb_epdesc_s *)(buf + sizeof(g_adb_ifdesc));

  memcpy(dest, &g_adb_ifdesc, sizeof(g_adb_ifdesc));

  usbdev_copy_epdesc(&epdesc[0], devinfo->epno[USBADB_EP_BULKIN_IDX],
                     hispeed, &g_adb_epbulkin);
  usbdev_copy_epdesc(&epdesc[1], devinfo->epno[USBADB_EP_BULKOUT_IDX],
                     hispeed, &g_adb_epbulkout);

#ifdef CONFIG_USBADB_COMPOSITE
  /* For composite device, apply possible offset to the interface numbers */

  dest->ifno = devinfo->ifnobase;
  dest->iif  = devinfo->strbase + USBADB_INTERFACESTRID;
#endif

  return sizeof(g_adb_ifdesc) + 2 * USB_SIZEOF_EPDESC;
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

      case USBADB_INTERFACESTRID:
        str = CONFIG_USBADB_INTERFACESTR;
        break;

      default:
        return -EINVAL;
    }

  /* The string is utf16-le.  The poor man's utf-8 to utf16-le
   * conversion below will only handle 7-bit en-us ascii
   */

  len = strlen(str);
  if (len > (USBADB_MAXSTRLEN / 2))
    {
      len = (USBADB_MAXSTRLEN / 2);
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

#ifndef CONFIG_USBADB_COMPOSITE
/****************************************************************************
 * Name: usbdev_adb_initialize
 *
 * Description:
 *   Initialize the Android Debug Bridge USB device driver.
 *
 * Returned Value:
 *   A non-NULL "handle" is returned on success.
 *
 ****************************************************************************/

FAR void *usbdev_adb_initialize(void)
{
  struct composite_devdesc_s devdesc;

  usbdev_adb_get_composite_devdesc(&devdesc);

  devdesc.devinfo.epno[USBADB_EP_BULKIN_IDX] =
    USB_EPNO(CONFIG_USBADB_EPBULKIN);
  devdesc.devinfo.epno[USBADB_EP_BULKOUT_IDX] =
    USB_EPNO(CONFIG_USBADB_EPBULKOUT);

  return usbdev_fs_initialize(&g_adb_devdescs, &devdesc);
}

/****************************************************************************
 * Name: usbdev_adb_uninitialize
 *
 * Description:
 *   Uninitialize the Android Debug Bridge USB device driver.
 *
 ****************************************************************************/

void usbdev_adb_uninitialize(FAR void *handle)
{
  usbdev_fs_uninitialize(handle);
}
#endif

/****************************************************************************
 * Name: usbdev_adb_get_composite_devdesc
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void usbdev_adb_get_composite_devdesc(FAR struct composite_devdesc_s *dev)
{
  memset(dev, 0, sizeof(struct composite_devdesc_s));

  dev->classobject         = usbdev_fs_classobject;
  dev->uninitialize        = usbdev_fs_classuninitialize;
  dev->mkconfdesc          = usbclass_mkcfgdesc,
  dev->mkstrdesc           = usbclass_mkstrdesc,
  dev->nconfigs            = USBADB_NCONFIGS;
  dev->configid            = 1;
  dev->cfgdescsize         = sizeof(g_adb_ifdesc) + 2 * USB_SIZEOF_EPDESC;
  dev->devinfo.ninterfaces = 1;
  dev->devinfo.nstrings    = USBADB_NSTRIDS;
  dev->devinfo.nendpoints  = USBADB_NUM_EPS;
  dev->devinfo.epinfos     = g_adb_epinfos;
  dev->devinfo.name        = USBADB_CHARDEV_PATH;
}
