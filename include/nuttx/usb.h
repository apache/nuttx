/************************************************************************************
 * include/nuttx/usb.h
 *
 *   Copyright (C) 2008 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <spudmonkey@racsa.co.cr>
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
 ************************************************************************************/

#ifndef __NUTTX_USB_H
#define __NUTTX_USB_H

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

/************************************************************************************
 * Definitions
 ************************************************************************************/

/* All 16-bit values must be little-endian */

#define MSBYTE(u16)                             ((u16) >> 8)     /* Get MS byte from uint16 */
#define LSBYTE(u16)                             ((u16) & 0xff)   /* Get LS byte from uint16 */

#define GETUINT16(p)                            (((uint16)p[1] << 8)|(uint16)p[1])

/* USB directions (in endpoint addresses) */

#define USB_DIR_OUT                             (0x00) /* To the device */
#define USB_DIR_IN                              (0x80) /* To the host */

#define USB_EPNO(addr)                          ((addr)&0x7f)
#define USB_EPOUT(addr)                         ((addr)|USB_DIR_OUT)
#define USB_EPIN(addr)                          ((addr)|USB_DIR_IN)

/* Standard requests */

#define USB_REQ_GETSTATUS                       (0x00)
#define USB_REQ_CLEARFEATURE                    (0x01)
#define USB_REQ_SETFEATURE                      (0x03)
#define USB_REQ_SETADDRESS                      (0x05)
#define USB_REQ_GETDESCRIPTOR                   (0x06)
#define USB_REQ_SETDESCRIPTOR                   (0x07)
#define USB_REQ_GETCONFIGURATION                (0x08)
#define USB_REQ_SETCONFIGURATION                (0x09)
#define USB_REQ_GETINTERFACE                    (0x0a)
#define USB_REQ_SETINTERFACE                    (0x0b)
#define USB_REQ_SYNCHFRAME                      (0x0c)

#define USB_REQ_SETENCRYPTION                   (0x0d) /* Wireless USB */
#define USB_REQ_GETENCRYPTION                   (0x0e)
#define USB_REQ_SETHANDSHAKE                    (0x0f)
#define USB_REQ_GETHANDSHAKE                    (0x10)
#define USB_REQ_SETCONNECTION                   (0x11)
#define USB_REQ_SETSECURITYDATA                 (0x12)
#define USB_REQ_GETSECURITYDATA                 (0x13)
#define USB_REQ_SETWUSBDATA                     (0x14)
#define USB_REQ_LOOPBACKDATAWRITE               (0x15)
#define USB_REQ_LOOPBACKDATAREAD                (0x16)
#define USB_REQ_SETINTERFACEDS                  (0x17)

/* Request type encoding */

#define USB_REQ_TYPE_MASK                       (0x60)
#define USB_REQ_TYPE_STANDARD                   (0x00)
#define USB_REQ_TYPE_CLASS                      (0x20)
#define USB_REQ_TYPE_VENDOR                     (0x40)

#define USB_REQ_RECIPIENT_MASK                  (0x1f)
#define USB_REQ_RECIPIENT_DEVICE                (0x00)
#define USB_REQ_RECIPIENT_INTERFACE             (0x01)
#define USB_REQ_RECIPIENT_ENDPOINT              (0x02)
#define USB_REQ_RECIPIENT_OTHER                 (0x03)
#define USB_REQ_TYPE_MASK                       (0x60)
#define USB_REQ_DIR_IN                          (0x80)

/* USB feature values */

#define USB_FEATURE_SELFPOWERED                  0
#define USB_FEATURE_REMOTEWAKEUP                 1
#define USB_FEATURE_TESTMODE                     2
#define USB_FEATURE_BATTERY                      2
#define USB_FEATURE_BHNPENABLE                   3
#define USB_FEATURE_WUSBDEVICE                   3
#define USB_FEATURE_AHNPSUPPORT                  4
#define USB_FEATURE_AALTHNPSUPPORT               5
#define USB_FEATURE_DEBUGMODE                    6

#define USB_ENDPOINT_HALT                        0

/* Generic descriptor header offsets */

#define USB_DESC_DESCLENOFFSET                   0
#define USB_DESC_DESCTYPEOFFSET                  1

/* Descriptor types */

#define USB_DESC_TYPE_DEVICE                    (0x01)
#define USB_DESC_TYPE_CONFIG                    (0x02)
#define USB_DESC_TYPE_STRING                    (0x03)
#define USB_DESC_TYPE_INTERFACE                 (0x04)
#define USB_DESC_TYPE_ENDPOINT                  (0x05)
#define USB_DESC_TYPE_DEVICEQUALIFIER           (0x06)
#define USB_DESC_TYPE_OTHERSPEEDCONFIG          (0x07)
#define USB_DESC_TYPE_INTERFACEPOWER            (0x08)
#define USB_DESC_TYPE_OTG                       (0x09)
#define USB_DESC_TYPE_DEBUG                     (0x0a)
#define USB_DESC_TYPE_INTERFACEASSOCIATION      (0x0b)
#define USB_DESC_TYPE_SECURITY                  (0x0c)
#define USB_DESC_TYPE_KEY                       (0x0d)
#define USB_DESC_TYPE_ENCRYPTION_TYPE           (0x0e)
#define USB_DESC_TYPE_BOS                       (0x0f)
#define USB_DESC_TYPE_DEVICECAPABILITY          (0x10)
#define USB_DESC_TYPE_WIRELESS_ENDPOINTCOMP     (0x11)
#define USB_DESC_TYPE_CSDEVICE                  (0x21)
#define USB_DESC_TYPE_CSCONFIG                  (0x22)
#define USB_DESC_TYPE_CSSTRING                  (0x23)
#define USB_DESC_TYPE_CSINTERFACE               (0x24)
#define USB_DESC_TYPE_CSENDPOINT                (0x25)

/* Device and interface descriptor class codes */

#define USB_CLASS_PER_INTERFACE                 (0x00)
#define USB_CLASS_AUDIO                         (0x01)
#define USB_CLASS_COMM                          (0x02)
#define USB_CLASS_HID                           (0x03)
#define USB_CLASS_PHYSICAL                      (0x05)
#define USB_CLASS_STILL_IMAGE                   (0x06)
#define USB_CLASS_PRINTER                       (0x07)
#define USB_CLASS_MASS_STORAGE                  (0x08)
#define USB_CLASS_HUB                           (0x09)
#define USB_CLASS_CDC_DATA                      (0x0a)
#define USB_CLASS_CSCID                         (0x0b)
#define USB_CLASS_CONTENT_SEC                   (0x0d)
#define USB_CLASS_VIDEO                         (0x0e)
#define USB_CLASS_WIRELESS_CONTROLLER           (0xe0)
#define USB_CLASS_APP_SPEC                      (0xfe)
#define USB_CLASS_VENDOR_SPEC                   (0xff)

/* Values for configuration descriptor attributes */

#define USB_CONFIG_ATTR_BATTERY                 (0x10) /* Battery powered */
#define USB_CONFIG_ATTR_WAKEUP                  (0x20) /* Remote wakeup */
#define USB_CONFIG_ATTR_SELFPOWER               (0x40) /* Self-powered */
#define USB_CONFIG_ATTR_ONE                     (0x80) /* Must be one */

/* Endpoint descriptor address */

#define USB_EP_ADDR_NUMBER_MASK                 (0x0f)
#define USB_EP_ADDR_DIR_MASK                    (0x80)

/* Endpoint descriptor attributes */

#define USB_EP_ATTR_XFERTYPE_MASK               (0x03)
#define USB_EP_ATTR_XFER_CONTROL                (0x00)
#define USB_EP_ATTR_XFER_ISOC                   (0x01)
#define USB_EP_ATTR_XFER_BULK                   (0x02)
#define USB_EP_ATTR_XFER_INT                    (0x03)
#define USB_EP_ATTR_MAX_ADJUSTABLE              (0x80)

/************************************************************************************
 * Public Types
 ************************************************************************************/

/* This structure is used to send control requests to a USB device. */

struct usb_ctrlreq_s
{
  ubyte type;                /* Matches request type */
  ubyte req;                 /* Matches request field */
  ubyte value[2];
  ubyte index[2];
  ubyte len[2];
};

/* Generic descriptor */

struct usb_desc_s
{
  ubyte len;               /* Descriptor length */
  ubyte type;              /* Descriptor type */
};

/* Device descriptor */

struct usb_devdesc_s
{
  ubyte len;               /* Descriptor length */
  ubyte type;              /* Descriptor type */
  ubyte usb[2];            /* USB version */
  ubyte class;             /* Device class */
  ubyte subclass;          /* Device sub-class */
  ubyte protocol;          /* Device protocol */
  ubyte mxpacketsize;      /* Max packet size (ep0) */
  ubyte vender[2];         /* Vendor ID */
  ubyte product[2];        /* Product ID */
  ubyte device[2];         /* Device ID */
  ubyte imfgr;             /* Manufacturer */
  ubyte iproduct;          /* Product */
  ubyte serno;             /* Serial number */
  ubyte nconfigs;          /* Number of configurations */
};

/* Configuration descriptor */

struct usb_cfgdesc_s
{
  ubyte len;               /* Descriptor length */
  ubyte type;              /* Descriptor type */
  ubyte totallen[2];       /* Total length */
  ubyte ninterfaces;       /* Number of interfaces */
  ubyte cfgvalue;          /* Configuration value */
  ubyte icfg;              /* Configuration */
  ubyte attr;              /* Attributes */
  ubyte mxpower;           /* Max power (mA/2) */
};

/* String descriptor */

struct usb_strdesc_s
{
  ubyte len;               /* Descriptor length */
  ubyte type;              /* Descriptor type */
  ubyte data[2];
};

/* Interface descriptor */

struct usb_ifdesc_s
{
  ubyte len;               /* Descriptor length */
  ubyte type;              /* Descriptor type */
  ubyte ifno;              /* Interface number */
  ubyte alt;               /* Alternate setting */
  ubyte neps;              /* Number of endpoints */
  ubyte class;             /* Interface class */
  ubyte subclass;          /* Interface sub-class */
  ubyte protocol;          /* Interface protocol */
  ubyte iif;               /* iInterface */
};

/* Endpoint descriptor */

struct usb_epdesc_s
{
  ubyte  len;               /* Descriptor length */
  ubyte  type;              /* Descriptor type */
  ubyte  addr;              /* Endpoint address */
  ubyte  attr;              /* Endpoint attributes */
  ubyte  mxpacketsize[2];   /* Maximum packet size */
  ubyte  interval;          /* Interval */
};

struct usb_audioepdesc_s
{
  struct usb_epdesc_s ep;
  ubyte  refresh;
  ubyte  synchaddr;
};

/* Device qualifier descriptor */

struct usb_qualdesc_s
{
  ubyte  len;               /* Descriptor length */
  ubyte  type;              /* Descriptor type */
  ubyte  usb[2];            /* USB version */
  ubyte  class;             /* Qualifier class */
  ubyte  subclass;          /* Qualifier sub-class */
  ubyte  protocol;          /* Qualifier protocol */
  ubyte  mxpacketsize;      /* Max packet size (ep0) */
  ubyte  nconfigs;          /* Number of configurations */
  ubyte  reserved;
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

/************************************************************************************
 * Public Data
 ************************************************************************************/

extern const struct usb_devdesc_s UsbStandardDeviceDescriptorStr;
extern const ubyte UsbStandardConfigurationDescriptor[];
extern const ubyte UsbLanguagesStr[];
extern const ubyte *const *const UsbString[];

/************************************************************************************
 * Private Functions
 ************************************************************************************/

/************************************************************************************
 * Public Functions
 ************************************************************************************/

#endif // __NUTTX_USB_H
