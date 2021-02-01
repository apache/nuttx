/****************************************************************************
 * include/nuttx/usb/usb.h
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

#ifndef __INCLUDE_NUTTX_USB_USB_H
#define __INCLUDE_NUTTX_USB_USB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* A packet identifier (PID) immediately follows the SYNC field of every USB
 * packet.
 * A PID consists of a four-bit packet type field followed by a four-bit
 * check field USB Tokens (See Table 8-1 in the USB specification)
 */

#define USB_PID_OUT_TOKEN                       (0x01) /* Tokens */
#define USB_PID_IN_TOKEN                        (0x09)
#define USB_PID_SOF_TOKEN                       (0x05)
#define USB_PID_SETUP_TOKEN                     (0x0d)

#define USB_PID_DATA0                           (0x03) /* Data */
#define USB_PID_DATA1                           (0x0b)
#define USB_PID_DATA2                           (0x07)
#define USB_PID_MDATA                           (0x0f)

#define USB_PID_ACK                             (0x02) /* Handshake */
#define USB_PID_NAK                             (0x0a)
#define USB_PID_STALL                           (0x0e)
#define USB_PID_NYET                            (0x06)

#define USB_PID_PRE_TOKEN                       (0x0c) /* Special */
#define USB_PID_ERR                             (0x0c)
#define USB_PID_SPLIT_TOKEN                     (0x08)
#define USB_PID_PING_TOKEN                      (0x04)
#define USB_PID_RESERVED                        (0x00)

/* All 16-bit values must be little-endian */

#define MSBYTE(u16)                             ((u16) >> 8)     /* Get MS byte from uint16_t */
#define LSBYTE(u16)                             ((u16) & 0xff)   /* Get LS byte from uint16_t */

#define GETUINT16(p)                            (((uint16_t)p[1] << 8) | (uint16_t)p[0])
#define GETUINT32(p)                            (((uint32_t)p[3] << 24) | \
                                                 ((uint32_t)p[2] << 16) | \
                                                 ((uint32_t)p[1] << 8) | (uint32_t)p[0])

/* USB directions (in endpoint addresses) */

#define USB_DIR_MASK                            (0x80)
#define USB_EPNO_MASK                           (0x7f)
#define USB_DIR_OUT                             (0x00) /* host-to-device */
#define USB_DIR_IN                              (0x80) /* device-to-host */

#define USB_EPNO(addr)                          ((addr) & USB_EPNO_MASK)
#define USB_EPOUT(addr)                         ((addr) | USB_DIR_OUT)
#define USB_EPIN(addr)                          ((addr) | USB_DIR_IN)
#define USB_ISEPIN(addr)                        (((addr) & USB_DIR_MASK) == USB_DIR_IN)
#define USB_ISEPOUT(addr)                       (((addr) & USB_DIR_MASK) == USB_DIR_OUT)

/* Control Setup Packet.  Byte 0 = Request type */

#define USB_REQ_DIR_MASK                        (1 << 7) /* Bit 7=1: Direction bit */
#define USB_REQ_DIR_IN                          (1 << 7) /* Bit 7=1: Device-to-host */
#define USB_REQ_DIR_OUT                         (0 << 7) /* Bit 7=0: Host-to-device */

#define USB_REQ_ISIN(type)                      (((type) & USB_REQ_DIR_MASK) != 0)
#define USB_REQ_ISOUT(type)                     (((type) & USB_REQ_DIR_MASK) == 0)

#define USB_REQ_TYPE_SHIFT                      (5) /* Bits 5:6: Request type */
#  define USB_REQ_TYPE_MASK                     (3 << USB_REQ_TYPE_SHIFT)
#  define USB_REQ_TYPE_STANDARD                 (0 << USB_REQ_TYPE_SHIFT)
#  define USB_REQ_TYPE_CLASS                    (1 << USB_REQ_TYPE_SHIFT)
#  define USB_REQ_TYPE_VENDOR                   (2 << USB_REQ_TYPE_SHIFT)

#define USB_REQ_RECIPIENT_SHIFT                 (0) /* Bits 0:4: Recipient */
#define USB_REQ_RECIPIENT_MASK                  (0x1f << USB_REQ_RECIPIENT_SHIFT)
#  define USB_REQ_RECIPIENT_DEVICE              (0 << USB_REQ_RECIPIENT_SHIFT)
#  define USB_REQ_RECIPIENT_INTERFACE           (1 << USB_REQ_RECIPIENT_SHIFT)
#  define USB_REQ_RECIPIENT_ENDPOINT            (2 << USB_REQ_RECIPIENT_SHIFT)
#  define USB_REQ_RECIPIENT_OTHER               (3 << USB_REQ_RECIPIENT_SHIFT)

/* Control Setup Packet.  Byte 1 = Standard Request Codes */

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

/* USB feature values */

#define USB_FEATURE_ENDPOINTHALT                 0
#define USB_FEATURE_SELFPOWERED                  0
#define USB_FEATURE_REMOTEWAKEUP                 1
#define USB_FEATURE_TESTMODE                     2
#define USB_FEATURE_BATTERY                      2
#define USB_FEATURE_BHNPENABLE                   3
#define USB_FEATURE_WUSBDEVICE                   3
#define USB_FEATURE_AHNPSUPPORT                  4
#define USB_FEATURE_AALTHNPSUPPORT               5
#define USB_FEATURE_DEBUGMODE                    6

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
#define USB_CLASS_CDC                           (0x02)
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
#define USB_CLASS_MISC                          (0xef)
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

#define USB_EP_ATTR_XFERTYPE_SHIFT              (0)
#define USB_EP_ATTR_XFERTYPE_MASK               (3 << USB_EP_ATTR_XFERTYPE_SHIFT)
#  define USB_EP_ATTR_XFER_CONTROL              (0 << USB_EP_ATTR_XFERTYPE_SHIFT)
#  define USB_EP_ATTR_XFER_ISOC                 (1 << USB_EP_ATTR_XFERTYPE_SHIFT)
#  define USB_EP_ATTR_XFER_BULK                 (2 << USB_EP_ATTR_XFERTYPE_SHIFT)
#  define USB_EP_ATTR_XFER_INT                  (3 << USB_EP_ATTR_XFERTYPE_SHIFT)
#define USB_EP_ATTR_SYNC_SHIFT                  (2)
#define USB_EP_ATTR_SYNC_MASK                   (3 << USB_EP_ATTR_SYNC_SHIFT)
#  define USB_EP_ATTR_NO_SYNC                   (0 << USB_EP_ATTR_SYNC_SHIFT)
#  define USB_EP_ATTR_ASYNC                     (1 << USB_EP_ATTR_SYNC_SHIFT)
#  define USB_EP_ATTR_ADAPTIVE                  (2 << USB_EP_ATTR_SYNC_SHIFT)
#  define USB_EP_ATTR_SYNC                      (3 << USB_EP_ATTR_SYNC_SHIFT)
#define USB_EP_ATTR_USAGE_SHIFT                 (4)
#define USB_EP_ATTR_USAGE_MASK                  (3 << USB_EP_ATTR_USAGE_SHIFT)
#  define USB_EP_ATTR_USAGE_DATA                (0 << USB_EP_ATTR_USAGE_SHIFT)
#  define USB_EP_ATTR_USAGE_FEEDBACK            (1 << USB_EP_ATTR_USAGE_SHIFT)
#  define USB_EP_ATTR_USAGE_IMPLICIT            (2 << USB_EP_ATTR_USAGE_SHIFT)
#define USB_EP_ATTR_MAX_ADJUSTABLE              (1 << 7)

/* OTG Definitions */

/* OTG SET FEATURE Constants */

#define USBOTG_FEATURE_B_HNP_ENABLE             3  /* Enable B device to perform HNP */
#define USBOTG_FEATURE_A_HNP_SUPPORT            4  /* A device supports HNP */
#define USBOTG_FEATURE_A_ALT_HNP_SUPPORT        5  /* Another port on the A device supports HNP */

/* Device speeds */

#define USB_SPEED_UNKNOWN                       0 /* Transfer rate not yet set */
#define USB_SPEED_LOW                           1 /* USB 1.1 */
#define USB_SPEED_FULL                          2 /* USB 1.1 */
#define USB_SPEED_HIGH                          3 /* USB 2.0 */
#define USB_SPEED_VARIABLE                      4 /* Wireless USB 2.5 */

/* Maximum number of devices per controller */

#define USB_MAX_DEVICES                         (127)

/* Microsoft OS Descriptor specific values */

#define USB_REQ_GETMSFTOSDESCRIPTOR             (0xEE)
#define MSFTOSDESC_INDEX_FUNCTION               4
#define MSFTOSDESC_INDEX_EXTPROP                5

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* This structure is used to send control requests to a USB device. */

struct usb_ctrlreq_s
{
  uint8_t type;                /* Matches request type */
  uint8_t req;                 /* Matches request field */
  uint8_t value[2];
  uint8_t index[2];
  uint8_t len[2];
};

#define USB_SIZEOF_CTRLREQ 8

/* Generic descriptor */

struct usb_desc_s
{
  uint8_t len;               /* Descriptor length */
  uint8_t type;              /* Descriptor type */
};

/* Device descriptor */

struct usb_devdesc_s
{
  uint8_t len;               /* Descriptor length */
  uint8_t type;              /* Descriptor type */
  uint8_t usb[2];            /* USB version */
  uint8_t classid;           /* Device class */
  uint8_t subclass;          /* Device sub-class */
  uint8_t protocol;          /* Device protocol */
  uint8_t mxpacketsize;      /* Max packet size (ep0) */
  uint8_t vendor[2];         /* Vendor ID */
  uint8_t product[2];        /* Product ID */
  uint8_t device[2];         /* Device ID */
  uint8_t imfgr;             /* Manufacturer */
  uint8_t iproduct;          /* Product */
  uint8_t serno;             /* Serial number */
  uint8_t nconfigs;          /* Number of configurations */
};

#define USB_SIZEOF_DEVDESC 18

/* Configuration descriptor */

struct usb_cfgdesc_s
{
  uint8_t len;               /* Descriptor length */
  uint8_t type;              /* Descriptor type */
  uint8_t totallen[2];       /* Total length */
  uint8_t ninterfaces;       /* Number of interfaces */
  uint8_t cfgvalue;          /* Configuration value */
  uint8_t icfg;              /* Configuration */
  uint8_t attr;              /* Attributes */
  uint8_t mxpower;           /* Max power (mA/2) */
};

#define USB_SIZEOF_CFGDESC 9

struct usb_otherspeedconfigdesc_s
{
  uint8_t  len;               /* Descriptor length */
  uint8_t  type;              /* Descriptor type */
  uint8_t  totallen[2];       /* Total length */
  uint8_t  ninterfaces;       /* Number of interfaces */
  uint8_t  cfgvalue;          /* Configuration value */
  uint8_t  icfg;              /* Configuration */
  uint8_t  attr;              /* Attributes */
  uint8_t  mxpower;           /* Max power (mA/2) */
};

#define USB_SIZEOF_OTHERSPEEDCONFIGDESC 9

/* String descriptor */

struct usb_strdesc_s
{
  uint8_t len;               /* Descriptor length */
  uint8_t type;              /* Descriptor type */
  uint8_t data[2];
};

/* Interface descriptor */

struct usb_ifdesc_s
{
  uint8_t len;               /* Descriptor length */
  uint8_t type;              /* Descriptor type */
  uint8_t ifno;              /* Interface number */
  uint8_t alt;               /* Alternate setting */
  uint8_t neps;              /* Number of endpoints */
  uint8_t classid;           /* Interface class */
  uint8_t subclass;          /* Interface sub-class */
  uint8_t protocol;          /* Interface protocol */
  uint8_t iif;               /* iInterface */
};

#define USB_SIZEOF_IFDESC 9

/* Endpoint descriptor */

struct usb_epdesc_s
{
  uint8_t  len;               /* Descriptor length */
  uint8_t  type;              /* Descriptor type */
  uint8_t  addr;              /* Endpoint address */
  uint8_t  attr;              /* Endpoint attributes */
  uint8_t  mxpacketsize[2];   /* Maximum packet size */
  uint8_t  interval;          /* Interval */
};

#define USB_SIZEOF_EPDESC 7

struct usb_audioepdesc_s
{
  struct usb_epdesc_s ep;
  uint8_t  refresh;
  uint8_t  synchaddr;
};

#define USB_SIZEOF_AUDIOEPDESC 9

/* Device qualifier descriptor */

struct usb_qualdesc_s
{
  uint8_t  len;               /* Descriptor length */
  uint8_t  type;              /* Descriptor type */
  uint8_t  usb[2];            /* USB version */
  uint8_t  classid;           /* Qualifier class */
  uint8_t  subclass;          /* Qualifier sub-class */
  uint8_t  protocol;          /* Qualifier protocol */
  uint8_t  mxpacketsize;      /* Max packet size (ep0) */
  uint8_t  nconfigs;          /* Number of configurations */
  uint8_t  reserved;
};

#define USB_SIZEOF_QUALDESC 10

/* Interface association descriptor
 *
 * The Universal Serial Bus Specification, revision 2.0, does not support
 * grouping more than one interface of a composite device within a single
 * function. However, the USB Device Working Group (DWG) created USB device
 * classes that allow for functions with multiple interfaces, and the USB
 * Implementor's Forum issued an Engineering Change Notification (ECN) that
 * defines a mechanism for grouping interfaces.
 */

struct usb_iaddesc_s
{
  uint8_t  len;               /* Descriptor length */
  uint8_t  type;              /* Descriptor type */
  uint8_t  firstif;           /* Number of first interface of the function */
  uint8_t  nifs;              /* Number of interfaces associated with the function */
  uint8_t  classid;           /* Class code */
  uint8_t  subclass;          /* Sub-class code */
  uint8_t  protocol;          /* Protocol code */
  uint8_t  ifunction;         /* Index to string identifying the function */
};

#define USB_SIZEOF_IADDESC 8

/* Microsoft OS function descriptor.
 * This can be used to request a specific driver (such as WINUSB) to be
 * loaded on Windows. Unlike other descriptors, it is requested by a special
 * request USB_REQ_GETMSFTOSDESCRIPTOR.
 * More details:
 *       https://msdn.microsoft.com/en-us/windows/hardware/gg463179
 * And excellent explanation:
 *       https://github.com/pbatard/libwdi/wiki/WCID-Devices
 *
 * The device will have exactly one "Extended Compat ID Feature Descriptor",
 * which may contain multiple "Function Descriptors" associated with
 * different interfaces.
 */

struct usb_msft_os_function_desc_s
{
  uint8_t firstif;           /* Index of first associated interface */
  uint8_t nifs;              /* Number of associated interfaces */
  uint8_t compatible_id[8];  /* COMPATIBLE_ID of the driver to load */
  uint8_t sub_id[8];         /* SUB_COMPATIBLE_ID of the driver */
  uint8_t reserved[6];
};

struct usb_msft_os_feature_desc_s
{
  uint8_t len[4];            /* Descriptor length */
  uint8_t version[2];        /* Descriptor syntax version, 0x0100 */
  uint8_t index[2];          /* Set to 4 for "extended compat ID descriptors" */
  uint8_t count;             /* Number of function sections */
  uint8_t reserved[7];
  struct usb_msft_os_function_desc_s function[1];
};

/* Microsoft OS extended property descriptor.
 * This can be used to set specific registry values, such as interface GUID
 * for a device. It is requested per-interface by special request
 * USB_REQ_GETMSFTOSDESCRIPTOR.
 *
 * The interface will have one extended properties descriptor, which may
 * contain multiple properties inside it.
 */

struct usb_msft_os_extprop_hdr_s
{
  uint8_t len[4];            /* Descriptor length */
  uint8_t version[2];        /* Descriptor syntax version, 0x0100 */
  uint8_t index[2];          /* Set to 5 for "extended property descriptors" */
  uint8_t count[2];          /* Number of property sections */

  /* The properties are appended after the header and follow this format:
   * uint8_t prop_len[4];
   * uint8_t data_type[4];
   * uint8_t name_len[2];
   * uint8_t name[name_len];
   * uint8_t data_len[4];
   * uint8_t data[data_len];
   */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions Definitions
 ****************************************************************************/

#endif /* __INCLUDE_NUTTX_USB_USB_H */
