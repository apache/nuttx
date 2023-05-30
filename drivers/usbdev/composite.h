/****************************************************************************
 * drivers/usbdev/composite.h
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

#ifndef __DRIVERS_USBDEV_COMPOSITE_H
#define __DRIVERS_USBDEV_COMPOSITE_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/param.h>
#include <sys/types.h>
#include <stdint.h>

#include <nuttx/usb/usb.h>
#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

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

#ifndef CONFIG_COMPOSITE_COMPOSITE
#  ifndef CONFIG_COMPOSITE_VENDORID
#    warning "CONFIG_COMPOSITE_VENDORID not defined"
#    define CONFIG_COMPOSITE_VENDORID 0x03eb
#  endif

#  ifndef CONFIG_COMPOSITE_PRODUCTID
#    warning "CONFIG_COMPOSITE_PRODUCTID not defined"
#    define CONFIG_COMPOSITE_PRODUCTID 0x2022
#  endif

#  ifndef CONFIG_COMPOSITE_VERSIONNO
#    define CONFIG_COMPOSITE_VERSIONNO (0x0101)
#  endif

#  ifndef CONFIG_COMPOSITE_VENDORSTR
#    warning "No Vendor string specified"
#    define CONFIG_COMPOSITE_VENDORSTR  "NuttX"
#  endif

#  ifndef CONFIG_COMPOSITE_PRODUCTSTR
#    warning "No Product string specified"
#    define CONFIG_COMPOSITE_PRODUCTSTR "Composite Device"
#  endif

#  undef CONFIG_COMPOSITE_SERIALSTR
#  define CONFIG_COMPOSITE_SERIALSTR "0101"
#endif

#undef CONFIG_COMPOSITE_CONFIGSTR
#define CONFIG_COMPOSITE_CONFIGSTR "Composite"

#ifdef CONFIG_USBDEV_SELFPOWERED
#  define COMPOSITE_SELFPOWERED USB_CONFIG_ATTR_SELFPOWER
#else
#  define COMPOSITE_SELFPOWERED       (0)
#endif

#ifdef CONFIG_USBDEV_REMOTEWAKEUP
#  define COMPOSITE_REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define COMPOSITE_REMOTEWAKEUP      (0)
#endif

#define NUM_DEVICES_TO_HANDLE         (8)

/* Descriptors **************************************************************/

/* These settings are not modifiable via the NuttX configuration */

#define COMPOSITE_CONFIGIDNONE        (0)  /* Config ID = 0 means to return to address mode */
#define COMPOSITE_CONFIGID            (1)  /* The only supported configuration ID */

/* String language */

#define COMPOSITE_STR_LANGUAGE        (0x0409) /* en-us */

/* Descriptor strings */

#define COMPOSITE_MANUFACTURERSTRID   (1)
#define COMPOSITE_PRODUCTSTRID        (2)
#define COMPOSITE_SERIALSTRID         (3)
#define COMPOSITE_CONFIGSTRID         (4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct composite_devobj_s
{
  /* Device description given by the user code in the dynamic
   * configuration.
   */

  struct composite_devdesc_s compdesc;

  /* Pointer to device class */

  FAR struct usbdevclass_driver_s *dev;
};

/* This structure describes the internal state of the driver */

struct composite_dev_s
{
  FAR struct usbdev_s      *usbdev;      /* usbdev driver pointer */
  uint8_t                   config;      /* Configuration number */
  FAR struct usbdev_req_s  *ctrlreq;     /* Allocated control request */
  uint8_t                   ndevices;    /* Num devices in this composite device */
  int                       cfgdescsize; /* Total size of the configuration descriptor: */
  int                       ninterfaces; /* The total number of interfaces in this composite device */

  struct composite_devobj_s device[NUM_DEVICES_TO_HANDLE]; /* Device class object */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern const char g_compvendorstr[];
extern const char g_compproductstr[];
extern const char g_compserialstr[];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: composite_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int composite_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);

/****************************************************************************
 * Name: composite_getepdesc
 *
 * Description:
 *   Return a pointer to the composite device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_COMPOSITE_COMPOSITE
FAR const struct usb_devdesc_s *composite_getdevdesc(void);
#endif

/****************************************************************************
 * Name: composite_mkcfgdesc
 *
 * Description:
 *   Construct the composite configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t composite_mkcfgdesc(FAR struct composite_dev_s *priv,
                            FAR uint8_t *buf, uint8_t speed, uint8_t type);
#else
int16_t composite_mkcfgdesc(FAR struct composite_dev_s *priv,
                            FAR uint8_t *buf);
#endif

/****************************************************************************
 * Name: composite_getqualdesc
 *
 * Description:
 *   Return a pointer to the composite qual descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
FAR const struct usb_qualdesc_s *composite_getqualdesc(void);
#endif

#endif /* CONFIG_USBDEV_COMPOSITE */
#endif /* __DRIVERS_USBDEV_COMPOSITE_H */
