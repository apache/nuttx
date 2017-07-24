/****************************************************************************
 * drivers/usbdev/composite.h
 *
 *   Copyright (C) 2011-2012, 2017 Gregory Nutt. All rights reserved.
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

#ifndef __DRIVERS_USBDEV_COMPOSITE_H
#define __DRIVERS_USBDEV_COMPOSITE_H 1

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

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
# endif

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

#define NUM_DEVICES_TO_HANDLE         (4)

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

/* Everpresent MIN/MAX macros ***********************************************/

#ifndef MIN
#  define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MAX
#  define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

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
