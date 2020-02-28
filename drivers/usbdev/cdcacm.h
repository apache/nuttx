/****************************************************************************
 * drivers/usbdev/cdcacm.h
 *
 *   Copyright (C) 2011-2012, 2015, 2017 Gregory Nutt. All rights reserved.
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

#ifndef __DRIVERS_USBDEV_CDCACM_H
#define __DRIVERS_USBDEV_CDCACM_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/cdc.h>
#include <nuttx/usb/usbdev_trace.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* If the serial device is configured as part of a composite device than both
 * CONFIG_USBDEV_COMPOSITE and CONFIG_CDCACM_COMPOSITE must be defined.
 */

#ifndef CONFIG_USBDEV_COMPOSITE
#  undef CONFIG_CDCACM_COMPOSITE
#endif

#if defined(CONFIG_CDCACM_COMPOSITE) && !defined(CONFIG_CDCACM_STRBASE)
#  define CONFIG_CDCACM_STRBASE    (0)
#endif

#if defined(CONFIG_CDCACM_COMPOSITE) && !defined(CONFIG_COMPOSITE_IAD)
#  warning "CONFIG_COMPOSITE_IAD may be needed"
#endif

/* Packet and request buffer sizes */

#ifndef CONFIG_CDCACM_COMPOSITE
#  ifndef CONFIG_CDCACM_EP0MAXPACKET
#    define CONFIG_CDCACM_EP0MAXPACKET 64
#  endif
#endif

/* Interface IDs.  If the serial driver is built as a component of a composite
 * device, then the interface IDs may need to be offset.
 */

#ifndef CONFIG_CDCACM_COMPOSITE
#  undef CONFIG_CDCACM_IFNOBASE
#  define CONFIG_CDCACM_IFNOBASE   (0)
#endif

#ifndef CONFIG_CDCACM_IFNOBASE
#  define CONFIG_CDCACM_IFNOBASE   (0)
#endif

/* Descriptors **************************************************************/

/* These settings are not modifiable via the NuttX configuration */

#define CDC_VERSIONNO              0x0110   /* CDC version number 1.10 (BCD) */
#define CDCACM_CONFIGIDNONE        (0)      /* Config ID means to return to address mode */

/* Interface IDs:
 *
 * CDCACM_NINTERFACES              Two interfaces
 * CDCACM_NOTIFID                  ID of the notifier interface
 * CDCACM_NOTALTIFID               No alternate for the notifier interface
 * CDCACM_DATAIFID                 ID of the data interface
 * CDCACM_DATAALTIFID              No alternate for the data interface
 */

#define CDCACM_NOTALTIFID          (0)
#define CDCACM_DATAALTIFID         (0)

/* Buffer big enough for any of our descriptors (the config descriptor is the
 * biggest).
 */

#define CDCACM_MXDESCLEN           (64)
#define CDCACM_MAXSTRLEN           (CDCACM_MXDESCLEN-2)

/* Device descriptor values */

#define CDCACM_VERSIONNO           (0x0101) /* Device version number 1.1 (BCD) */

/* String language */

#define CDCACM_STR_LANGUAGE        (0x0409) /* en-us */

/* Descriptor strings.  If there serial device is part of a composite device
 * then the manufacturer, product, and serial number strings will be provided
 * by the composite logic.
 */

#ifndef CONFIG_CDCACM_COMPOSITE
#  define CDCACM_MANUFACTURERSTRID (1)
#  define CDCACM_PRODUCTSTRID      (2)
#  define CDCACM_SERIALSTRID       (3)
#  define CDCACM_CONFIGSTRID       (4)

#  define CDCACM_LASTBASESTRID     (4)
#  define CDCACM_STRBASE           (0)
#else
#  define CDCACM_STRBASE           CONFIG_CDCACM_STRBASE
#  define CDCACM_LASTBASESTRID     CONFIG_CDCACM_STRBASE
#endif

/* These string IDs only exist if a user-defined string is provided */

#ifdef CONFIG_CDCACM_NOTIFSTR
#  define CDCACM_NOTIFSTRID        (CDCACM_LASTBASESTRID + 1)
#else
#  define CDCACM_NOTIFSTRID        CDCACM_LASTBASESTRID
#endif

#ifdef CONFIG_CDCACM_DATAIFSTR
#  define CDCACM_DATAIFSTRID       (CDCACM_NOTIFSTRID + 1)
#else
#  define CDCACM_DATAIFSTRID       CDCACM_NOTIFSTRID
#endif

#define CDCACM_LASTSTRID           CDCACM_DATAIFSTRID
#define CDCACM_NSTRIDS             (CDCACM_LASTSTRID - CDCACM_STRBASE)

/* Endpoint configuration ****************************************************/

#define CDCACM_MKEPINTIN(desc)     (USB_DIR_IN | (desc)->epno[CDCACM_EP_INTIN_IDX])
#define CDCACM_EPINTIN_ATTR        (USB_EP_ATTR_XFER_INT)

#define CDCACM_MKEPBULKIN(desc)    (USB_DIR_IN | (desc)->epno[CDCACM_EP_BULKIN_IDX])
#define CDCACM_EPOUTBULK_ATTR      (USB_EP_ATTR_XFER_BULK)

#define CDCACM_MKEPBULKOUT(desc)   ((desc)->epno[CDCACM_EP_BULKOUT_IDX])
#define CDCACM_EPINBULK_ATTR       (USB_EP_ATTR_XFER_BULK)

/* Device driver definitions ************************************************/

/* A CDC/ACM device is specific by a minor number in the range of 0-255.
 * This maps to a character device at /dev/ttyACMx, x=0..255.
 */

#define CDCACM_DEVNAME_FORMAT      "/dev/ttyACM%d"
#define CDCACM_DEVNAME_SIZE        16

/* Misc Macros **************************************************************/

/* MIN/MAX macros */

#ifndef MIN
#  define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef MAX
#  define MAX(a,b) ((a)>(b)?(a):(b))
#endif

/* Trace values *************************************************************/

#define CDCACM_CLASSAPI_SETUP       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SETUP)
#define CDCACM_CLASSAPI_SHUTDOWN    TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SHUTDOWN)
#define CDCACM_CLASSAPI_ATTACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_ATTACH)
#define CDCACM_CLASSAPI_DETACH      TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_DETACH)
#define CDCACM_CLASSAPI_IOCTL       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_IOCTL)
#define CDCACM_CLASSAPI_RECEIVE     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RECEIVE)
#define CDCACM_CLASSAPI_RXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RXINT)
#define CDCACM_CLASSAPI_RXAVAILABLE TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_RXAVAILABLE)
#define CDCACM_CLASSAPI_SEND        TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_SEND)
#define CDCACM_CLASSAPI_TXINT       TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXINT)
#define CDCACM_CLASSAPI_TXREADY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXREADY)
#define CDCACM_CLASSAPI_TXEMPTY     TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_TXEMPTY)
#define CDCACM_CLASSAPI_FLOWCONTROL TRACE_EVENT(TRACE_CLASSAPI_ID, USBSER_TRACECLASSAPI_FLOWCONTROL)

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum cdcacm_epdesc_e
{
  CDCACM_EPINTIN = 0,  /* Interrupt IN endpoint descriptor */
  CDCACM_EPBULKOUT,    /* Bulk OUT endpoint descriptor */
  CDCACM_EPBULKIN      /* Bulk IN endpoint descriptor */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: cdcacm_mkstrdesc
 *
 * Description:
 *   Construct a string descriptor
 *
 ****************************************************************************/

int cdcacm_mkstrdesc(uint8_t id, struct usb_strdesc_s *strdesc);

/****************************************************************************
 * Name: cdcacm_getdevdesc
 *
 * Description:
 *   Return a pointer to the raw device descriptor
 *
 ****************************************************************************/

#ifndef CONFIG_CDCACM_COMPOSITE
FAR const struct usb_devdesc_s *cdcacm_getdevdesc(void);
#endif

/****************************************************************************
 * Name: cdcacm_copy_epdesc
 *
 * Description:
 *   Copies the requested Endpoint Description into the buffer given.
 *   Returns the number of Bytes filled in (sizeof(struct usb_epdesc_s)).
 *
 ****************************************************************************/

int cdcacm_copy_epdesc(enum cdcacm_epdesc_e epid,
                       FAR struct usb_epdesc_s *epdesc,
                       FAR struct usbdev_devinfo_s *devinfo,
                       bool hispeed);

/****************************************************************************
 * Name: cdcacm_mkcfgdesc
 *
 * Description:
 *   Construct the configuration descriptor
 *
 ****************************************************************************/

#ifdef CONFIG_USBDEV_DUALSPEED
int16_t cdcacm_mkcfgdesc(FAR uint8_t *buf,
                         FAR struct usbdev_devinfo_s *devinfo,
                         uint8_t speed, uint8_t type);
#else
int16_t cdcacm_mkcfgdesc(FAR uint8_t *buf,
                         FAR struct usbdev_devinfo_s *devinfo);
#endif

/****************************************************************************
 * Name: cdcacm_getqualdesc
 *
 * Description:
 *   Return a pointer to the raw qual descriptor
 *
 ****************************************************************************/

#if !defined(CONFIG_CDCACM_COMPOSITE) && defined(CONFIG_USBDEV_DUALSPEED)
FAR const struct usb_qualdesc_s *cdcacm_getqualdesc(void);
#endif

#endif /* __DRIVERS_USBDEV_CDCACM_H */
