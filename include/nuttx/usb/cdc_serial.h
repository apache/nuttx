/****************************************************************************
 * include/nuttx/usb/cdc_serial.h
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
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
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_USB_CDC_SERIAL_H
#define __INCLUDE_NUTTX_USB_CDC_SERIAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/usb/usb.h>

/****************************************************************************
 * Preprocessor definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* EP0 max packet size */

#ifndef CONFIG_CDCSER_EP0MAXPACKET
#  define CONFIG_CDCSER_EP0MAXPACKET 8
#endif

/* Endpoint number and size (in bytes) of the CDC serial device-to-host (IN)
 * notification interrupt endpoint.
 */

#ifndef CONFIG_CDCSER_EPINTIN
#  define CONFIG_CDCSER_EPINTIN 2
#endif

#ifndef CONFIG_CDCSER_EPINTIN_FSSIZE
#  define CONFIG_CDCSER_EPINTIN_FSSIZE 8
#endif

#ifndef CONFIG_CDCSER_EPINTIN_HSSIZE
#  define CONFIG_CDCSER_EPINTIN_HSSIZE 8
#endif

/* Endpoint number and size (in bytes) of the CDC device-to-host (IN) data
 * bulk endpoint.  NOTE that difference sizes may be selected for full (FS)
 * or high speed (HS) modes.
 */

#ifndef CONFIG_CDCSER_EPBULKIN
#  define CONFIG_CDCSER_EPBULKIN 3
#endif

#ifndef CONFIG_CDCSER_EPBULKIN_FSSIZE
#  define CONFIG_CDCSER_EPBULKIN_FSSIZE 64
#endif

#ifndef CONFIG_CDCSER_EPBULKIN_HSSIZE
#  define CONFIG_CDCSER_EPBULKIN_HSSIZE 512
#endif

/* Endpoint number and size (in bytes) of the CDC host-to-device (OUT) data
 * bulk endpoint.  NOTE that difference sizes may be selected for full (FS)
 * or high speed (HS) modes.
 */

#ifndef CONFIG_CDCSER_EPBULKOUT
#  define CONFIG_CDCSER_EPBULKOUT 4
#endif

#ifndef CONFIG_CDCSER_EPBULKOUT_FSSIZE
#  define CONFIG_CDCSER_EPBULKOUT_FSSIZE 64
#endif

#ifndef CONFIG_CDCSER_EPBULKOUT_HSSIZE
#  define CONFIG_CDCSER_EPBULKOUT_HSSIZE 512
#endif

/* Number of requests in the write queue */

#ifndef CONFIG_CDCSER_NWRREQS
#  define CONFIG_CDCSER_NWRREQS 4
#endif

/* Number of requests in the read queue */

#ifndef CONFIG_CDCSER_NRDREQS
#  define CONFIG_CDCSER_NRDREQS 4
#endif

/* TX/RX buffer sizes */

#ifndef CONFIG_CDCSER_RXBUFSIZE
#  define CONFIG_CDCSER_RXBUFSIZE 256
#endif

#ifndef CONFIG_CDCSER_TXBUFSIZE
#  define CONFIG_CDCSER_TXBUFSIZE 256
#endif

/* Vendor and product IDs and strings */

#ifndef CONFIG_CDCSER_VENDORID
#  define CONFIG_CDCSER_VENDORID  0x03eb
#endif

#ifndef CONFIG_CDCSER_PRODUCTID
#  define CONFIG_CDCSER_PRODUCTID 0x204b
#endif

#ifndef CONFIG_CDCSER_VENDORSTR
#  define CONFIG_CDCSER_VENDORSTR  "NuttX"
#endif

#ifndef CONFIG_CDCSER_PRODUCTSTR
#  define CONFIG_CDCSER_PRODUCTSTR "USBdev Serial"
#endif

#undef CONFIG_CDCSER_SERIALSTR
#define CONFIG_CDCSER_SERIALSTR "0"

#undef CONFIG_CDCSER_CONFIGSTR
#define CONFIG_CDCSER_CONFIGSTR "Bulk"

/* USB Controller */

#ifndef CONFIG_USBDEV_SELFPOWERED
#  define SELFPOWERED USB_CONFIG_ATT_SELFPOWER
#else
#  define SELFPOWERED (0)
#endif

#ifndef CONFIG_USBDEV_REMOTEWAKEUP
#  define REMOTEWAKEUP USB_CONFIG_ATTR_WAKEUP
#else
#  define REMOTEWAKEUP (0)
#endif

#ifndef CONFIG_USBDEV_MAXPOWER
#  define CONFIG_USBDEV_MAXPOWER 100
#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
# define EXTERN extern "C"
extern "C" {
#else
# define EXTERN extern
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __INCLUDE_NUTTX_USB_CDC_SERIAL_H */
